#!/usr/bin/env python3
"""Click-to-track commissioning UI. Robot motion requires an explicit Robot mode."""
import argparse
import json
import math
from pathlib import Path
import time
import uuid
import cv2
import numpy as np
from gemini_config import WIDTH, HEIGHT, configure_close_range, stream_profiles
from local_tracker import LocalTracker
from model_selection import (DEFAULT_DETECTOR_PYTHON, DetectionCenterFusion,
                             ModelSelector, fuse_model_observation, replay_selection,
                             observation_is_fresh)
from projection_preview import CaptureClock, aligned_rgbd, color_bgr
from tracking_diagnostics import TrackingDiagnostics
from wrist_projection import boot_id, calibration, FRAME

STATE = 'robot:wrist:state:v1'
OBSERVATION = 'robot:wrist:observation:v1'
COMMAND = 'robot:wrist:command:v1'


class StartupFramesUnavailable(RuntimeError):
    """The first synchronized RGB-D frame never arrived after stream start."""


class ClickSession:
    """UI ownership gate, independent of SDK and Redis for offline tests."""
    def __init__(self, digest):
        self.digest = digest
        self.producer = uuid.uuid4().hex
        self.runtime = ''
        self.target = ''
        self.sequence = 0
        self.following = False
        self.started = False
        self.last_request_sequence = 0
        self.last_state_sequence = 0
        self.state = None

    def accept_state(self, state, now):
        expected = (('version', 1), ('scope', 'click_follow'),
                    ('boot_id', boot_id()), ('calibration_sha256', self.digest),
                    ('frame', FRAME), ('length_unit', 'm'))
        for field, value in expected:
            if state.get(field) != value:
                raise ValueError('Robot state %s mismatch' % field)
        if state.get('valid') is not True:
            raise ValueError('Robot state invalid')
        stamp = state.get('timestamp_monotonic_ns')
        if not isinstance(stamp, int):
            raise ValueError('Robot state timestamp missing/invalid')
        age_ns = now - stamp
        if not 0 <= age_ns <= 200_000_000:
            raise ValueError('Robot state age %.1f ms outside 0..200 ms' % (age_ns / 1e6))
        runtime = state['runtime_session_id']
        if not isinstance(runtime, str) or not runtime:
            raise ValueError('missing Robot session')
        changed = bool(self.runtime and self.runtime != runtime)
        if changed:
            self.following = False
            self.started = False
            self.target = ''
            self.last_request_sequence = 0
            self.last_state_sequence = 0
        if state['sequence'] <= self.last_state_sequence:
            return changed
        self.runtime = runtime
        self.last_state_sequence = state['sequence']
        self.state = state
        self.started = self.started or state.get("resume_required", False)
        if (self.following and state.get('request_producer_id') == self.producer
                and state.get('request_sequence', 0) >= self.last_request_sequence
                and state.get('follow_state') != 'following'):
            self.following = False
            self.target = ''
            changed = True
        return changed

    def select(self):
        if self.following:
            raise ValueError('pause before selecting another target')
        self.target = uuid.uuid4().hex

    def envelope(self):
        self.sequence += 1
        return dict(version=1, runtime_session_id=self.runtime, producer_id=self.producer,
                    target_id=self.target, sequence=self.sequence, timestamp_monotonic_ns=time.monotonic_ns(),
                    boot_id=boot_id(), calibration_sha256=self.digest, frame=FRAME, length_unit='m')

    def request(self, action, valid):
        if action in ('begin', 'resume'):
            if not self.runtime or not self.state or not self.target or not valid:
                raise ValueError('select a valid target and wait for Robot before starting')
            if time.monotonic_ns()-self.state['timestamp_monotonic_ns'] > 200_000_000:
                raise ValueError('Robot state stale')
            if self.started and action != 'resume':
                raise ValueError('use r after pause/loss and selecting a new target')
            if not self.started and action != 'begin':
                raise ValueError('use b for the first start')
            self.following = True
            self.started = True
        elif action in ('pause', 'end'):
            self.following = False
        packet = self.envelope()
        packet['action'] = action
        self.last_request_sequence = packet['sequence']
        return packet


class ModelStart:
    """One b press authorizes one fresh model selection and acknowledged start."""
    def __init__(self):
        self.cancel()

    def cancel(self):
        self.runtime = ''
        self.old_target = ''

    def queue(self, session, now):
        state = session.state or {}
        if session.following:
            raise ValueError('already following; pause before selecting another target')
        if (not session.runtime or not 0 <= now-state.get('timestamp_monotonic_ns', 0) <= 200_000_000
                or state.get('follow_state') != 'hold'
                or str(state.get('control_state', '')).lower() == 'fault'):
            raise ValueError('wait for a fresh Robot Hold before starting')
        self.runtime = session.runtime
        self.old_target = session.target

    def poll(self, session, result, now, *, selecting):
        if not self.runtime:
            return None
        state = session.state or {}
        if (session.runtime != self.runtime or session.following
                or not 0 <= now-state.get('timestamp_monotonic_ns', 0) <= 200_000_000
                or state.get('follow_state') != 'hold'
                or str(state.get('control_state', '')).lower() == 'fault'):
            self.cancel()
            return None
        if selecting:
            return None
        if (not session.target or session.target == self.old_target
                or not observation_is_fresh(result, now)):
            self.cancel()
            return None
        # Observation is published by the caller before polling. Wait until Robot
        # echoes its target identity, so begin/resume cannot race the observation.
        if state.get('target_id') != session.target:
            return None
        packet = session.request('resume' if session.started else 'begin', True)
        self.cancel()
        return packet


class ModelRecovery:
    """Keep operator follow intent across visual loss, with an acknowledged pause."""
    VISUAL_HOLDS = {'operator_pause', 'operator_paused', 'invalid wrist observation',
                    'wrist_pose_time_unmatched', 'wrist_observation_missing_stale_or_mismatched'}

    def __init__(self):
        self.cancel()

    def cancel(self):
        self.enabled = self.waiting = False
        self.runtime = self.old_target = ''
        self.pause_sequence = self.pause_state_sequence = 0

    def arm(self, session):
        self.cancel()
        self.enabled = True
        self.runtime = session.runtime

    def accept_state(self, session, state, now):
        """Turn an owned Robot timing Hold into the acknowledged recovery path."""
        was_following = session.following
        old_target = session.target
        previous_sequence = session.last_state_sequence
        previous_runtime = session.runtime
        changed = session.accept_state(state, now)
        pause = None
        if (self.enabled and not self.waiting and was_following and old_target
                and session.runtime == previous_runtime == self.runtime
                and session.state is state and state['sequence'] > previous_sequence
                and state.get('follow_state') == 'hold'
                and state.get('follow_reason') == 'wrist_pose_time_unmatched'
                and str(state.get('control_state', '')).lower() != 'fault'
                and state.get('target_id') == old_target
                and state.get('request_producer_id') == session.producer
                and state.get('request_sequence', 0) >= session.last_request_sequence):
            # accept_state clears the held target; carry its identity on our pause
            # so recovery still requires a different, acknowledged new target.
            pause = self.pause_for_loss(session, target_id=old_target)
        self.check_state(session, now)
        return changed, pause

    def pause_for_loss(self, session, *, target_id=None):
        packet = session.request('pause', False)
        if target_id is not None:
            packet['target_id'] = target_id
        self.waiting = self.enabled
        self.pause_sequence = packet['sequence']
        self.pause_state_sequence = session.last_state_sequence
        self.old_target = packet['target_id']
        return packet

    def check_state(self, session, now):
        if not self.enabled:
            return
        state = session.state
        if (session.runtime != self.runtime or not state
                or not 0 <= now-state['timestamp_monotonic_ns'] <= 200_000_000
                or str(state.get('control_state', '')).lower() == 'fault'):
            self.cancel()
            return
        if self.waiting:
            if state['sequence'] <= self.pause_state_sequence:
                return
            if (state.get('follow_state') == 'hold'
                    and state.get('follow_reason') not in self.VISUAL_HOLDS):
                self.cancel()
            elif (state.get('request_sequence', 0) >= self.pause_sequence
                  and state.get('request_producer_id') != session.producer):
                self.cancel()
        elif not session.following:
            self.cancel()

    def ready(self, session, now):
        self.check_state(session, now)
        state = session.state or {}
        return (self.enabled and self.waiting
                and state.get('follow_state') == 'hold'
                and state.get('request_producer_id') == session.producer
                and state.get('request_sequence') == self.pause_sequence
                and state.get('follow_reason') in self.VISUAL_HOLDS)

    def resume(self, session, result, now):
        if (not self.ready(session, now) or not session.target
                or session.target == self.old_target
                or session.state.get('target_id') != session.target
                or not observation_is_fresh(result, now)):
            return None
        packet = session.request('resume', True)
        self.waiting = False
        return packet


def draw_detection_boxes(image, response, now_ns, *, max_age_ns=500_000_000):
    """Draw recent model boxes without treating them as tracked or Robot-valid."""
    if not response or response.get('image_size') != [image.shape[1], image.shape[0]]:
        return False
    stamp = response.get('capture_monotonic_ns')
    if not isinstance(stamp, int) or not 0 <= now_ns - stamp <= max_age_ns:
        return False
    for box in response.get('detections', []):
        try:
            x, y, width, height, confidence = (
                float(box[key]) for key in ('x', 'y', 'width', 'height', 'confidence'))
            if not all(map(math.isfinite, (x, y, width, height, confidence))):
                continue
            if width <= 0 or height <= 0 or not 0 <= confidence <= 1:
                continue
            x0, y0 = max(0, round(x-width/2)), max(0, round(y-height/2))
            x1, y1 = min(image.shape[1]-1, round(x+width/2)), min(image.shape[0]-1, round(y+height/2))
            if x0 > x1 or y0 > y1:
                continue
            color = (0, 165, 255)
            cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)
            label = '%s %.2f' % (box.get('class', '?'), confidence)
            cv2.putText(image, label, (x0, max(12, y0-4)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='CV2L360000HZ')
    parser.add_argument('--calibration', type=Path, default=Path(__file__).with_name('gemini305_to_rm75_armtip.json'))
    parser.add_argument('--redis-port', type=int, default=7777)
    parser.add_argument('--model-id', help='Enable model P0 selection with d; disables mouse selection')
    parser.add_argument('--detector-python', type=Path, default=DEFAULT_DETECTOR_PYTHON)
    parser.add_argument('--model-class', default='carotid')
    parser.add_argument('--model-confidence', type=float, default=.4, help='Model selection confidence (default: 0.4)')
    parser.add_argument('--log', type=Path, default=Path(__file__).resolve().parent / 'log' / ('wrist_click_%d.jsonl' % time.time_ns()))
    args = parser.parse_args(argv)
    model_confidence = args.model_confidence
    if not 0 <= model_confidence <= 1:
        parser.error('--model-confidence must be in [0, 1]')
    if args.model_id and not args.detector_python.is_file():
        parser.error('detector Python environment not found')
    return args


def main():
    args = parse_args()
    model_confidence = args.model_confidence
    _, digest = calibration(args.calibration, 'T_armtip_camera', 'gemini305_color_optical_to_rm75_armtip')
    import pyorbbecsdk as ob
    sdk_log = Path(__file__).resolve().parent / 'log' / 'sdk'
    sdk_log.mkdir(parents=True, exist_ok=True)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    ob.Context.set_logger_to_file(ob.OBLogLevel.ERROR, str(sdk_log))
    context = ob.Context()
    device = context.query_devices().get_device_by_serial_number(args.serial)
    if not device.is_global_timestamp_supported():
        raise ValueError('SDK capture clock unsupported')
    camera_settings = configure_close_range(device, ob)
    device.enable_global_timestamp(True)
    pipeline = ob.Pipeline(device)
    config = ob.Config()
    color_profile, depth_profile = stream_profiles(pipeline, ob)
    config.enable_stream(color_profile); config.enable_stream(depth_profile)
    intr, dist = color_profile.get_intrinsic(), color_profile.get_distortion()
    if dist.model not in (ob.OBCameraDistortionModel.NONE, ob.OBCameraDistortionModel.BROWN_CONRADY,
                           ob.OBCameraDistortionModel.BROWN_CONRADY_K6):
        raise ValueError('unsupported distortion model')
    k = np.array([[intr.fx, 0., intr.cx], [0., intr.fy, intr.cy], [0., 0., 1.]])
    d = np.array([dist.k1, dist.k2, dist.p1, dist.p2, dist.k3, dist.k4, dist.k5, dist.k6])
    if dist.model == ob.OBCameraDistortionModel.NONE: d[:] = 0
    align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
    tracker, session, clock = LocalTracker(), ClickSession(digest), CaptureClock()
    selector = ModelSelector(args.detector_python, args.model_id, args.model_class,
                             model_confidence, 0,
                             inference_confidence=.4) if args.model_id else None
    fusion = DetectionCenterFusion() if selector is not None else None
    import redis
    client = redis.Redis(host='127.0.0.1', port=args.redis_port, socket_timeout=.05, socket_connect_timeout=.1)
    subscriber = client.pubsub(ignore_subscribe_messages=True)
    subscriber.subscribe(STATE)
    clicked = []
    window = 'Wrist click follow'
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, lambda event, x, y, flags, param:
                        clicked.append((x, y)) if not selector and event == cv2.EVENT_LBUTTONDOWN else None)
    started = False
    last_heartbeat = 0.
    user_notice = ""
    model_catching_up = False
    model_catchup_started = 0
    latest_detection = None
    selection_requested = False
    selection_pending = False
    recovery = ModelRecovery()
    model_start = ModelStart()
    next_recovery_at = 0.
    last_preview_request = 0.
    last_model_confirmed_ns = None
    last_state_error = ""
    last_published_capture = {}
    with args.log.open('x') as log:
        def record(item):
            log.write(json.dumps(item, allow_nan=False)+'\n'); log.flush()
        def publish(channel, item):
            record(dict(type='redis_out', channel=channel, payload=item))
            if client is not None:
                try: client.publish(channel, json.dumps(item, allow_nan=False))
                except Exception as error:
                    session.following = False
                    tracker.lose('Redis publication failed')
                    recovery.cancel()
                    model_start.cancel()
                    record(dict(type='redis_error', reason=str(error)))
        def publish_observation(result):
            if not session.runtime or not session.target:
                return
            capture = result.get('capture_monotonic_ns', 0)
            if result.get('valid') and last_published_capture.get(session.target) == capture:
                return
            packet = session.envelope()
            packet.update(result)
            packet.setdefault('capture_monotonic_ns', 0)
            publish(OBSERVATION, packet)
            if result.get('valid'):
                last_published_capture.clear()
                last_published_capture[session.target] = capture
        record(camera_settings)
        diagnostics = TrackingDiagnostics(args.log, record)
        record(dict(type='configuration', mode='click_follow', serial=args.serial, calibration_sha256=digest,
                    intrinsic=k.tolist(), distortion=d.tolist(), sdk=str(ob.__version__), independent_validation=False,
                    selection='model' if selector else 'click', model_id=args.model_id,
                    vision_profile='standard',
                    model_confidence=model_confidence))
        try:
            pipeline.start(config); started = True; pipeline.enable_frame_sync()
            # Allow the first synchronized frames to arrive after SDK/USB startup.
            last_complete = time.monotonic()
            first_complete = False
            while True:
                if subscriber is not None:
                    try:
                        for _ in range(100):
                            msg = subscriber.get_message(timeout=0)
                            if msg is None: break
                            if msg['type'] != 'message': continue
                            previous_runtime = session.runtime
                            changed, recovery_pause = recovery.accept_state(
                                session, json.loads(msg['data']), time.monotonic_ns())
                            last_state_error = ""
                            if session.runtime != previous_runtime:
                                record(dict(type='robot_state_connected', runtime_session_id=session.runtime,
                                            sequence=session.last_state_sequence))
                            if recovery_pause is not None:
                                publish(COMMAND, recovery_pause)
                                selection_requested = selection_pending = model_catching_up = False
                                next_recovery_at = time.monotonic()+.75
                                record(dict(type='model_recovery_wait',
                                            reason='wrist_pose_time_unmatched', source='robot'))
                                user_notice = 'Robot timing Hold; waiting for pause acknowledgement and new model center'
                            if changed:
                                tracker.lose('Robot held/restarted: '+str(session.state.get('follow_reason') or 'session changed')+'; select again', force=True)
                    except Exception as error:
                        if session.following:
                            tracker.lose('Robot state rejected/disconnected')
                            session.following = False
                            session.target = ''
                        recovery.cancel()
                        model_start.cancel()
                        last_state_error = str(error)
                        record(dict(type='state_rejected', reason=last_state_error))
                if session.following and (session.state is None or time.monotonic_ns()-session.state['timestamp_monotonic_ns'] > 200_000_000):
                    tracker.lose('Robot state timeout'); session.following = False; session.target = ''
                recovery.check_state(session, time.monotonic_ns())
                color, depth, missing = aligned_rgbd(pipeline.wait_for_frames(100), align)
                frame = None
                result = dict(valid=False, reason=missing or tracker.reason)
                image = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
                if missing:
                    tracker.lose(missing)
                    startup_limit = 10 if not first_complete else 5
                    if time.monotonic()-last_complete > startup_limit:
                        message = 'no complete RGB-D for %d seconds: %s' % (startup_limit, missing)
                        if not first_complete:
                            raise StartupFramesUnavailable(message)
                        raise ValueError(message)
                else:
                    first_complete = True
                    last_complete = time.monotonic()
                    image = color_bgr(color, ob)
                    try:
                        timestamp = clock.convert(color.get_global_timestamp_us())
                        if abs(color.get_global_timestamp_us()-depth.get_global_timestamp_us()) > 20_000:
                            raise ValueError('RGB-depth capture skew')
                        z = np.frombuffer(depth.get_data(), np.uint16).reshape(depth.get_height(), depth.get_width()).astype(float)*depth.get_depth_scale()/1000
                        if z.shape != image.shape[:2]: raise ValueError('alignment shape mismatch')
                        frame = (image.copy(), z, timestamp)
                        result = tracker.update(image, z, k, d, timestamp)
                        if model_catching_up:
                            if not result.get('valid') or time.monotonic_ns()-model_catchup_started > 2_000_000_000:
                                model_catching_up = False
                                tracker.lose('model catch-up failed or timed out', force=True)
                                result = dict(valid=False, reason='model catch-up failed or timed out')
                                record(dict(type='model_selection_rejected', reason=result['reason']))
                            elif observation_is_fresh(result, time.monotonic_ns()):
                                session.select()
                                last_model_confirmed_ns = time.monotonic_ns()
                                model_catching_up = False
                                record(dict(type='model_selection', target_id=session.target,
                                            current_capture_monotonic_ns=result['capture_monotonic_ns'],
                                            catchup=True))
                                user_notice = ('model P0 selected; waiting for Robot confirmation' if model_start.runtime
                                                           else 'model P0 selected; press b or r to follow')
                        if selector is not None:
                            selector.add_frame(frame)
                            completed = selector.poll()
                            if completed is not None:
                                initial, intervening, response, error = completed
                                if response is not None and response.get('capture_monotonic_ns') == initial[2]:
                                    latest_detection = response
                                if selection_pending:
                                    selection_pending = False
                                    try:
                                        if error:
                                            raise ValueError(error)
                                        if response.get('capture_monotonic_ns') != initial[2]:
                                            raise ValueError('model response frame identity mismatch')
                                        if not response.get('valid'):
                                            raise ValueError(response.get('reason', 'model rejected P0'))
                                        if session.following:
                                            raise ValueError('pause before selecting another target')
                                        result = replay_selection(tracker, initial, intervening,
                                                                  response['prediction'], k, d)
                                        record(dict(type='model_replay', prediction=response['prediction'],
                                                    source_capture_monotonic_ns=initial[2],
                                                    current_capture_monotonic_ns=result['capture_monotonic_ns'],
                                                    replay_frames=len(intervening)))
                                        if observation_is_fresh(result, time.monotonic_ns()):
                                            session.select()
                                            last_model_confirmed_ns = time.monotonic_ns()
                                            record(dict(type='model_selection', target_id=session.target,
                                                        current_capture_monotonic_ns=result['capture_monotonic_ns'],
                                                        catchup=False))
                                            user_notice = ('model P0 selected; waiting for Robot confirmation' if model_start.runtime
                                                           else 'model P0 selected; press b or r to follow')
                                        else:
                                            model_catching_up = True
                                            model_catchup_started = time.monotonic_ns()
                                            user_notice = 'model tracked; waiting for a fresh camera frame'
                                    except (ValueError, TypeError, KeyError) as error:
                                        model_catching_up = False
                                        tracker.lose(error, force=True)
                                        result = dict(valid=False, reason=str(error))
                                        user_notice = str(error)
                                        record(dict(type='model_selection_rejected', reason=str(error)))
                                        if recovery.waiting:
                                            next_recovery_at = time.monotonic()+.75
                                else:
                                    if response is not None:
                                        record(dict(type='model_preview',
                                                    capture_monotonic_ns=initial[2],
                                                    receive_monotonic_ns=time.monotonic_ns(),
                                                    inference_ms=response.get('inference_ms'),
                                                    box_count=len(response.get('detections', [])),
                                                    valid=response.get('valid', False)))
                                    if error:
                                        user_notice = 'model preview: ' + error
                                    if (fusion is not None and tracker.active and session.target
                                            and response is not None and result.get('valid')):
                                        result, applied, reason, confirmed = fuse_model_observation(
                                            tracker, fusion, response, initial[2], frame, result,
                                            session.target, k, d, time.monotonic_ns())
                                        if confirmed:
                                            last_model_confirmed_ns = time.monotonic_ns()
                                        record(dict(type='model_fusion', applied=applied,
                                                    reason=reason, source_capture_monotonic_ns=initial[2],
                                                    current_capture_monotonic_ns=timestamp,
                                                    pixel=result.get('pixel')))
                            recovery_due = (recovery.ready(session, time.monotonic_ns())
                                            and not session.following
                                            and not tracker.active and not model_catching_up
                                            and time.monotonic() >= next_recovery_at)
                            if (selection_requested or recovery_due) and not selector.pending:
                                try:
                                    selector.request(frame, dict(capture_monotonic_ns=frame[2]))
                                    purpose = 'manual' if selection_requested else 'recovery'
                                    selection_requested = False
                                    selection_pending = True
                                    next_recovery_at = time.monotonic()+.75
                                    latest_detection = None
                                    model_catching_up = False
                                    tracker.lose('model detection pending', force=True)
                                    session.target = ''
                                    user_notice = ('recovering P0 from model' if purpose == 'recovery' else
                                                   'model selecting P0 on a fresh frame')
                                    record(dict(type='model_detection_requested', purpose=purpose, capture_monotonic_ns=frame[2]))
                                except ValueError as error:
                                    selection_requested = False
                                    next_recovery_at = time.monotonic()+.75
                                    user_notice = str(error)
                                    record(dict(type='model_selection_rejected', reason=str(error)))
                            elif (not selector.pending and not selection_pending
                                  and time.monotonic()-last_preview_request >= .1):
                                try:
                                    selector.request(frame, dict(capture_monotonic_ns=frame[2]),
                                                     collect_frames=False)
                                    last_preview_request = time.monotonic()
                                except ValueError as error:
                                    user_notice = 'model preview: ' + str(error)
                        if (fusion is not None and tracker.active
                                and session.target and last_model_confirmed_ns is not None
                                and time.monotonic_ns()-last_model_confirmed_ns > 1_000_000_000):
                            tracker.lose('model confirmation missing for 1 second', force=True)
                            result = dict(valid=False, reason=tracker.reason)
                        if time.monotonic_ns()-timestamp > 200_000_000:
                            result = dict(valid=False, reason='tracking computation exceeded observation age')
                    except ValueError as error:
                        tracker.lose(error); result = dict(valid=False, reason=str(error))
                if fusion is not None:
                    if not tracker.active or not session.target:
                        fusion.reset()
                    elif result.get('valid') and observation_is_fresh(result, time.monotonic_ns()):
                        fusion.remember(result, session.target)
                diagnostics.emit(result, tracker)
                publish_observation(result)
                if not result.get('valid') and session.following:
                    packet = (recovery.pause_for_loss(session) if recovery.enabled
                              else session.request('pause', False))
                    publish(COMMAND, packet)
                    tracker.lose(result.get('reason', 'visual tracking lost'), force=True)
                    next_recovery_at = time.monotonic()+.75
                    if recovery.waiting:
                        record(dict(type='model_recovery_wait', reason=result.get('reason')))
                        user_notice = 'visual target lost; waiting for Robot Hold and new model center'
                start_packet = model_start.poll(
                    session, result, time.monotonic_ns(),
                    selecting=selection_requested or selection_pending or model_catching_up)
                if start_packet is not None:
                    recovery.arm(session)
                    publish(COMMAND, start_packet)
                    record(dict(type='model_start', action=start_packet['action'], target_id=session.target))
                    user_notice = 'model target confirmed; follow requested'
                resume_packet = recovery.resume(session, result, time.monotonic_ns())
                if resume_packet is not None:
                    publish(COMMAND, resume_packet)
                    record(dict(type='model_auto_resume', target_id=session.target))
                    user_notice = 'new model center acquired; automatic resume requested'
                if (session.following or recovery.waiting) and time.monotonic()-last_heartbeat >= .1:
                    heartbeat = session.envelope(); heartbeat['action'] = 'heartbeat'
                    heartbeat['target_id'] = session.target or recovery.old_target
                    publish(COMMAND, heartbeat); last_heartbeat = time.monotonic()
                if selector is not None:
                    shown = draw_detection_boxes(image, latest_detection, time.monotonic_ns())
                    status = 'boxes: %d' % len(latest_detection.get('detections', [])) if shown else 'boxes: waiting'
                    cv2.putText(image, status, (10, 175), 0, .45, (0, 165, 255), 1)
                if result.get('valid'):
                    display_pixel, display_position = result['pixel'], result['point_camera_m']
                    cv2.circle(image, tuple(np.floor(display_pixel).astype(int)), 6, (0,255,0), 2)
                    cv2.putText(image, 'p(m): '+str(np.round(display_position,3)), (10,75), 0,.5,(0,255,255),1)
                    cv2.putText(image, 'n: '+str(np.round(result['normal_out_camera'],3)), (10,100),0,.5,(0,255,255),1)
                    quality = result['quality']
                    cv2.putText(image, 'features: %d | plane RMS: %.2f mm | points: %d' % (
                        quality['feature_inliers'], quality['plane_rms_m']*1000, quality['surface_points']),
                        (10,125),0,.45,(0,255,255),1)
                if session.state is None:
                    robot_text = 'offline: ' + (last_state_error or 'waiting for Robot state')
                elif time.monotonic_ns()-session.state['timestamp_monotonic_ns'] > 200_000_000:
                    robot_text = 'offline: Robot state timeout'
                else:
                    robot_text = session.state.get('follow_state','waiting')+': '+session.state.get('follow_reason','')
                cv2.putText(image, 'Robot: '+robot_text[:70], (10,25),0,.45,(0,255,255),1)
                cv2.putText(image, result.get('reason','')[:80],(10,50),0,.45,(0,0,255),1)
                cv2.putText(image, user_notice[:85],(10,150),0,.45,(0,165,255),1)
                instructions = ('b: select + follow | d: select only | p: pause | r: resume | q: exit'
                                if selector else 'click: select | b: begin | p: pause | r: resume | q: exit')
                cv2.putText(image,instructions,(10,465),0,.45,(255,255,255),1)
                cv2.imshow(window,image)
                key = cv2.waitKey(1)&255
                if key == ord('b') and selector is not None:
                    try:
                        if (model_start.runtime or selection_requested or selection_pending
                                or model_catching_up):
                            raise ValueError('model selection/start already running')
                        model_start.queue(session, time.monotonic_ns())
                        recovery.cancel()
                        tracker.lose('model start selecting a fresh target', force=True)
                        session.target = ''
                        selection_requested = True
                        user_notice = 'selecting model target; follow will start after Robot confirmation'
                    except ValueError as error:
                        user_notice = str(error)
                    # Model b is handled asynchronously; never start on the old point.
                    key = -1
                if key == ord('d') and selector is not None:
                    model_start.cancel()
                    if session.following:
                        user_notice = 'pause before selecting another target'
                    elif selection_requested or selection_pending:
                        user_notice = 'model selection already running'
                    else:
                        recovery.cancel()
                        selection_requested = True
                        user_notice = 'model selection queued for next fresh frame'
                if clicked:
                    pixel = clicked[-1]; clicked.clear()
                    try:
                        if frame is None: raise ValueError('no valid fresh RGB-D frame')
                        session.select()
                        user_notice = ""
                        raw,z,ts = frame
                        result = tracker.initialize(raw,z,pixel,k,d,ts)
                        diagnostics.emit(result,tracker,kind='tracking_initialization')
                        record(dict(type='click',pixel=pixel,target_id=session.target))
                    except ValueError as error:
                        user_notice = str(error)
                        record(dict(type='selection_rejected',reason=str(error)))
                if key in (ord('b'),ord('r'),ord('p'),ord('q')):
                    action = {ord('b'):'begin',ord('r'):'resume',ord('p'):'pause',ord('q'):'end'}[key]
                    model_start.cancel()
                    try:
                        # Publish initialized observation before begin; never command first.
                        if result.get('valid') and observation_is_fresh(result, time.monotonic_ns()):
                            publish_observation(result)
                        packet=session.request(action,observation_is_fresh(result, time.monotonic_ns()))
                        if selector is not None and action in ('begin', 'resume'):
                            recovery.arm(session)
                        publish(COMMAND,packet)
                        user_notice = ''
                        if action in ('pause','end'):
                            recovery.cancel()
                            selection_requested = False
                            selection_pending = False
                            model_catching_up = False
                            if fusion is not None: fusion.reset()
                            tracker.lose('operator paused; select again',force=True);session.target=''
                    except ValueError as error:
                        user_notice = str(error)
                        record(dict(type='command_rejected',reason=str(error)));print(error,flush=True)
                    if key == ord('q'): break
        finally:
            if session.runtime and session.following:
                publish(COMMAND,session.request('end',False))
            if started: pipeline.stop()
            if subscriber is not None: subscriber.close()
            if client is not None: client.close()
            if selector is not None: selector.close()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: pass
    except StartupFramesUnavailable as error:
        print('BLOCKED:', error, flush=True)
        raise SystemExit(7)
    except Exception as error: print('BLOCKED:',error); raise SystemExit(2)
