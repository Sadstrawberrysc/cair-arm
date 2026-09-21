#!/usr/bin/env python3
"""Click-to-track commissioning UI. Robot motion requires an explicit Robot mode."""
import argparse
import json
from pathlib import Path
import time
import uuid
import cv2
import numpy as np
from gemini_config import WIDTH, HEIGHT, configure_close_range, stream_profiles
from local_tracker import LocalTracker
from projection_preview import CaptureClock, aligned_rgbd, color_bgr
from tracking_diagnostics import TrackingDiagnostics
from wrist_projection import boot_id, calibration, FRAME

STATE = 'robot:wrist:state:v1'
OBSERVATION = 'robot:wrist:observation:v1'
COMMAND = 'robot:wrist:command:v1'


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
        if (state.get('version') != 1 or state.get('scope') != 'click_follow'
                or state.get('boot_id') != boot_id() or state.get('calibration_sha256') != self.digest
                or state.get('frame') != FRAME or state.get('length_unit') != 'm'
                or not state.get('valid') or not 0 <= now-state.get('timestamp_monotonic_ns', 0) <= 200_000_000):
            raise ValueError('invalid/stale Robot state or calibration mismatch')
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='CV2L360000HZ')
    parser.add_argument('--calibration', type=Path, default=Path(__file__).with_name('gemini305_to_rm75_armtip.json'))
    parser.add_argument('--no-redis', action='store_true', help='independent click/normal test; cannot request Robot motion')
    parser.add_argument('--redis-port', type=int, default=7777)
    parser.add_argument('--log', type=Path, default=Path(__file__).resolve().parent / 'log' / ('wrist_click_%d.jsonl' % time.time_ns()))
    args = parser.parse_args()
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
    client = subscriber = None
    if not args.no_redis:
        import redis
        client = redis.Redis(host='127.0.0.1', port=args.redis_port, socket_timeout=.05, socket_connect_timeout=.1)
        subscriber = client.pubsub(ignore_subscribe_messages=True)
        subscriber.subscribe(STATE)
    clicked = []
    window = 'Wrist click follow'
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, lambda event, x, y, flags, param:
                        clicked.append((x, y)) if event == cv2.EVENT_LBUTTONDOWN else None)
    started = False
    last_heartbeat = 0.
    user_notice = ""
    last_published_capture = {}
    last_complete = time.monotonic()
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
                    intrinsic=k.tolist(), distortion=d.tolist(), sdk=str(ob.__version__), independent_validation=False))
        try:
            pipeline.start(config); started = True; pipeline.enable_frame_sync()
            while True:
                if subscriber is not None:
                    try:
                        for _ in range(100):
                            msg = subscriber.get_message(timeout=0)
                            if msg is None: break
                            if msg['type'] != 'message': continue
                            if session.accept_state(json.loads(msg['data']), time.monotonic_ns()):
                                tracker.lose('Robot held/restarted: '+str(session.state.get('follow_reason') or 'session changed')+'; select again', force=True)
                    except Exception as error:
                        if session.following:
                            tracker.lose('Robot state rejected/disconnected')
                            session.following = False
                            session.target = ''
                        record(dict(type='state_rejected', reason=str(error)))
                if session.following and (session.state is None or time.monotonic_ns()-session.state['timestamp_monotonic_ns'] > 200_000_000):
                    tracker.lose('Robot state timeout'); session.following = False; session.target = ''
                color, depth, missing = aligned_rgbd(pipeline.wait_for_frames(100), align)
                frame = None
                result = dict(valid=False, reason=missing or tracker.reason)
                image = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
                if missing:
                    tracker.lose(missing)
                    if time.monotonic()-last_complete > 5: raise ValueError('no complete RGB-D for 5 seconds')
                else:
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
                        if time.monotonic_ns()-timestamp > 200_000_000:
                            raise ValueError('tracking computation exceeded observation age')
                    except ValueError as error:
                        tracker.lose(error); result = dict(valid=False, reason=str(error))
                diagnostics.emit(result, tracker)
                publish_observation(result)
                if not result.get('valid') and session.following:
                    publish(COMMAND, session.request('pause', False))
                if session.following and time.monotonic()-last_heartbeat >= .1:
                    heartbeat = session.envelope(); heartbeat['action'] = 'heartbeat'
                    publish(COMMAND, heartbeat); last_heartbeat = time.monotonic()
                if result.get('valid'):
                    cv2.circle(image, tuple(np.floor(result['pixel']).astype(int)), 6, (0,255,0), 2)
                    cv2.putText(image, 'p(m): '+str(np.round(result['point_camera_m'],3)), (10,75), 0,.5,(0,255,255),1)
                    cv2.putText(image, 'n: '+str(np.round(result['normal_out_camera'],3)), (10,100),0,.5,(0,255,255),1)
                    quality = result['quality']
                    cv2.putText(image, 'features: %d | plane RMS: %.2f mm | points: %d' % (
                        quality['feature_inliers'], quality['plane_rms_m']*1000, quality['surface_points']),
                        (10,125),0,.45,(0,255,255),1)
                robot_text = 'offline' if session.state is None else session.state.get('follow_state','waiting')+': '+session.state.get('follow_reason','')
                cv2.putText(image, 'Robot: '+robot_text[:70], (10,25),0,.45,(0,255,255),1)
                cv2.putText(image, result.get('reason','')[:80],(10,50),0,.45,(0,0,255),1)
                cv2.putText(image, user_notice[:85],(10,150),0,.45,(0,165,255),1)
                cv2.putText(image,'click: select | b: begin | p: pause | r: resume | q: exit',(10,465),0,.45,(255,255,255),1)
                cv2.imshow(window,image)
                key = cv2.waitKey(1)&255
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
                    try:
                        if args.no_redis and action in ('begin','resume'): raise ValueError('no-redis is observation only')
                        # Publish initialized observation before begin; never command first.
                        if result.get('valid'):
                            publish_observation(result)
                        packet=session.request(action,bool(result.get('valid')));publish(COMMAND,packet)
                        user_notice = ''
                        if action in ('pause','end'):
                            tracker.lose('operator paused; select again',force=True);session.target=''
                    except ValueError as error:
                        user_notice = str(error)
                        record(dict(type='command_rejected',reason=str(error)));print(error,flush=True)
                    if key == ord('q'): break
        finally:
            if session.runtime:
                publish(COMMAND,session.request('end',False))
            if started: pipeline.stop()
            if subscriber is not None: subscriber.close()
            if client is not None: client.close()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: pass
    except Exception as error: print('BLOCKED:',error); raise SystemExit(2)
