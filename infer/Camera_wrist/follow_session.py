"""Follow request ownership, model startup, and acknowledged recovery.

These state machines do not access camera hardware, Redis, or the robot SDK.
The camera loop supplies observations and Robot state, then sends their requests.
"""
import time
import uuid

from model_selection import observation_is_fresh
from wrist_projection import FRAME, boot_id


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
    """One queued start requires a fresh selection and Robot acknowledgement."""
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


class ModelStartup:
    """Retry initial model selection until acknowledged start or cancellation."""
    def __init__(self, enabled):
        self.pending = enabled
        self.runtime = ''
        self.next_attempt_ns = 0

    def cancel(self):
        self.pending = False

    def failed(self, now):
        if self.pending:
            self.next_attempt_ns = now + 100_000_000

    def queue(self, session, start, now, *, selecting):
        if not self.pending:
            return False
        state = session.state or {}
        if (session.following
                or (self.runtime and session.runtime != self.runtime)
                or (self.runtime and str(state.get('control_state', '')).lower() == 'fault')):
            self.cancel()
            return False
        if selecting or start.runtime or now < self.next_attempt_ns:
            return False
        try:
            start.queue(session, now)
        except ValueError:
            return False
        self.runtime = session.runtime
        self.next_attempt_ns = now + 100_000_000
        return True


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
