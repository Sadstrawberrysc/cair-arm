"""Single-key model start waits for a new target and Robot acknowledgement."""
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from follow_session import ClickSession, ModelStart, ModelStartup
from wrist_projection import FRAME, boot_id


class ModelStartTests(unittest.TestCase):
    def setup_session(self, started=False):
        s = ClickSession('hash')
        s.accept_state(dict(version=1, scope='click_follow', boot_id=boot_id(),
                            calibration_sha256='hash', frame=FRAME, length_unit='m',
                            valid=True, timestamp_monotonic_ns=time.monotonic_ns(),
                            runtime_session_id='run', sequence=1, follow_state='hold'),
                       time.monotonic_ns())
        s.started = started
        s.select()
        start = ModelStart()
        start.queue(s, time.monotonic_ns())
        return s, start

    def poll(self, start, s, selecting=False):
        return start.poll(s, dict(valid=True, capture_monotonic_ns=time.monotonic_ns()),
                          time.monotonic_ns(), selecting=selecting)

    def test_one_key_waits_for_selection_and_ack_then_starts_once(self):
        for started, action in [(False, 'begin'), (True, 'resume')]:
            s, start = self.setup_session(started)
            self.assertIsNone(self.poll(start, s, selecting=True))
            self.assertFalse(s.following)
            s.select()
            self.assertIsNone(self.poll(start, s))
            s.state['target_id'] = s.target
            packet = self.poll(start, s)
            self.assertEqual(packet['action'], action)
            self.assertTrue(s.following)
            self.assertIsNone(self.poll(start, s))

    def test_old_target_cannot_start(self):
        s, start = self.setup_session()
        s.state['target_id'] = s.target
        self.assertIsNone(self.poll(start, s))
        self.assertFalse(s.following)
        self.assertFalse(start.runtime)

    def test_invalid_selection_cancels_instead_of_starting_later(self):
        s, start = self.setup_session()
        s.select()
        s.state['target_id'] = s.target
        self.assertIsNone(start.poll(s, dict(valid=False), time.monotonic_ns(), selecting=False))
        self.assertIsNone(self.poll(start, s))
        self.assertFalse(s.following)

    def test_pause_restart_fault_and_stale_state_cancel_pending_start(self):
        for cause in ('pause', 'restart', 'fault', 'stale', 'following'):
            with self.subTest(cause=cause):
                s, start = self.setup_session()
                s.select()
                s.state['target_id'] = s.target
                if cause == 'pause': start.cancel()
                if cause == 'restart': s.runtime = 'new'
                if cause == 'fault': s.state['control_state'] = 'fault'
                if cause == 'stale': s.state['timestamp_monotonic_ns'] -= 300_000_000
                if cause == 'following': s.state['follow_state'] = 'following'
                self.assertIsNone(self.poll(start, s))
                self.assertFalse(start.runtime)
                self.assertFalse(s.following)

    def test_queue_requires_robot_hold_and_no_active_follow(self):
        s = ClickSession('hash')
        with self.assertRaises(ValueError): ModelStart().queue(s, time.monotonic_ns())
        s, _ = self.setup_session()
        s.following = True
        with self.assertRaises(ValueError): ModelStart().queue(s, time.monotonic_ns())


class ModelStartupTests(unittest.TestCase):
    setup_session = ModelStartTests.setup_session
    poll = ModelStartTests.poll

    def test_initial_start_waits_for_robot_selection_and_target_ack(self):
        session = ClickSession('hash')
        start = ModelStart()
        startup = ModelStartup(True)
        self.assertFalse(startup.queue(session, start, time.monotonic_ns(), selecting=False))
        connected, _ = self.setup_session()
        session.accept_state(connected.state, time.monotonic_ns())
        self.assertTrue(startup.queue(session, start, time.monotonic_ns(), selecting=False))
        self.assertIsNone(self.poll(start, session, selecting=True))
        session.select()
        self.assertIsNone(self.poll(start, session))
        session.state['target_id'] = session.target
        self.assertEqual(self.poll(start, session)['action'], 'begin')
        self.assertFalse(startup.queue(session, start, time.monotonic_ns(), selecting=False))
        self.assertFalse(startup.pending)
        session.request('pause', False)
        self.assertFalse(startup.queue(session, start, time.monotonic_ns(), selecting=False))

    def test_rejected_selection_retries_with_new_target_after_delay(self):
        session, _ = self.setup_session()
        start, startup = ModelStart(), ModelStartup(True)
        now = time.monotonic_ns()
        self.assertTrue(startup.queue(session, start, now, selecting=False))
        self.assertIsNone(start.poll(session, dict(valid=False), now, selecting=False))
        self.assertFalse(session.following)
        # A slow failed inference must still wait 0.1 s after failure, rather
        # than treating the time spent inferring as part of the retry interval.
        failed_at = now + 300_000_000
        startup.failed(failed_at)
        self.assertFalse(startup.queue(session, start, failed_at + 50_000_000, selecting=False))
        now = failed_at + 100_000_000
        session.state['timestamp_monotonic_ns'] = now
        self.assertTrue(startup.queue(session, start, now, selecting=False))
        session.select()
        session.state['target_id'] = session.target
        # Keep the artificial retry clock out of ClickSession.request's real clock.
        session.state['timestamp_monotonic_ns'] = time.monotonic_ns()
        self.assertEqual(self.poll(start, session)['action'], 'begin')

    def test_manual_mode_and_cancel_never_queue_initial_start(self):
        for enabled in (False, True):
            session, _ = self.setup_session()
            start, startup = ModelStart(), ModelStartup(enabled)
            if enabled:
                startup.cancel()
            self.assertFalse(startup.queue(session, start, time.monotonic_ns(), selecting=False))
            self.assertFalse(session.following)

    def test_restart_or_fault_cancels_initial_intent(self):
        for cause in ('restart', 'fault'):
            with self.subTest(cause=cause):
                session, _ = self.setup_session()
                start, startup = ModelStart(), ModelStartup(True)
                self.assertTrue(startup.queue(session, start, time.monotonic_ns(), selecting=False))
                if cause == 'restart':
                    session.runtime = 'new'
                else:
                    session.state['control_state'] = 'fault'
                self.assertFalse(startup.queue(session, start, time.monotonic_ns(), selecting=True))
                self.assertFalse(startup.pending)
                self.assertIsNone(self.poll(start, session))
                self.assertFalse(session.following)

    def test_busy_stale_fault_and_non_hold_state_cannot_queue(self):
        for cause in ('busy', 'stale', 'fault', 'non_hold'):
            with self.subTest(cause=cause):
                session, _ = self.setup_session()
                start, startup = ModelStart(), ModelStartup(True)
                if cause == 'stale':
                    session.state['timestamp_monotonic_ns'] -= 300_000_000
                if cause == 'fault':
                    session.state['control_state'] = 'fault'
                if cause == 'non_hold':
                    session.state['follow_state'] = 'following'
                self.assertFalse(startup.queue(session, start, time.monotonic_ns(),
                                               selecting=cause == 'busy'))
                self.assertFalse(start.runtime)
                self.assertFalse(session.following)


if __name__ == '__main__':
    unittest.main()
