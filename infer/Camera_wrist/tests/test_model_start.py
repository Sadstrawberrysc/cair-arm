"""Single-key model start waits for a new target and Robot acknowledgement."""
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from click_follow import ClickSession, ModelStart
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


if __name__ == '__main__':
    unittest.main()
