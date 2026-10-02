import sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from click_follow import ClickSession, ModelRecovery
from wrist_projection import boot_id,FRAME

class ClickSessionTests(unittest.TestCase):
    def state(self):
        return dict(version=1,scope='click_follow',boot_id=boot_id(),calibration_sha256='hash',
            frame=FRAME,length_unit='m',valid=True,timestamp_monotonic_ns=time.monotonic_ns(),
            runtime_session_id='run',sequence=1,follow_state='hold')
    def test_click_does_not_start_and_start_requires_robot(self):
        s=ClickSession('hash');s.select()
        self.assertFalse(s.following)
        with self.assertRaises(ValueError):s.request('begin',True)
        state=self.state();s.accept_state(state,time.monotonic_ns())
        s.request('begin',True)
        self.assertTrue(s.following)
        with self.assertRaises(ValueError):s.select()
    def test_pause_reselect_resume_and_robot_restart(self):
        s=ClickSession('hash');s.accept_state(self.state(),time.monotonic_ns());s.select()
        s.request('begin',True);s.request('pause',False);s.select()
        with self.assertRaises(ValueError):s.request('begin',True)
        s.request('resume',True)
        state=self.state();state['runtime_session_id']='new_run'
        self.assertTrue(s.accept_state(state,time.monotonic_ns()))
        self.assertFalse(s.following);self.assertEqual(s.target,'')
    def test_invalid_observation_never_starts(self):
        s=ClickSession('hash');s.accept_state(self.state(),time.monotonic_ns());s.select()
        with self.assertRaises(ValueError):s.request('begin',False)
        state=self.state();state['calibration_sha256']='other'
        with self.assertRaises(ValueError):s.accept_state(state,time.monotonic_ns())
    def test_robot_hold_latches_ui_and_restart_requires_resume(self):
        s=ClickSession('hash');s.accept_state(self.state(),time.monotonic_ns());s.select()
        packet=s.request('begin',True)
        state=self.state();state.update(sequence=2,request_producer_id=s.producer,
            request_sequence=packet['sequence'],resume_required=True,follow_state='hold')
        self.assertTrue(s.accept_state(state,time.monotonic_ns()))
        self.assertFalse(s.following);self.assertEqual(s.target,'')
        restarted=ClickSession('hash');restarted.accept_state(state,time.monotonic_ns());restarted.select()
        with self.assertRaises(ValueError):restarted.request('begin',True)
        restarted.request('resume',True)

    def test_stale_state(self):
        s=ClickSession('hash');state=self.state();state['timestamp_monotonic_ns']-=300_000_000
        with self.assertRaises(ValueError):s.accept_state(state,time.monotonic_ns())


    def recovering(self):
        session = ClickSession('hash')
        session.accept_state(self.state(), time.monotonic_ns())
        session.select()
        session.request('begin', True)
        recovery = ModelRecovery()
        recovery.arm(session)
        packet = recovery.pause_for_loss(session)
        return session, recovery, packet

    def ack_pause(self, session, packet, **changes):
        state = self.state()
        state.update(sequence=2, request_producer_id=session.producer,
                     request_sequence=packet['sequence'], resume_required=True,
                     follow_reason='operator_paused', target_id=session.target)
        state.update(changes)
        session.accept_state(state, time.monotonic_ns())

    def test_recovery_waits_for_pause_and_new_observation_ack(self):
        session, recovery, pause = self.recovering()
        self.assertFalse(recovery.ready(session, time.monotonic_ns()))
        self.ack_pause(session, pause)
        self.assertTrue(recovery.ready(session, time.monotonic_ns()))
        result = dict(valid=True, capture_monotonic_ns=time.monotonic_ns())
        self.assertIsNone(recovery.resume(session, result, time.monotonic_ns()))
        session.select()
        self.assertNotEqual(session.target, pause['target_id'])
        self.assertIsNone(recovery.resume(session, result, time.monotonic_ns()))
        self.ack_pause(session, pause, sequence=3)
        result['capture_monotonic_ns'] -= 300_000_000
        self.assertIsNone(recovery.resume(session, result, time.monotonic_ns()))
        result['capture_monotonic_ns'] = time.monotonic_ns()
        request = recovery.resume(session, result, time.monotonic_ns())
        self.assertEqual(request['action'], 'resume')
        self.assertTrue(session.following)
        self.assertFalse(recovery.waiting)
        self.assertIsNone(recovery.resume(session, result, time.monotonic_ns()))

    def test_recovery_accepts_only_visual_hold_after_own_pause(self):
        for reason in ('planner_rejected', 'operator_ended', 'robot_state_invalid_or_stale'):
            session, recovery, pause = self.recovering()
            self.ack_pause(session, pause, follow_reason=reason)
            self.assertFalse(recovery.ready(session, time.monotonic_ns()))
            self.assertFalse(recovery.enabled)
        session, recovery, pause = self.recovering()
        self.ack_pause(session, pause, follow_reason='invalid wrist observation')
        self.assertTrue(recovery.ready(session, time.monotonic_ns()))

    def test_recovery_cancel_on_pause_disconnect_restart_or_foreign_request(self):
        for cause in ('manual', 'stale', 'restart', 'foreign'):
            session, recovery, pause = self.recovering()
            self.ack_pause(session, pause)
            if cause == 'manual':
                recovery.cancel()
            elif cause == 'stale':
                session.state['timestamp_monotonic_ns'] -= 300_000_000
            elif cause == 'restart':
                self.ack_pause(session, pause, runtime_session_id='other', sequence=3)
            else:
                self.ack_pause(session, pause, request_producer_id='another-user', sequence=3)
            self.assertFalse(recovery.ready(session, time.monotonic_ns()), cause)
            self.assertFalse(recovery.enabled, cause)

    def test_unexpected_robot_hold_does_not_enable_recovery(self):
        session = ClickSession('hash')
        session.accept_state(self.state(), time.monotonic_ns())
        session.select()
        begin = session.request('begin', True)
        recovery = ModelRecovery()
        recovery.arm(session)
        self.ack_pause(session, begin, follow_reason='planner_rejected')
        recovery.check_state(session, time.monotonic_ns())
        self.assertFalse(recovery.enabled)

    def active_model_session(self):
        session = ClickSession('hash')
        session.accept_state(self.state(), time.monotonic_ns())
        session.select()
        request = session.request('begin', True)
        recovery = ModelRecovery()
        recovery.arm(session)
        return session, recovery, request

    def timing_hold(self, session, request, **changes):
        state = self.state()
        state.update(sequence=session.last_state_sequence+1,
                     follow_reason='wrist_pose_time_unmatched',
                     target_id=session.target, request_producer_id=session.producer,
                     request_sequence=request['sequence'], resume_required=True)
        state.update(changes)
        return state

    def test_robot_timing_hold_recovers_repeatedly_after_own_pause_and_new_target(self):
        session, recovery, request = self.active_model_session()
        for _ in range(2):
            old_target = session.target
            hold = self.timing_hold(session, request)
            changed, pause = recovery.accept_state(session, hold, time.monotonic_ns())
            self.assertTrue(changed)
            self.assertEqual(pause['action'], 'pause')
            self.assertEqual(pause['target_id'], old_target)
            self.assertFalse(session.following)
            self.assertEqual(session.target, '')
            self.assertTrue(recovery.enabled and recovery.waiting)
            self.assertFalse(recovery.ready(session, time.monotonic_ns()))
            # Repeated state must not issue another pause or reset the sequence.
            self.assertIsNone(recovery.accept_state(session, hold, time.monotonic_ns())[1])
            self.ack_pause(session, pause, sequence=session.last_state_sequence+1,
                           target_id=old_target)
            self.assertTrue(recovery.ready(session, time.monotonic_ns()))
            session.select()
            result = dict(valid=True, capture_monotonic_ns=time.monotonic_ns())
            self.assertIsNone(recovery.resume(session, result, time.monotonic_ns()))
            self.ack_pause(session, pause, sequence=session.last_state_sequence+1)
            request = recovery.resume(session, result, time.monotonic_ns())
            self.assertEqual(request['action'], 'resume')
            self.assertNotEqual(request['target_id'], old_target)
            self.assertTrue(session.following)

    def test_robot_timing_recovery_requires_owned_current_nonfault_hold(self):
        for changes in [dict(follow_reason='planner_rejected'),
                        dict(follow_reason='operator_paused'),
                        dict(control_state='fault'),
                        dict(runtime_session_id='new-run'),
                        dict(target_id='other-target'),
                        dict(request_producer_id='other-producer'),
                        dict(request_sequence=0), dict(sequence=1)]:
            with self.subTest(changes=changes):
                session, recovery, request = self.active_model_session()
                hold = self.timing_hold(session, request, **changes)
                self.assertIsNone(recovery.accept_state(session, hold, time.monotonic_ns())[1])
                self.assertFalse(recovery.waiting)

    def test_robot_timing_hold_does_not_rearm_cancelled_or_manual_session(self):
        for manual in (False, True):
            session, recovery, request = self.active_model_session()
            hold = self.timing_hold(session, request)
            recovery.cancel()
            if manual:
                session.request('pause', False)
            self.assertIsNone(recovery.accept_state(session, hold, time.monotonic_ns())[1])
            self.assertFalse(recovery.enabled)
            self.assertFalse(session.following)

    def test_robot_timing_hold_stale_state_is_rejected(self):
        session, recovery, request = self.active_model_session()
        hold = self.timing_hold(session, request,
                                timestamp_monotonic_ns=time.monotonic_ns()-300_000_000)
        with self.assertRaisesRegex(ValueError, 'age'):
            recovery.accept_state(session, hold, time.monotonic_ns())
        self.assertFalse(recovery.waiting)

if __name__=='__main__':unittest.main()
