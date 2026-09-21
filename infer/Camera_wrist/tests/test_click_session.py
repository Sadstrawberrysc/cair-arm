import sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from click_follow import ClickSession
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

if __name__=='__main__':unittest.main()
