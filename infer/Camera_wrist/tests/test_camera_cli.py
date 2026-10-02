"""Camera startup options must not bypass the Robot state connection."""
import contextlib
import io
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from click_follow import parse_args


class CameraCliTests(unittest.TestCase):
    def test_default_and_model_start_use_standard_confidence(self):
        for options in ([], ['--model-id', 'test/model', '--detector-python', sys.executable]):
            with self.subTest(options=options):
                args = parse_args(options)
                self.assertEqual(args.redis_port, 7777)
                self.assertEqual(args.model_confidence, .4)
                self.assertFalse(hasattr(args, 'no_redis'))

    def test_offline_flags_are_rejected_before_hardware_start(self):
        for flag in ('--no-redis', '--no--redis', '--standard-vision'):
            with self.subTest(flag=flag), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    parse_args([flag])
                self.assertEqual(raised.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
