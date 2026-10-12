"""Default model startup and explicit manual mode keep the Robot connection."""
import contextlib
import io
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import click_follow
from infer.Camera_wrist.detector.config import DEFAULT_MODEL_ID


class CameraCliTests(unittest.TestCase):
    def setUp(self):
        # Tests must not depend on the operator's model or detector environment.
        self.environment = patch.dict(os.environ, {'CAROTID_MODEL_ID': ''})
        self.interpreter = patch.object(click_follow, 'DEFAULT_DETECTOR_PYTHON',
                                        Path(sys.executable))
        self.environment.start()
        self.interpreter.start()
        self.addCleanup(self.environment.stop)
        self.addCleanup(self.interpreter.stop)

    def assert_cli_error(self, options):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                click_follow.parse_args(options)
        self.assertEqual(raised.exception.code, 2)

    def test_no_arguments_start_default_model(self):
        args = click_follow.parse_args([])
        self.assertEqual(args.model_id, DEFAULT_MODEL_ID)
        self.assertEqual(args.detector_python, Path(sys.executable))

    def test_environment_overrides_default_model(self):
        with patch.dict(os.environ, {'CAROTID_MODEL_ID': 'test/environment-model'}):
            args = click_follow.parse_args([])
        self.assertEqual(args.model_id, 'test/environment-model')

    def test_empty_environment_falls_back_to_default_model(self):
        for value in ('', '   '):
            with self.subTest(value=value), patch.dict(os.environ, {'CAROTID_MODEL_ID': value}):
                self.assertEqual(click_follow.parse_args([]).model_id, DEFAULT_MODEL_ID)

    def test_explicit_model_overrides_environment(self):
        with patch.dict(os.environ, {'CAROTID_MODEL_ID': 'test/environment-model'}):
            args = click_follow.parse_args(['--model-id', 'test/explicit-model'])
        self.assertEqual(args.model_id, 'test/explicit-model')

    def test_manual_mode_needs_no_model_or_detector_interpreter(self):
        with tempfile.TemporaryDirectory() as directory:
            missing_python = Path(directory) / 'missing-python'
            with patch.object(click_follow, 'DEFAULT_DETECTOR_PYTHON', missing_python), \
                    patch.dict(os.environ, {'CAROTID_MODEL_ID': 'test/environment-model'}), \
                    patch.object(click_follow, 'ModelSelector') as selector:
                args = click_follow.parse_args(['--manual'])
                self.assertIsNone(args.model_id)
                selector.assert_not_called()
            args = click_follow.parse_args(['--manual', '--detector-python', str(missing_python)])
            self.assertIsNone(args.model_id)
            self.assertEqual(args.redis_port, 7777)

    def test_manual_and_model_id_are_mutually_exclusive(self):
        self.assert_cli_error(['--manual', '--model-id', 'test/model'])
        self.assert_cli_error(['--model-id', 'test/model', '--manual'])

    def test_explicit_empty_model_id_is_rejected(self):
        for value in ('', '   '):
            with self.subTest(value=value):
                self.assert_cli_error(['--model-id', value])

    def test_model_mode_rejects_missing_interpreter_before_hardware_start(self):
        with tempfile.TemporaryDirectory() as directory:
            missing_python = Path(directory) / 'missing-python'
            with patch.object(click_follow, 'DEFAULT_DETECTOR_PYTHON', missing_python):
                self.assert_cli_error([])
            self.assert_cli_error(['--model-id', 'test/model',
                                   '--detector-python', str(missing_python)])

    def test_all_modes_keep_robot_connection_and_standard_confidence(self):
        for options in ([], ['--model-id', 'test/model'], ['--manual']):
            with self.subTest(options=options):
                args = click_follow.parse_args(options)
                self.assertEqual(args.redis_port, 7777)
                self.assertEqual(args.model_confidence, .4)
                self.assertFalse(hasattr(args, 'no_redis'))

    def test_offline_flags_are_rejected_before_hardware_start(self):
        for flag in ('--no-redis', '--no--redis', '--standard-vision'):
            with self.subTest(flag=flag):
                self.assert_cli_error([flag])


class CameraDeviceTests(unittest.TestCase):
    def device_list(self, entries):
        devices = Mock()
        devices.get_count.return_value = len(entries)
        devices.get_device_name_by_index.side_effect = lambda i: entries[i][0]
        devices.get_device_serial_number_by_index.side_effect = lambda i: entries[i][1]
        context = Mock()
        context.query_devices.return_value = devices
        return context, devices

    def test_no_devices_reports_connection_failure(self):
        context, devices = self.device_list([])
        with self.assertRaisesRegex(ValueError, '枚举到 0 台设备') as raised:
            click_follow.select_wrist_device(context, 'CV2L360000HZ')
        self.assertIn('CV2L360000HZ', str(raised.exception))
        devices.get_device_by_serial_number.assert_not_called()

    def test_wrong_serial_reports_available_identity(self):
        context, devices = self.device_list([('Gemini 305', 'OTHER_SERIAL')])
        with self.assertRaisesRegex(ValueError, 'OTHER_SERIAL') as raised:
            click_follow.select_wrist_device(context, 'CV2L360000HZ')
        self.assertIn('Gemini 305', str(raised.exception))
        devices.get_device_by_serial_number.assert_not_called()

    def test_exact_serial_selected_among_multiple_devices(self):
        context, devices = self.device_list([('Other camera', 'OTHER_SERIAL'),
                                            ('Gemini 305', 'CV2L360000HZ')])
        selected = click_follow.select_wrist_device(context, 'CV2L360000HZ')
        self.assertIs(selected, devices.get_device_by_serial_number.return_value)
        devices.get_device_by_serial_number.assert_called_once_with('CV2L360000HZ')


if __name__ == '__main__':
    unittest.main()
