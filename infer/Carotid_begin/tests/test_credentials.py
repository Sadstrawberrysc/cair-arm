"""Local API Key storage never uses the repository or prints the secret."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from infer.Carotid_begin.credentials import get_api_key, save_api_key


class CredentialsTests(unittest.TestCase):
    def test_private_file_and_environment_override(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config" / "roboflow_api_key"
            save_api_key("  stored-key  ", path)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(path.parent.stat().st_mode & 0o777, 0o700)
            with patch.dict(os.environ, {"ROBOFLOW_API_KEY": ""}):
                self.assertEqual(get_api_key(path), "stored-key")
            with patch.dict(os.environ, {"ROBOFLOW_API_KEY": "session-key"}):
                self.assertEqual(get_api_key(path), "session-key")

    def test_missing_insecure_and_invalid_key(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "key"
            with patch.dict(os.environ, {"ROBOFLOW_API_KEY": ""}):
                self.assertIsNone(get_api_key(path))
                with self.assertRaisesRegex(ValueError, "single line"):
                    save_api_key("line1\nline2", path)
                save_api_key("key", path)
                path.chmod(0o644)
                with self.assertRaisesRegex(ValueError, "mode 600"):
                    get_api_key(path)


if __name__ == "__main__":
    unittest.main()
