"""Reject invalid diagnostics before any robot connection."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class ControllerCheckArgsTest(unittest.TestCase):
    def run_checker(self, *args):
        root = Path(__file__).resolve().parents[2]
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = str(Path.home() / ".local/xcore-vision/lib") + ":" + env.get("LD_LIBRARY_PATH", "")
        result = subprocess.run([str(root / "build/bin/check_reach_controller"), *args],
                                env=env, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertNotIn("[READ-ONLY]", result.stdout)
        return result.stderr

    def test_execute_not_supported(self):
        self.assertIn("Usage:", self.run_checker("--execute"))
        self.assertIn("Usage:", self.run_checker("missing.json", "--execute"))

    def test_missing_arguments(self):
        self.assertIn("Usage:", self.run_checker())

    def test_invalid_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps({"schema_version": 0}), encoding="ascii")
            self.assertIn("unsupported reach manifest", self.run_checker(str(path)))


if __name__ == "__main__":
    unittest.main()
