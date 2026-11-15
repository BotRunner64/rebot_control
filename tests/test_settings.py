"""Shared control settings are validated before any hardware is opened."""

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from rebot.paths import CONTROL_SETTINGS
from rebot.settings import ControlSettings, load_settings


class SettingsTests(unittest.TestCase):
    def test_invalid_settings_are_rejected_before_can_is_opened(self):
        for key, value in (("rate_hz", 1), ("gravity_scale", [-1] * 6),
                           ("hold_kp", [math.nan] * 6), ("max_sample_gap_s", 1),
                           ("unknown_key", 1)):
            settings = yaml.safe_load(CONTROL_SETTINGS.read_text())
            settings[key] = value
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "settings.yaml"
                path.write_text(yaml.safe_dump(settings))
                with self.assertRaises(ValueError):
                    load_settings(path)

    def test_shipped_settings_load_as_typed_attributes(self):
        settings = load_settings()
        self.assertIsInstance(settings, ControlSettings)
        raw = yaml.safe_load(CONTROL_SETTINGS.read_text())
        self.assertEqual(settings.rate_hz, raw["rate_hz"])
        for name in ("torque_limit_nm", "gravity_scale", "hold_kp", "damping"):
            self.assertEqual(getattr(settings, name).shape, (6,))
            np.testing.assert_array_equal(getattr(settings, name), raw[name])


if __name__ == "__main__":
    unittest.main()
