"""Offline continuous RS gripper tracking, closing preload, and wire-torque tests."""

from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from rebot_gripper import GripperController, GripperSettings, load_settings
from rebot_mit import LIMITS, decode_mit, encode_mit


class GripperTests(unittest.TestCase):
    def setUp(self):
        self.cfg = GripperSettings()
        self.ctrl = GripperController(.2, 4.9, self.cfg)

    def step(self, target, position, velocity):
        return self.ctrl.update(target, position, velocity)

    def test_small_position_errors_and_velocity_use_one_total_pd_limit(self):
        self.ctrl = GripperController(.2, 4.9, replace(self.cfg, close_preload_deg=0.))
        self.assertAlmostEqual(self.step(1.21, 1.2, .1), .07)
        self.assertEqual(self.step(4.9, 1.2, 0.), 1.)
        self.assertEqual(self.step(.2, 1.2, 0.), -1.)
        self.assertEqual(self.step(1.2, 1.2, 10.), -1.)
        # Opposing terms must cancel before saturation, rather than being
        # clipped separately or adding uncapped motor-side damping afterward.
        self.assertAlmostEqual(self.step(2.2, 1.2, 24.), 0.)

    def test_slow_or_stalled_closing_never_reanchors_the_requested_position(self):
        for _ in range(400):
            self.assertEqual(self.step(.2, 1.2, 0.), -1.)
            self.assertEqual(self.step(.2, 1.2, -.1), -1.)

    def test_opening_reverses_torque_on_the_first_sample_after_a_long_stall(self):
        for _ in range(400):
            self.step(.2, 1.2, 0.)
        self.assertEqual(self.step(2., 1.2, 0.), 1.)
        self.assertEqual(self.step(.2, 1.2, 0.), -1.)

    def test_torque_is_continuous_near_the_target_and_has_no_contact_threshold(self):
        self.ctrl = GripperController(.2, 4.9, replace(self.cfg, close_preload_deg=0.))
        errors = np.linspace(-.02, .02, 81)
        torques = [self.step(1.2 + error, 1.2, 0.) for error in errors]
        np.testing.assert_allclose(torques, self.cfg.kp * errors, atol=1e-14)

    def test_targets_are_clipped_to_travel_before_computing_torque(self):
        self.assertAlmostEqual(self.step(-10., .2, 0.), -self.cfg.kp * math.radians(1.))
        self.assertEqual(self.step(10., 4.9, 0.), 0.)
        self.assertLess(self.step(.2, .21, 0.), 0.)
        self.assertGreater(self.step(4.9, 4.89, 0.), 0.)

    def simulate_friction(self, close_preload_deg):
        ctrl = GripperController(.2, 4.9, replace(self.cfg, close_preload_deg=close_preload_deg))
        position, velocity, dt = 1.2, 0., .001
        history = []
        for i in range(12000):
            torque = ctrl.update(.2, position, velocity)
            self.assertLessEqual(abs(torque), self.cfg.torque_limit_nm)
            # The first 0.3 seconds model a temporary obstruction. Static
            # friction balances effort at rest until the breakaway threshold.
            if i < 300 or (abs(velocity) < .001 and abs(torque) <= .15):
                velocity = 0.
            else:
                friction = .12 * math.copysign(1., velocity if abs(velocity) > .001 else torque)
                acceleration = (torque - friction - .1 * velocity) / .005
                velocity += acceleration * dt
                position = max(.2, position + velocity * dt)
                if position == .2:
                    velocity = 0.
            history.append(position)
        return history

    def test_preload_closes_against_modelled_friction_after_obstruction(self):
        pd_history = self.simulate_friction(close_preload_deg=0.)
        compensated = self.simulate_friction(close_preload_deg=self.cfg.close_preload_deg)
        self.assertGreater(pd_history[-1] - .2, .005)
        self.assertEqual(compensated[-1], .2)
        self.assertLess(compensated[400], 1.15)
        self.assertLess(max(compensated[-500:]) - min(compensated[-500:]), .001)

    def test_closed_target_keeps_bounded_preload_at_the_actual_closed_pose(self):
        self.assertAlmostEqual(self.step(.2, .2, 0.), -math.radians(1.) * self.cfg.kp)
        self.assertAlmostEqual(self.step(.2, .2 + math.radians(.5), 0.),
                               -math.radians(1.5) * self.cfg.kp)

    def test_constant_error_does_not_accumulate_effort_and_opening_removes_preload(self):
        expected = -(.04 + math.radians(1.)) * self.cfg.kp
        for _ in range(2000):
            self.assertAlmostEqual(self.step(.2, .24, 0.), expected)
        self.assertEqual(self.step(1.2, 1.2, 0.), 0.)
        self.assertAlmostEqual(self.step(1.21, 1.2, 0.), .01 * self.cfg.kp)

    def test_free_travel_has_no_directional_friction_torque_or_arrival_switch(self):
        for error in np.linspace(-.0009, .0009, 19):
            for velocity in (0., .1, -.1):
                with self.subTest(error=error, velocity=velocity):
                    self.assertAlmostEqual(self.step(1.2 + error, 1.2, velocity),
                                           self.cfg.kp * error - self.cfg.kd * velocity)
        self.assertAlmostEqual(self.step(1.202, 1.2, 0.), .024)
        self.assertAlmostEqual(self.step(1.198, 1.2, 0.), -.024)

    def test_closing_window_smoothly_releases_preload_without_losing_pd_feedback(self):
        for gap, preload in ((0., 1.), (2.5, .5), (5., 0.), (6., 0.)):
            target = .2 + math.radians(gap)
            with self.subTest(gap=gap):
                self.assertAlmostEqual(self.step(target, target, 0.),
                                       -self.cfg.kp * math.radians(preload))
                self.assertAlmostEqual(self.step(target, target, .1),
                                       -self.cfg.kp * math.radians(preload) - .05)
        torques = [self.step(.2 + math.radians(gap), .2, 0.)
                   for gap in np.linspace(0., 6., 601)]
        self.assertTrue(np.all(np.diff(torques) >= 0.))
        self.assertLess(max(np.diff(torques)), .003)

    def test_shortened_travel_preserves_full_preload_and_full_open_endpoint(self):
        ctrl = GripperController(.2, .2 + math.radians(2.), self.cfg)
        self.assertAlmostEqual(ctrl.update(.2, .2, 0.), -self.cfg.kp * math.radians(1.))
        self.assertEqual(ctrl.update(ctrl.open, ctrl.open, 0.), 0.)

    def test_total_limit_includes_preload_and_damping(self):
        self.assertEqual(self.step(.2, .24, 2.), -1.)
        self.assertEqual(self.step(.28, .24, -2.), 1.)
        # Sum opposing terms before applying one common limit.
        self.assertAlmostEqual(self.step(2.2, 1.2, 24.), 0.)
        cfg = replace(self.cfg, torque_limit_nm=.1)
        cfg.validate_motor(LIMITS["rs-00"])
        ctrl = GripperController(.2, 4.9, cfg)
        self.assertEqual(ctrl.update(.2, 1.2, 0.), -.1)
        self.assertEqual(ctrl.update(2., 1.2, 0.), .1)
        self.assertEqual(ctrl.update(.2, .2, 0.), -.1)

    def test_wire_torque_stays_inside_limit_after_mit_quantization(self):
        for limit in (.0005, .1, .3, 1., 3.5, 14.):
            for torque in (-14., -limit, 0., limit, 14.):
                with self.subTest(limit=limit, torque=torque):
                    frame = encode_mit(7, "rs-00", .2, torque, allow_gripper=True,
                                       torque_limit=limit)
                    decoded = decode_mit(frame, "rs-00")
                    self.assertLessEqual(abs(decoded["gravity"]), limit + 1e-14)
                    self.assertEqual(decoded["kp"], 0.)
                    self.assertEqual(decoded["kd"], 0.)

    def test_total_limit_rejects_native_pd_that_would_bypass_it(self):
        for gains in ({"kp": 1.}, {"kd": .5}):
            with self.subTest(gains=gains), self.assertRaisesRegex(ValueError, "zero native gains"):
                encode_mit(7, "rs-00", .2, 0., allow_gripper=True, torque_limit=1., **gains)
        for limit in (0., -1., .00001, 15., math.nan, math.inf):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                encode_mit(7, "rs-00", .2, 0., allow_gripper=True, torque_limit=limit)

    def test_invalid_feedback_and_travel_are_rejected(self):
        valid = dict(target=.2, position=1.2, velocity=0.)
        for key in valid:
            for invalid in (math.nan, math.inf, -math.inf):
                with self.subTest(key=key, invalid=invalid), self.assertRaises(ValueError):
                    self.ctrl.update(**{**valid, key: invalid})
        for low, high in ((1., 1.), (2., 1.), (math.nan, 1.), (0., math.inf)):
            with self.subTest(low=low, high=high), self.assertRaises(ValueError):
                GripperController(low, high, self.cfg)

    def test_settings_reject_invalid_values_and_motor_ranges(self):
        for key in vars(self.cfg):
            for invalid in (0, -1, math.nan, math.inf, True):
                if key in ("leader_close_tolerance_deg", "close_preload_deg") and invalid == 0:
                    self.assertEqual(getattr(replace(self.cfg, **{key: 0.}), key), 0.)
                    continue
                with self.subTest(key=key, invalid=invalid), self.assertRaises(ValueError):
                    replace(self.cfg, **{key: invalid})
        for key, invalid in (("torque_limit_nm", 15.), ("torque_limit_nm", .00001),
                             ("kp", 501.), ("kd", 6.)):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "MIT range"):
                replace(self.cfg, **{key: invalid}).validate_motor(LIMITS["rs-00"])
        self.cfg.validate_motor(LIMITS["rs-00"])
        for tolerance in (45., 46.):
            with self.assertRaisesRegex(ValueError, "45-degree"):
                replace(self.cfg, leader_close_tolerance_deg=tolerance)

    def test_yaml_loading_and_torque_limit_override(self):
        self.assertEqual(load_settings(), self.cfg)
        self.assertEqual(load_settings(torque_limit_nm=.3).torque_limit_nm, .3)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gripper.yaml"
            values = vars(self.cfg).copy()
            values.pop("leader_close_tolerance_deg")
            values.pop("close_preload_deg")
            values.pop("close_preload_window_deg")
            path.write_text(yaml.safe_dump(values))
            self.assertEqual(load_settings(path), self.cfg)
            for removed in ({"ki": 0.}, {"ki": 6.}, {"contact_torque_ratio": .5},
                            {"friction_torque_nm": .3}, {"position_deadband_rad": .001}):
                path.write_text(yaml.safe_dump({**vars(self.cfg), **removed}))
                with self.subTest(removed=removed), self.assertRaisesRegex(ValueError, "exactly"):
                    load_settings(path)


if __name__ == "__main__":
    unittest.main()
