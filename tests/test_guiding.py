"""Offline dynamics and hand-guiding controller transition checks."""

import copy
import unittest

import numpy as np
import pinocchio as pin

from rebot.model.gravity import GravityModel
from rebot.model.guiding import HandGuidingController
from rebot.settings import load_settings


def override(settings, **changes):
    """Copy frozen settings with values outside the loadable range (the old tests used a dict)."""
    result = copy.copy(settings)
    for name, value in changes.items():
        object.__setattr__(result, name, value)
    return result


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = GravityModel()

    def setUp(self):
        self.cfg = load_settings()
        self.q = np.array([0, .5, 1., 0, .1, 0])
        self.ctrl = HandGuidingController(self.model, self.cfg)

    def advance(self, q, start, end, step=.02):
        for now in np.arange(start, end + step / 2, step):
            result = self.ctrl.update(q, float(now))
        return result

    def test_gravity_matches_potential_energy_derivative(self):
        epsilon = 1e-6
        gradient = []
        for i in range(6):
            delta = np.zeros(6)
            delta[i] = epsilon
            upper = pin.computePotentialEnergy(self.model.model, self.model.data, self.q + delta)
            lower = pin.computePotentialEnergy(self.model.model, self.model.data, self.q - delta)
            gradient.append((upper - lower) / (2 * epsilon))
        np.testing.assert_allclose(self.model.gravity(self.q), gradient, atol=1e-6)

    def test_startup_preserves_gravity_support_from_the_first_command(self):
        result = self.ctrl.update(self.q, 0)
        gravity = self.model.gravity(self.q) * self.cfg.gravity_scale
        np.testing.assert_allclose(result.torque, gravity)
        np.testing.assert_allclose(self.ctrl.target, self.q)
        result = self.advance(self.q, .02, 1.0)
        np.testing.assert_allclose(result.gravity, gravity)

    def test_additional_gravity_scale_is_applied_from_the_first_command(self):
        # The per-run scale moved from update() to the constructor.
        ctrl = HandGuidingController(self.model, self.cfg, .5)
        result = ctrl.update(self.q, 0)
        gravity = self.model.gravity(self.q) * self.cfg.gravity_scale * .5
        np.testing.assert_allclose(result.gravity, gravity)
        np.testing.assert_allclose(result.torque, gravity)

    def test_startup_to_hold_keeps_the_existing_target_and_gain(self):
        self.ctrl.update(self.q, 0)
        moved = self.q.copy()
        moved[0] += .002
        self.advance(moved, .02, 1.98)
        before = self.ctrl.update(moved, 1.999)
        after = self.ctrl.update(moved, 2.001)
        self.assertEqual(after.state, "HOLD")
        np.testing.assert_allclose(self.ctrl.target, self.q)
        np.testing.assert_allclose(after.torque, before.torque, atol=1e-6)

    def test_fast_motion_does_not_abort_in_startup_or_drag(self):
        for start in (0., 2.4):
            for direction in (-1, 1):
                with self.subTest(start=start, direction=direction):
                    self.ctrl = HandGuidingController(self.model, self.cfg)
                    self.advance(self.q, 0, start)
                    moved = self.q.copy()
                    for i in range(1, 11):
                        moved[0] += direction * .08
                        result = self.ctrl.update(moved, start + i * .02)
                        self.assertTrue(np.all(np.isfinite(result.torque)))
                        self.assertTrue(np.all(np.abs(result.torque) <= self.cfg.torque_limit_nm))
                    self.assertEqual(result.state, "STARTUP" if start == 0 else "DRAG")

    def test_drag_release_stillness_hold_and_push_again(self):
        result = self.advance(self.q, 0, 2.4)
        self.assertEqual(result.state, "HOLD")
        moved = self.q.copy()
        for i in range(1, 9):
            moved[0] += .005
            result = self.ctrl.update(moved, 2.4 + i * .02)
        self.assertEqual(result.state, "DRAG")
        np.testing.assert_allclose(self.ctrl.target, moved)
        result = self.advance(moved, 2.58, 3.5)
        self.assertEqual(result.state, "HOLD")
        np.testing.assert_allclose(self.ctrl.target, moved)
        for now in (3.52, 3.54, 3.56):
            moved[1] += .01
            result = self.ctrl.update(moved, now)
        self.assertEqual(result.state, "DRAG")

    def test_error_threshold_releases_even_during_slow_push(self):
        self.advance(self.q, 0, 2.4)
        moved = self.q.copy()
        # Slow enough that velocity release does not trigger first.
        for i in range(1, 41):
            moved[0] += .001
            result = self.ctrl.update(moved, 2.4 + i * .02)
        self.assertEqual(result.state, "DRAG")

    def test_total_torque_is_clamped(self):
        cfg = override(self.cfg, damping=np.full(6, 1000.))
        ctrl = HandGuidingController(self.model, cfg)
        ctrl.update(self.q, 0)
        result = ctrl.update(self.q + .01, .02)
        self.assertTrue(np.all(np.abs(result.torque) <= cfg.torque_limit_nm))
        self.assertTrue(np.any(np.abs(result.torque) == cfg.torque_limit_nm))

    def test_gap_and_invalid_pose_abort(self):
        self.ctrl.update(self.q, 0)
        for q, now, message in ((self.q, .2, "deadline"),
                                (np.array([0, -1, 0, 0, 0, 0]), .02, "outside")):
            with self.subTest(message=message), self.assertRaisesRegex((ValueError, RuntimeError), message):
                self.ctrl.update(q, now)

    def test_gravity_above_limit_is_rejected_instead_of_silently_undercompensated(self):
        self.ctrl.settings = override(self.cfg, torque_limit_nm=np.full(6, .001))
        with self.assertRaisesRegex(RuntimeError, "gravity torque"):
            self.ctrl.update(self.q, 0)


if __name__ == "__main__":
    unittest.main()
