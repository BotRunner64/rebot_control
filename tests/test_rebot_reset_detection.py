"""Synthetic reset signals separate repeated tasks from return-to-home pauses."""

import unittest

import numpy as np

from rebot_reset_detection import STEP_NS, suggest_ranges


def pose_series(position, angle=None):
    result = np.zeros((len(position), 12))
    result[:, 0] = position
    angle = np.zeros(len(position)) if angle is None else angle
    for index, value in enumerate(angle):
        c, s = np.cos(value), np.sin(value)
        result[index, 3:] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).flatten(order="F")
    return result


class ResetDetectionTests(unittest.TestCase):
    def setUp(self):
        self.duration = 23_000_000_000
        self.grid = np.arange(0, self.duration, STEP_NS, dtype=np.int64)
        self.t = self.grid / 1e9
        keys = [0, 2, 3, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 23]
        values = [0, 0, .2, .2, 0, 0, .2, .2, 0, 0, .2, .2, 0, 0]
        self.position = np.interp(self.t, keys, values)
        self.poses = {"left": pose_series(self.position), "right": pose_series(self.position * .8)}
        self.valid = np.ones(len(self.grid), dtype=bool)

    def detect(self, **options):
        return suggest_ranges(self.grid, self.poses, self.valid, self.duration, **options)

    def test_three_tasks_from_stationary_home_returns_without_cutting_far_pauses(self):
        result = self.detect(reset_lead_seconds=2)
        self.assertTrue(result["usable"])
        self.assertEqual(len(result["episodes"]), 3)
        self.assertEqual(len(result["resets"]), 4)
        for episode, active in zip(result["episodes"], (3.5, 10.5, 17.5)):
            self.assertLess(episode["start_ns"], active * 1e9)
            self.assertGreater(episode["end_ns"], active * 1e9)
        self.assertLess(result["reference_ns"], 2e9)
        self.assertEqual(result["resets"][0]["start_ns"], 0)
        self.assertEqual(result["resets"][-1]["end_ns"], self.duration)

    def test_other_arm_must_also_be_near_reference(self):
        right = self.position.copy()
        right[self.t >= 9] += .1
        self.poses["right"] = pose_series(right)
        result = self.detect()
        self.assertEqual(len(result["resets"]), 2)

    def test_pose_orientation_prevents_false_reset_at_same_translation(self):
        angles = np.where(self.t >= 9, np.pi / 2, 0)
        self.poses["left"] = pose_series(self.position, angles)
        result = self.detect()
        self.assertEqual(len(result["resets"]), 2)

    def test_short_home_pause_is_not_a_reset(self):
        self.position[(self.t >= 7) & (self.t < 8.7)] = .1
        self.poses = {side: pose_series(self.position) for side in ("left", "right")}
        self.assertEqual(len(self.detect()["resets"]), 3)

    def test_all_stationary_has_no_fabricated_task_episodes(self):
        self.poses = {side: pose_series(np.zeros(len(self.grid))) for side in ("left", "right")}
        result = self.detect()
        self.assertTrue(result["usable"])
        self.assertEqual(result["episodes"], [])
        self.assertEqual(len(result["resets"]), 1)

    def test_feedback_gap_does_not_become_a_task_or_establish_stillness(self):
        self.poses = {side: pose_series(np.zeros(len(self.grid))) for side in ("left", "right")}
        self.valid[(self.t >= 8) & (self.t < 11)] = False
        self.assertEqual(self.detect()["episodes"], [])

    def test_explicit_reference_and_missing_initial_still_pose(self):
        moving = self.t * .03
        self.poses = {side: pose_series(moving) for side in ("left", "right")}
        self.assertFalse(self.detect()["usable"])
        self.poses = {side: pose_series(self.position) for side in ("left", "right")}
        self.assertEqual(len(self.detect(reference_ns=8_000_000_000)["episodes"]), 3)
        self.valid[self.t == 8] = False
        with self.assertRaisesRegex(ValueError, "No recent feedback"):
            self.detect(reference_ns=8_000_000_000)

    def test_invalid_reference_and_thresholds_are_rejected(self):
        for options in ({"reference_ns": -1}, {"reference_ns": True}, {"home_radius_cm": float("nan")},
                        {"still_seconds": .01}, {"reset_lead_seconds": -1}, {"unknown": 1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.detect(**options)


if __name__ == "__main__":
    unittest.main()
