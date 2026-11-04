"""Synthetic identification and virtual-CAN automatic sampling checks."""

from contextlib import redirect_stdout, redirect_stderr
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import yaml

import rebot_gravity as gravity
import rebot_hand_guiding as drive
import rebot_identify as identify
import rebot_mit as mit
from test_rebot_hand_guiding import VirtualMotors


def dataset(path, scales, independent=False, corrupt_validation=False, friction=0.):
    model = gravity.GravityModel()
    cfg = drive.load_settings()
    plan = identify.make_plan([0, .7, 1.1, 0, 0, 0], independent=independent)
    metadata = {"type": "metadata", "version": 3, "control": "native_mit", "side": "right",
                "motor_uids": {i: i.to_bytes(8, "little").hex() for i in range(1, 8)},
                "reference_rad": [0.] * 6,
                "model_sha256": model.sha256, "plan": plan, "settings": cfg, "sample_s": .5}
    with path.open("w") as stream:
        stream.write(json.dumps(identify.plain(metadata)) + "\n")
        for pose in plan["poses"]:
            if pose["stage"] != "sample":
                continue
            q = np.array(pose["q"])
            tau = model.basis(q) @ scales + pose["direction"] * np.broadcast_to(friction, (6,))
            if corrupt_validation and pose["split"] == "validation":
                tau[1] += 2.
            for step in range(30):
                row = {"type": "sample", "pose_id": pose["id"], "window": 0,
                       "time_s": pose["id"] + step * .02, "q": q, "target": q,
                       "estimated": tau, "reported": tau, "requested": tau,
                       "velocity": np.zeros(6), "raw_velocity": np.zeros(6),
                       "eligible": True, "saturated": False, "phase": "SAMPLE"}
                stream.write(json.dumps(identify.plain(row)) + "\n")
            stream.write(json.dumps({"type": "pose_complete", "pose_id": pose["id"], "window": 0}) + "\n")
        stream.write(json.dumps({"type": "end", "status": "completed"}) + "\n")
    return metadata


class ModelTests(unittest.TestCase):
    def test_basis_matches_gravity_and_export_preserves_fixed_gripper_mass(self):
        model = gravity.GravityModel()
        scales = np.array([1, 1.1, 1.45, 1.2, 1.1, 1.3])
        q = np.array([.1, .6, 1.1, .3, -.2, .4])
        np.testing.assert_allclose(model.basis(q) @ np.ones(6), model.gravity(q), atol=1e-10)
        predicted = model.basis(q) @ scales
        model.apply_scales(scales)
        np.testing.assert_allclose(predicted, model.gravity(q), atol=1e-10)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "calibrated.urdf"
            model.export_urdf(scales, output)
            reloaded = gravity.GravityModel(output)
            np.testing.assert_allclose(reloaded.gravity(q), predicted, atol=1e-10)
            with self.assertRaises(FileExistsError):
                model.export_urdf(scales, output)

    def test_plan_rejects_nonfinite_start_and_old_random_plans(self):
        with self.assertRaises(ValueError):
            identify.make_plan([0, float("nan"), 1.2, 0, .1, 0])
        with self.assertRaisesRegex(ValueError, "old random plans"):
            identify.validate_plan({"version": 1}, gravity.GravityModel(), drive.load_settings())

    def test_fixed_single_joint_sweeps_unfold_before_lifting(self):
        plan = identify.make_plan(np.zeros(6))
        entry = [p for p in plan["poses"] if p["stage"] == "entry"]
        np.testing.assert_allclose(entry[0]["q"], [0, 0, 1.1, 0, 0, 0])
        np.testing.assert_allclose(entry[1]["q"], [0, .7, 1.1, 0, 0, 0])
        samples = [p for p in plan["poses"] if p["stage"] == "sample"]
        self.assertEqual(len(samples), 48)
        self.assertEqual(sum(p["split"] == "train" for p in samples), 32)
        q = np.array([p["q"] for p in samples])
        self.assertGreater(q[:, 1].min(), .4)
        self.assertGreater(np.ptp(q[:, 1]), .8)
        self.assertGreater(np.ptp(q[:, 2]), 1.)
        self.assertGreater(np.ptp(q[:, 4]), 1.5)
        model = gravity.GravityModel()
        matrix = np.array([model.basis(p["q"])[p["axis"] - 1, 1:] for p in samples])
        self.assertEqual(np.linalg.matrix_rank(matrix), 5)
        self.assertLess(np.linalg.cond(matrix), 500.)
        self.assertEqual({p["axis"] for p in samples}, {2, 3, 4, 5})
        self.assertTrue(np.all(q[:, 5] == 0.))
        alternate = identify.make_plan([0, .9, 1.4, .1, .1, .1])
        np.testing.assert_allclose(q, [p["q"] for p in alternate["poses"] if p["stage"] == "sample"])

    def test_tracking_error_only_rejects_samples_and_feedback_gaps_stop(self):
        model = gravity.GravityModel()
        sampler = identify.MotionController(model, drive.load_settings())
        q = np.array([0, .6, 1.2, 0, .1, 0])
        sampler.update(q, q, 0.)
        with self.assertRaisesRegex(RuntimeError, "feedback gap"):
            sampler.update(q, q, .2)
        sampler = identify.MotionController(model, drive.load_settings())
        result = sampler.update(q, q + .21, 0.)
        self.assertFalse(result["stable"])
        np.testing.assert_allclose(result["target"], q + .21)

    def test_motion_uses_native_pd_and_gravity_only_feedforward(self):
        model = gravity.GravityModel()
        cfg = identify.motion_settings(drive.load_settings())
        controller = identify.MotionController(model, cfg)
        q = np.array([0, .6, 1.2, 0, .1, 0])
        target = q + [0, .01, -.01, .01, .01, .01]
        velocity = np.array([0, .01, -.01, .01, .01, .01])
        record = controller.update(q, target, 0., velocity)
        np.testing.assert_allclose(record["target"], target)
        raw = q + .5
        models = ["rs-06"] * 3 + ["rs-00"] * 3
        bus = MagicMock()

        def send(positions, velocities, feedforward, kp, kd, deadline):
            np.testing.assert_allclose(positions, target + .5)
            np.testing.assert_allclose(velocities, velocity)
            np.testing.assert_allclose(feedforward, model.gravity(q) * cfg["gravity_scale"])
            self.assertTrue(np.all(kp > 0))
            self.assertTrue(np.all(kd > 0))
            return [mit.decode_mit(mit.encode_mit(i, models[i-1], p, tau, k, d, velocity=v), models[i-1])
                    for i, (p, v, tau, k, d) in enumerate(zip(positions, velocities, feedforward, kp, kd), 1)]

        bus.command_position.side_effect = send
        controller.command(bus, raw, record, .08)
        bus.command_position.assert_called_once()
        self.assertIn("native_parameters", record)
        self.assertTrue(np.all(np.isfinite(record["estimated"])))

    def test_effort_budget_does_not_distort_position_and_velocity_targets(self):
        cfg = identify.motion_settings(drive.load_settings())
        controller = identify.MotionController(gravity.GravityModel(), cfg)
        target = np.array([0., .6, 2., 0., .1, 0.])
        velocity = np.array([0., 0., .04, 0., 0., 0.])
        sent = []
        models = ["rs-06"] * 3 + ["rs-00"] * 3
        bus = MagicMock()

        def send(positions, velocities, feedforward, kp, kd, deadline):
            sent.append((np.array(positions), np.array(velocities)))
            return [mit.decode_mit(mit.encode_mit(i, models[i-1], p, tau, k, d, velocity=v), models[i-1])
                    for i, (p, v, tau, k, d) in enumerate(zip(positions, velocities, feedforward, kp, kd), 1)]

        bus.command_position.side_effect = send
        for step, offset in enumerate((0., .003, -.003)):
            q = np.array([0, .6, 1.2 + offset, 0, .1, 0])
            record = controller.update(q, target, step * .02, velocity)
            self.assertTrue(record["saturated"])
            self.assertFalse(record["stable"])
            controller.command(bus, q + .5, record, 1.)
        for position, speed in sent:
            np.testing.assert_allclose(position, target + .5)
            np.testing.assert_array_equal(speed, velocity)

    def test_minimum_jerk_has_zero_velocity_at_both_ends(self):
        start, goal = np.zeros(6), np.ones(6)
        first, first_velocity = identify.minimum_jerk(start, goal, 0., 2.)
        last, last_velocity = identify.minimum_jerk(start, goal, 3., 2.)
        np.testing.assert_array_equal(first, start)
        np.testing.assert_array_equal(last, goal)
        np.testing.assert_array_equal(first_velocity, np.zeros(6))
        np.testing.assert_array_equal(last_velocity, np.zeros(6))

    def test_movement_duration_matches_peak_speed_without_two_second_floor(self):
        start = np.zeros(6)
        goal = np.array([0., np.pi / 6, 0., 0., 0., 0.])
        duration = identify.movement_duration(start, goal, .2)
        self.assertAlmostEqual(duration, 4.908738521, places=6)
        _, velocity = identify.minimum_jerk(start, goal, duration / 2, duration)
        self.assertAlmostEqual(np.max(np.abs(velocity)), .2)
        self.assertEqual(identify.movement_duration(start, np.full(6, .01), .2), .5)


class FittingTests(unittest.TestCase):
    def test_only_swept_joint_effort_is_used_in_the_mass_fit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "data.jsonl"
            scales = np.array([1, 1.1, 1.35, 1.2, 1.15, 1.25])
            meta = dataset(path, scales)
            poses = {pose["id"]: pose for pose in meta["plan"]["poses"]}
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            for row in rows:
                if row["type"] == "sample":
                    axis = poses[row["pose_id"]]["axis"] - 1
                    row["estimated"] = [value + (.3 if i != axis else 0.)
                                        for i, value in enumerate(row["estimated"])]
            path.write_text("\n".join(json.dumps(row) for row in rows))
            report = identify.fit(path, root / "fit", regularization=1e-6)
            self.assertTrue(report["accepted"])
            np.testing.assert_allclose(report["mass_scales"], scales, atol=.002)

    def test_recover_mass_scales_and_validate_independent_session(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            scales = np.array([1, 1.1, 1.35, 1.2, 1.15, 1.25])
            meta = dataset(root / "data.jsonl", scales, friction=[0, .4, .35, .25, .15, .1])
            report = identify.fit(root / "data.jsonl", root / "fit", regularization=1e-6)
            self.assertTrue(report["accepted"])
            np.testing.assert_allclose(report["mass_scales"], scales, atol=.002)
            np.testing.assert_allclose(report["train"]["median_approach_hysteresis_nm"][1:5],
                                       [.4, .35, .25, .15], atol=.002)
            profile_path = root / "fit" / "calibration.yaml"
            arm = {"motor_uids": meta["motor_uids"], "reference_rad": meta["reference_rad"]}
            model = gravity.GravityModel(calibration=profile_path, arm=arm, side="right")
            np.testing.assert_allclose(model.scales, scales, atol=.002)
            virtual = VirtualMotors()
            arm["models"] = ["rs-06"] * 3 + ["rs-00"] * 4
            expected = model.gravity(np.full(6, .2))
            with patch.object(mit, "check_channel"), \
                    patch.object(mit.can, "Bus", return_value=virtual), \
                    patch.object(drive, "load_config", return_value={"right": arm}), \
                    patch.object(drive, "resolve_channel", return_value="can0"):
                drive.run(side="right", duration=.025, calibration=profile_path)
            for frame in virtual.sent:
                if (frame.arbitration_id >> 24) & 31 != 1:
                    continue
                motor = frame.arbitration_id & 255
                bound = mit.LIMITS[arm["models"][motor-1]].torque
                actual = ((frame.arbitration_id >> 8) & 65535) / 65535 * 2 * bound - bound
                self.assertAlmostEqual(actual, expected[motor-1], delta=.003)
            with self.assertRaisesRegex(ValueError, "different collection"):
                identify.validate(root / "data.jsonl", profile_path)
            dataset(root / "independent.jsonl", scales, independent=True, friction=[0, .4, .35, .25, .15, .1])
            self.assertTrue(identify.validate(root / "independent.jsonl", profile_path)["accepted"])
            duplicate = root / "duplicate.jsonl"
            duplicate.write_text((root / "data.jsonl").read_text() + "\n")
            with self.assertRaisesRegex(ValueError, "targets distinct"):
                identify.validate(duplicate, profile_path)
            independent = root / "independent.jsonl"
            records = [json.loads(line) for line in independent.read_text().splitlines()]
            pairs = {p["id"]: p.get("pair_id") for p in records[0]["plan"]["poses"]}
            records = [r for r in records if r["type"] != "pose_complete" or pairs[r["pose_id"]] < 7]
            independent.write_text("\n".join(json.dumps(r) for r in records))
            with self.assertRaisesRegex(ValueError, "at least 8 complete"):
                identify.validate(independent, profile_path)
            with self.assertRaisesRegex(ValueError, "do not match"):
                gravity.GravityModel(calibration=profile_path, arm=arm, side="left")
            arm["reference_rad"] = [.01] * 6
            with self.assertRaisesRegex(ValueError, "do not match"):
                gravity.GravityModel(calibration=profile_path, arm=arm, side="right")

    def test_failed_validation_exports_report_without_loadable_profile(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            dataset(root / "data.jsonl", [1, 1.1, 1.2, 1.1, 1.1, 1.1], corrupt_validation=True)
            report = identify.fit(root / "data.jsonl", root / "fit")
            self.assertFalse(report["accepted"])
            self.assertIn("J2", report["reason"])
            self.assertTrue(report["reasons"])
            self.assertTrue((root / "fit" / "report.json").exists())
            self.assertFalse((root / "fit" / "calibration.yaml").exists())

    def test_incomplete_pairs_and_saturated_records_are_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.jsonl"
            metadata = dataset(path, np.ones(6))
            sample_ids = {p["id"] for p in metadata["plan"]["poses"]
                          if p.get("pair_id") == 0 and p.get("direction") == 1}
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            for row in rows:
                if row.get("type") == "sample" and row["pose_id"] in sample_ids:
                    row["saturated"] = True
            path.write_text("\n".join(json.dumps(row) for row in rows))
            _, accepted = identify.load_dataset(path)
            self.assertEqual(len(accepted), 23)
            self.assertNotIn(0, [row["pair_id"] for row in accepted])

    def test_one_invalid_frame_discards_its_whole_completed_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.jsonl"
            dataset(path, np.ones(6))
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            row = next(r for r in rows if r.get("type") == "sample")
            pose = next(p for p in rows[0]["plan"]["poses"] if p["id"] == row["pose_id"])
            row["raw_velocity"][2] = .03
            path.write_text("\n".join(json.dumps(r) for r in rows))
            _, accepted = identify.load_dataset(path)
            self.assertEqual(len(accepted), 23)
            self.assertNotIn(pose["pair_id"], [pair["pair_id"] for pair in accepted])

    def test_old_unpaired_data_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.jsonl"
            dataset(path, np.ones(6))
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            rows[0]["version"] = 1
            rows[0].pop("control")
            path.write_text("\n".join(json.dumps(row) for row in rows))
            with self.assertRaisesRegex(ValueError, "recollect old unpaired data"):
                identify.load_dataset(path)

    def test_pair_split_leakage_is_rejected(self):
        plan = identify.make_plan([0, .7, 1.1, 0, 0, 0])
        samples = [p for p in plan["poses"] if p.get("pair_id") == 0]
        samples[1]["split"] = "validation"
        with self.assertRaisesRegex(ValueError, "one split"):
            identify.validate_plan(plan, gravity.GravityModel(), drive.load_settings())

    def test_uninformative_measurements_cannot_export_a_mass_profile(self):
        model = gravity.GravityModel()
        rows = []
        for i in range(24):
            q = np.array([i * .05, .7, 1.1, 0, 0, 0])
            rows.append({"q": q, "basis": model.basis(q), "torque": model.gravity(q),
                         "axis": 1, "split": "validation" if i % 4 == 3 else "train"})
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(identify, "load_dataset", return_value=({}, rows)):
            root = Path(directory)
            report = identify.fit(root / "unused.jsonl", root / "fit")
            self.assertFalse(report["accepted"])
            self.assertTrue(report["rank"] < 5 or report["condition"] > 1e4)
            self.assertFalse((root / "fit" / "calibration.yaml").exists())

    def test_displaced_passes_average_regressors_not_just_angles(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.jsonl"
            meta = dataset(path, np.ones(6))
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            poses = {p["id"]: p for p in meta["plan"]["poses"]}
            model = gravity.GravityModel()
            for row in rows:
                if row.get("type") != "sample":
                    continue
                pose = poses[row["pose_id"]]
                q = np.array(row["q"]) + pose["direction"] * np.array([0, .01, .015, .01, .01, .01])
                row["q"] = q.tolist()
                tau = model.gravity(q) + pose["direction"] * np.array([0, .2, .2, .1, .1, .05])
                row["estimated"] = row["requested"] = tau.tolist()
            path.write_text("\n".join(json.dumps(row) for row in rows))
            _, pairs = identify.load_dataset(path)
            self.assertEqual(len(pairs), 24)
            for pair in pairs:
                np.testing.assert_allclose(pair["basis"] @ np.ones(6), pair["torque"], atol=1e-10)



class WorkflowTests(unittest.TestCase):
    start = [0, .7, 1.1, 0, 0, 0]

    def setUp(self):
        item = patch.object(identify, "preflight")
        self.preflight = item.start()
        self.addCleanup(item.stop)

    def fake_collect(self, plan, output, **options):
        dataset(Path(output), [1, 1.1, 1.35, 1.2, 1.15, 1.25],
                independent=Path(plan).name == "validation_plan.yaml")
        return "completed"

    def test_one_command_runs_both_sessions_and_only_then_exports_profile(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect", side_effect=self.fake_collect) as collect:
            root = Path(directory) / "run"
            result = identify.run_calibration(root, start=self.start)
            self.assertTrue(result["accepted"])
            self.assertEqual(collect.call_count, 2)
            self.assertTrue((root / "calibration.yaml").exists())
            self.assertTrue((root / "calibrated.urdf").exists())
            self.assertTrue((root / "report.json").exists())
            profile = yaml.safe_load((root / "calibration.yaml").read_text())
            self.assertTrue(profile["independent_validation"]["accepted"])
            with self.assertRaisesRegex(ValueError, "already exists"):
                identify.run_calibration(root, start=self.start)
            self.assertEqual(collect.call_count, 2)
            self.preflight.assert_called_once()

    def test_default_reads_start_and_runs_fixed_clearance_sampling(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "read_current_pose", return_value=np.array(self.start)) as read, \
                patch.object(identify, "collect", side_effect=self.fake_collect) as collect:
            root = Path(directory) / "run"
            result = identify.run_calibration(root)
            self.assertTrue(result["accepted"])
            self.assertEqual(collect.call_count, 2)
            read.assert_called_once()
            session = yaml.safe_load((root / "session.yaml").read_text())
            np.testing.assert_allclose(session["start"], self.start)

    def test_interrupted_first_round_does_not_fit_or_start_another_round(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect", return_value="interrupted") as collect, \
                patch.object(identify, "fit") as fit:
            root = Path(directory) / "run"
            result = identify.run_calibration(root, start=self.start)
            self.assertEqual(result["status"], "interrupted")
            collect.assert_called_once()
            fit.assert_not_called()
            self.assertFalse((root / "calibration.yaml").exists())

    def test_interrupted_preflight_never_starts_collection(self):
        self.preflight.side_effect = KeyboardInterrupt
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect") as collect, patch.object(identify, "fit") as fit:
            root = Path(directory) / "run"
            with self.assertRaises(KeyboardInterrupt):
                identify.run_calibration(root, start=self.start)
            collect.assert_not_called()
            fit.assert_not_called()
            report = json.loads((root / "report.json").read_text())
            self.assertEqual(report["status"], "interrupted")

    def test_rejected_fit_does_not_start_independent_round(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect", side_effect=self.fake_collect) as collect, \
                patch.object(identify, "fit", return_value={"accepted": False}):
            root = Path(directory) / "run"
            result = identify.run_calibration(root, start=self.start)
            self.assertEqual(result["status"], "rejected")
            collect.assert_called_once()
            self.assertFalse((root / "calibration.yaml").exists())

    def test_failed_independent_validation_does_not_export_final_profile(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect", side_effect=self.fake_collect), \
                patch.object(identify, "validate", return_value={"accepted": False}):
            root = Path(directory) / "run"
            result = identify.run_calibration(root, start=self.start)
            self.assertEqual(result["status"], "rejected")
            self.assertEqual(result["stage"], "independent_validation")
            self.assertFalse((root / "calibration.yaml").exists())

    def test_existing_directory_is_rejected_before_motor_access(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(identify, "read_current_pose") as read, patch.object(identify, "collect") as collect:
            with self.assertRaisesRegex(ValueError, "already exists"):
                identify.run_calibration(directory)
            read.assert_not_called()
            collect.assert_not_called()
            self.preflight.assert_not_called()

    def test_cli_without_enable_runs_the_workflow(self):
        with patch.object(identify.sys, "argv", ["rebot_identify.py"]), \
                patch.object(identify, "run_calibration", return_value={"status": "completed", "accepted": True}) as run:
            self.assertEqual(identify.main(), 0)
            run.assert_called_once_with(side="right", speed=.2)

    def test_old_subcommands_and_unneeded_flags_are_not_exposed(self):
        for command in ("plan", "collect", "fit", "validate", "--enable", "--seed", "--count", "--effort", "--span"):
            with patch.object(identify.sys, "argv", ["rebot_identify.py", command]), \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                identify.main()
            self.assertEqual(error.exception.code, 2)


class CollectTests(unittest.TestCase):
    def setup_bus(self):
        self.virtual = VirtualMotors()
        self.arm = {"models": ["rs-06"] * 3 + ["rs-00"] * 4,
                    "reference_rad": (np.r_[np.full(6, .2) - identify.clearance_pose(0), 0.]).tolist(),
                    "motor_uids": self.virtual.uids}
        for item in (patch.object(mit, "check_channel"),
                     patch.object(mit.can, "Bus", return_value=self.virtual),
                     patch.object(identify, "load_config", return_value={"right": self.arm}),
                     patch.object(identify, "resolve_channel", return_value="can0")):
            item.start()
            self.addCleanup(item.stop)
        self.keys = MagicMock()
        self.keys.__enter__.return_value.pressed.return_value = False
        item = patch.object(identify, "StopKey", return_value=self.keys)
        item.start()
        self.addCleanup(item.stop)
        self.clock = [0.]

        def now():
            self.clock[0] += .00001
            return self.clock[0]

        def sleep(seconds):
            self.clock[0] += seconds

        for item in (patch.object(identify.time, "monotonic", now),
                     patch.object(identify.time, "sleep", sleep)):
            item.start()
            self.addCleanup(item.stop)

    def plan(self, path, goal=None):
        goal = np.array(goal if goal is not None else identify.clearance_pose(0))
        poses = []
        for direction in (1, -1):
            poses.append({"id": len(poses), "stage": "transit", "q": (goal - direction * np.array([0, 0, 0, 0, identify.APPROACH_RAD, 0])).tolist()})
            poses.append({"id": len(poses), "stage": "sample", "q": goal.tolist(),
                          "pair_id": 0, "axis": 5, "direction": direction, "split": "train"})
        identify.write_yaml(path, {"version": 2, "method": "paired_static", "units": "rad",
                                  "model_sha256": gravity.GravityModel().sha256,
                                  "start": identify.clearance_pose(0).tolist(), "poses": poses})

    def assert_holding(self):
        self.assertTrue(self.virtual.closed)
        self.assertEqual(set(self.virtual.modes.values()), {2})
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in self.virtual.sent))
        frames = [m for m in self.virtual.sent if (m.arbitration_id >> 24) & 31 == 1]
        self.assertTrue(all(int.from_bytes(m.data[4:6], "big") > 0 for m in frames[-6:]))
        self.assertTrue(all((m.arbitration_id & 255) <= 6 for m in frames))

    def test_internal_preflight_never_writes_motor_parameters(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.plan(root / "plan.yaml")
            identify.preflight([root / "plan.yaml"])
            self.assertFalse((root / "data.jsonl").exists())
        self.assertTrue(all((m.arbitration_id >> 24) & 31 in (0, 17) for m in self.virtual.sent))

    def test_initial_pose_is_read_without_motor_writes(self):
        self.setup_bus()
        np.testing.assert_allclose(identify.read_current_pose(identify.CONFIG, "right"), identify.clearance_pose(0), atol=1e-6)
        self.assertTrue(all((m.arbitration_id >> 24) & 31 in (0, 17) for m in self.virtual.sent))

    def test_automatic_collection_completes_and_holds(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            self.plan(root / "plan.yaml")
            identify.collect(root / "plan.yaml", root / "data.jsonl",
                             settle_s=.5, sample_s=.5)
            rows = [json.loads(line) for line in (root / "data.jsonl").read_text().splitlines()]
            self.assertEqual(rows[0]["control"], "native_mit")
            self.assertTrue(all("native_parameters" in row for row in rows if row["type"] == "sample"))
            self.assertEqual(rows[-1], {"type": "end", "status": "completed"})
            self.assertTrue(any(r["type"] == "pose_complete" for r in rows))
            _, poses = identify.load_dataset(root / "data.jsonl")
            self.assertEqual(len(poses), 1)
        self.assert_holding()

    def test_next_pose_starts_at_previous_target_instead_of_feedback(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            poses = []
            for pair_id, angle in enumerate((.008, .012)):
                goal = identify.clearance_pose(0) + [0, 0, 0, 0, angle, 0]
                for direction in (1, -1):
                    poses.append({"id": len(poses), "stage": "transit", "q": (goal - direction * np.array([0, 0, 0, 0, identify.APPROACH_RAD, 0])).tolist()})
                    poses.append({"id": len(poses), "stage": "sample", "q": goal.tolist(),
                                  "pair_id": pair_id, "axis": 5, "direction": direction, "split": "train"})
            plan = {"version": 2, "method": "paired_static", "units": "rad",
                    "model_sha256": gravity.GravityModel().sha256, "start": identify.clearance_pose(0).tolist(), "poses": poses}
            identify.write_yaml(root / "plan.yaml", plan)
            identify.collect(root / "plan.yaml", root / "data.jsonl", settle_s=.5, sample_s=.5)
            rows = [json.loads(line) for line in (root / "data.jsonl").read_text().splitlines()]
            samples = [row for row in rows if row["type"] == "sample"]
            first_next = next(row for row in samples if row["pose_id"] == 4)
            self.assertAlmostEqual(first_next["target"][4], .008, delta=.0001)
            self.assertAlmostEqual(first_next["q"][4], 0., delta=.0001)
            steps = np.diff([row["target"][4] for row in samples])
            self.assertLess(np.max(np.abs(steps)), .005)
            sent_steps = np.diff([row["native_parameters"][4]["position"] for row in samples])
            self.assertLess(np.max(np.abs(sent_steps)), .005)
        self.assert_holding()

    def test_space_stop_keeps_enabled_and_marks_interruption(self):
        self.setup_bus()
        self.keys.__enter__.return_value.pressed.side_effect = lambda: self.clock[0] > .5
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            self.plan(root / "plan.yaml")
            identify.collect(root / "plan.yaml", root / "data.jsonl")
            rows = [json.loads(line) for line in (root / "data.jsonl").read_text().splitlines()]
            self.assertEqual(rows[-1]["status"], "interrupted")
            self.assertFalse(any(r["type"] == "pose_complete" for r in rows))
        self.assert_holding()

    def test_large_tracking_lag_keeps_running_past_old_timeout_until_manual_stop(self):
        self.setup_bus()
        self.keys.__enter__.return_value.pressed.side_effect = lambda: self.clock[0] > 35.
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            self.plan(root / "plan.yaml", [0, .82, 1.1, 0, 0, .3])
            result = identify.collect(root / "plan.yaml", root / "data.jsonl", settle_s=.5, sample_s=.5)
            self.assertEqual(result, "interrupted")
            self.assertGreater(self.clock[0], 35.)
            _, rows = identify.load_dataset(root / "data.jsonl")
            self.assertEqual(rows, [])
        self.assert_holding()

    def test_fault_during_sampling_still_attempts_native_hold(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                redirect_stderr(io.StringIO()):
            root = Path(directory)
            self.plan(root / "plan.yaml")
            with patch.object(identify.MotionController, "update", side_effect=RuntimeError("injected fault")):
                with self.assertRaisesRegex(RuntimeError, "injected fault"):
                    identify.collect(root / "plan.yaml", root / "data.jsonl")
        self.assert_holding()

    def test_existing_dataset_is_not_overwritten_or_arm_enabled(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.plan(root / "plan.yaml")
            (root / "data.jsonl").write_text("existing data")
            with self.assertRaises(FileExistsError):
                identify.collect(root / "plan.yaml", root / "data.jsonl")
            self.assertEqual((root / "data.jsonl").read_text(), "existing data")
        self.assertEqual(self.virtual.sent, [])


if __name__ == "__main__":
    unittest.main()
