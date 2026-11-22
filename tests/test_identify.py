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

from rebot.apps import guide as guide_app
from rebot.apps import identify
from rebot.bindings import ArmBinding
from rebot.devices import robstride
from rebot.devices.robstride import MODELS
from rebot.model import gravity, gravity_fit, gravity_trials
from rebot.model.motion_collision import MotionCollision, PATH_STEP_RAD
from rebot.model.trajectory import (MOTION_KD, MOTION_KP, TrajectoryTracker, minimum_jerk,
                                    movement_duration, plain)
from rebot.paths import ARM_BINDINGS
from rebot.runtime.arm import Arm
from rebot.settings import load_settings
from fakes import VirtualMotors, patch_hardware


def uids():
    return {i: i.to_bytes(8, "little").hex() for i in range(1, 8)}


def binding(side="right", reference=0., motor_uids=None):
    """A calibrated arm whose six joints share one raw reference angle."""
    return ArmBinding(side, "can0", MODELS, uids() if motor_uids is None else motor_uids,
                      np.r_[np.broadcast_to(reference, (6,)), 0.])


def open_arm(model, cfg, raw, reference=.5):
    """An opened `Arm` on a mock bus that reports `raw` motor angles."""
    bus = MagicMock()
    bus.__enter__.return_value = bus
    bus.scan.return_value = uids()
    bus.positions.return_value = list(raw)
    arm = Arm(binding(reference=reference), cfg, gravity=model, joint_limits=False,
              bus_factory=lambda *args, **kwargs: bus)
    arm.__enter__()
    bus.command_position.assert_not_called()
    return arm, bus


def dataset(path, scales, independent=False, corrupt_validation=False, friction=0.,
            experiment="identification", candidate=None, payload=None, side="right", speed=.2):
    model = gravity.GravityModel()
    cfg = load_settings()
    plan = gravity_fit.make_plan([0, .7, 1.1, 0, 0, 0], cfg, independent=independent)
    metadata = {"type": "metadata", "version": 3, "control": "native_mit", "side": side,
                "motor_uids": uids(),
                "reference_rad": [0.] * 6,
                "model_sha256": model.sha256, "plan": plan, "settings": cfg.plain(), "sample_s": .5}
    applied = np.ones(6) if candidate is None else np.array(candidate["mass_scales"])
    metadata.update(experiment=experiment, applied_mass_scales=applied.tolist(), payload=payload, speed=speed)
    if candidate is not None:
        metadata["settings"]["gravity_scale"] = [1.] * 6
    loaded_model = gravity.GravityModel()
    if payload is not None:
        loaded_model.add_point_payload(payload["mass_kg"], payload["com_link6_m"])
    with path.open("w") as stream:
        stream.write(json.dumps(plain(metadata)) + "\n")
        for pose in plan["poses"]:
            if pose["stage"] != "sample":
                continue
            q = np.array(pose["q"])
            tau = model.basis(q) @ scales + pose["direction"] * np.broadcast_to(friction, (6,))
            if payload is not None:
                tau += loaded_model.gravity(q) - model.gravity(q)
            ff = (model.basis(q) @ applied) * metadata["settings"]["gravity_scale"]
            if corrupt_validation and pose["split"] == "validation":
                tau[1] += 2.
            for step in range(30):
                row = {"type": "sample", "pose_id": pose["id"], "window": 0,
                       "time_s": pose["id"] + step * .02, "q": q, "target": q,
                       "estimated": tau, "reported": tau, "requested": tau,
                       "native_parameters": [{"gravity": g} for g in ff],
                       "velocity": np.zeros(6), "raw_velocity": np.zeros(6),
                       "eligible": True, "saturated": False, "phase": "SAMPLE"}
                stream.write(json.dumps(plain(row)) + "\n")
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

    def test_fixed_single_joint_sweeps_unfold_before_lifting(self):
        plan = gravity_fit.make_plan(np.zeros(6), load_settings())
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
        alternate = gravity_fit.make_plan([0, .9, 1.4, .1, .1, .1], load_settings())
        np.testing.assert_allclose(q, [p["q"] for p in alternate["poses"] if p["stage"] == "sample"])

    def test_fixed_sweeps_keep_gripper_at_least_200_mm_from_table(self):
        import mujoco

        scene = MotionCollision("right", np.zeros(7))
        table = scene.model.geom("work_table").id
        gripper = [i for i, name in enumerate(scene.names) if name.startswith("right_gripper")]
        self.assertTrue(gripper)
        for independent in (False, True):
            start = gravity_fit.CLEARANCE
            plan = gravity_fit.make_plan(start, load_settings(), independent)
            for pose in plan["poses"]:
                goal = np.array(pose["q"])
                steps = int(np.ceil(np.max(np.abs(goal - start)) / PATH_STEP_RAD))
                for q in np.linspace(start, goal, steps + 1):
                    scene.data.qpos[scene.joints] = q
                    mujoco.mj_kinematics(scene.model, scene.data)
                    distance = min(mujoco.mj_geomDistance(scene.model, scene.data, table, i, .21, None)
                                   for i in gripper)
                    self.assertGreaterEqual(distance, .2, (independent, pose["id"]))
                start = goal

    def test_payload_torque_matches_potential_energy_gradient(self):
        import pinocchio as pin

        mass, com = .13, np.array([.03, -.02, .09])
        q = np.array([.1, .7, 1.2, -.35, .4, -.3])
        nominal, loaded = gravity.GravityModel(), gravity.GravityModel()
        loaded.add_point_payload(mass, com)
        def energy(q):
            pin.framesForwardKinematics(nominal.model, nominal.data, q)
            pose = nominal.data.oMf[nominal.model.getFrameId("link6")]
            return mass * 9.81 * (pose.rotation @ com + pose.translation)[2]
        steps = np.eye(6) * 1e-6
        gradient = [(energy(q + d) - energy(q - d)) / 2e-6 for d in steps]
        np.testing.assert_allclose(loaded.gravity(q) - nominal.gravity(q), gradient, atol=1e-8)

    def test_tracking_error_only_rejects_samples_and_feedback_gaps_stop(self):
        model = gravity.GravityModel()
        sampler = TrajectoryTracker(model, load_settings())
        q = np.array([0, .6, 1.2, 0, .1, 0])
        sampler.update(q, q, 0.)
        with self.assertRaisesRegex(RuntimeError, "feedback gap"):
            sampler.update(q, q, .2)
        sampler = TrajectoryTracker(model, load_settings())
        result = sampler.update(q, q + .21, 0.)
        self.assertFalse(result["stable"])
        np.testing.assert_allclose(result["target"], q + .21)

    def test_motion_uses_native_pd_and_gravity_only_feedforward(self):
        model = gravity.GravityModel()
        cfg = load_settings()
        controller = TrajectoryTracker(model, cfg)
        q = np.array([0, .6, 1.2, 0, .1, 0])
        target = q + [0, .01, -.01, .01, .01, .01]
        velocity = np.array([0, .01, -.01, .01, .01, .01])
        arm, bus = open_arm(model, cfg, q + .5)
        np.testing.assert_allclose(arm.q, q)
        record = controller.update(arm.q, target, 0., velocity)
        np.testing.assert_allclose(record["target"], target)
        models = ["rs-06"] * 3 + ["rs-00"] * 3

        def send(positions, velocities, feedforward, kp, kd, deadline):
            np.testing.assert_allclose(positions, target + .5)
            np.testing.assert_allclose(velocities, velocity)
            # arm.q is raw - reference, so it differs from q by float rounding only.
            np.testing.assert_allclose(feedforward, model.gravity(q) * cfg.gravity_scale, atol=1e-12)
            self.assertTrue(np.all(kp > 0))
            self.assertTrue(np.all(kd > 0))
            # Homing and identification move with the reference motion gains, not the guiding ones.
            np.testing.assert_array_equal(kp, MOTION_KP)
            np.testing.assert_array_equal(kd, MOTION_KD)
            # Commands computed from a sample expire with that sample.
            self.assertAlmostEqual(deadline, arm.sampled_at + cfg.max_feedback_age_s)
            return [robstride.decode_mit(robstride.encode_mit(i, models[i-1], p, tau, k, d, velocity=v),
                                         models[i-1])
                    for i, (p, v, tau, k, d) in enumerate(zip(positions, velocities, feedforward, kp, kd), 1)]

        bus.command_position.side_effect = send
        arm.command_trajectory(record)
        bus.command_position.assert_called_once()
        self.assertIn("native_parameters", record)
        self.assertTrue(np.all(np.isfinite(record["estimated"])))

    def test_effort_budget_does_not_distort_position_and_velocity_targets(self):
        model, cfg = gravity.GravityModel(), load_settings()
        controller = TrajectoryTracker(model, cfg)
        # Stay below the tracking guard while still exceeding the effort budget.
        target = np.array([0., .6, 1.9, 0., .1, 0.])
        velocity = np.array([0., 0., .04, 0., 0., 0.])
        sent = []
        models = ["rs-06"] * 3 + ["rs-00"] * 3
        arm, bus = open_arm(model, cfg, np.array([0, .6, 1.2, 0, .1, 0]) + .5)

        def send(positions, velocities, feedforward, kp, kd, deadline):
            sent.append((np.array(positions), np.array(velocities)))
            return [robstride.decode_mit(robstride.encode_mit(i, models[i-1], p, tau, k, d, velocity=v),
                                         models[i-1])
                    for i, (p, v, tau, k, d) in enumerate(zip(positions, velocities, feedforward, kp, kd), 1)]

        bus.command_position.side_effect = send
        for step, offset in enumerate((0., .003, -.003)):
            q = np.array([0, .6, 1.2 + offset, 0, .1, 0])
            bus.positions.return_value = list(q + .5)
            np.testing.assert_allclose(arm.sample(), q)
            record = controller.update(arm.q, target, step * .02, velocity)
            self.assertTrue(record["saturated"])
            self.assertFalse(record["stable"])
            arm.command_trajectory(record)
        self.assertEqual(len(sent), 3)
        for position, speed in sent:
            np.testing.assert_allclose(position, target + .5)
            np.testing.assert_array_equal(speed, velocity)

    def test_minimum_jerk_has_zero_velocity_at_both_ends(self):
        start, goal = np.zeros(6), np.ones(6)
        first, first_velocity = minimum_jerk(start, goal, 0., 2.)
        last, last_velocity = minimum_jerk(start, goal, 3., 2.)
        np.testing.assert_array_equal(first, start)
        np.testing.assert_array_equal(last, goal)
        np.testing.assert_array_equal(first_velocity, np.zeros(6))
        np.testing.assert_array_equal(last_velocity, np.zeros(6))

    def test_movement_duration_matches_peak_speed_without_two_second_floor(self):
        start = np.zeros(6)
        goal = np.array([0., np.pi / 6, 0., 0., 0., 0.])
        duration = movement_duration(start, goal, .2)
        self.assertAlmostEqual(duration, 4.908738521, places=6)
        _, velocity = minimum_jerk(start, goal, duration / 2, duration)
        self.assertAlmostEqual(np.max(np.abs(velocity)), .2)
        self.assertEqual(movement_duration(start, np.full(6, .01), .2), .5)


class FittingTests(unittest.TestCase):
    def test_only_swept_joint_effort_is_used_in_the_mass_fit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "data.jsonl"
            scales = np.array([1, 1.1, 1.35, 1.2, 1., 1.25])
            meta = dataset(path, scales)
            poses = {pose["id"]: pose for pose in meta["plan"]["poses"]}
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            for row in rows:
                if row["type"] == "sample":
                    axis = poses[row["pose_id"]]["axis"] - 1
                    row["estimated"] = [value + (.3 if i != axis else 0.)
                                        for i, value in enumerate(row["estimated"])]
            path.write_text("\n".join(json.dumps(row) for row in rows))
            report = gravity_fit.fit(path, root / "fit", regularization=1e-6)
            self.assertTrue(report["accepted"])
            np.testing.assert_allclose(report["mass_scales"], scales, atol=.002)

    def test_recover_mass_scales_and_validate_independent_session(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            scales = np.array([1, 1.1, 1.35, 1.2, 1., 1.25])
            meta = dataset(root / "data.jsonl", scales, friction=[0, .4, .35, .25, .15, .1])
            report = gravity_fit.fit(root / "data.jsonl", root / "fit", regularization=1e-6)
            self.assertTrue(report["accepted"])
            np.testing.assert_allclose(report["mass_scales"], scales, atol=.002)
            np.testing.assert_allclose(report["train"]["median_approach_hysteresis_nm"][1:5],
                                       [.4, .35, .25, .15], atol=.002)
            candidate_path = profile_path = root / "fit" / "calibration.yaml"
            self.assertFalse(yaml.safe_load(profile_path.read_text())["verified"])
            arm = binding("right", meta["reference_rad"], meta["motor_uids"])
            model = gravity.GravityModel(calibration=profile_path, binding=arm)
            np.testing.assert_allclose(model.scales, scales, atol=.002)
            virtual = VirtualMotors()
            expected = model.gravity(np.full(6, .2))
            with patch_hardware(self, {"can0": virtual}, {"right": arm}, modules=(guide_app,)):
                guide_app.guide(side="right", duration=.025, calibration=profile_path)
            frames = [frame for frame in virtual.sent if (frame.arbitration_id >> 24) & 31 == 1]
            self.assertTrue(frames)
            for frame in frames:
                motor = frame.arbitration_id & 255
                bound = robstride.LIMITS[arm.models[motor-1]].torque
                actual = ((frame.arbitration_id >> 8) & 65535) / 65535 * 2 * bound - bound
                self.assertAlmostEqual(actual, expected[motor-1], delta=.003)
            with self.assertRaisesRegex(ValueError, "different collection"):
                gravity_fit.validate(root / "data.jsonl", candidate_path)
            dataset(root / "independent.jsonl", scales, independent=True, friction=[0, .4, .35, .25, .15, .1])
            self.assertTrue(gravity_fit.validate(root / "independent.jsonl", candidate_path)["accepted"])
            duplicate = root / "duplicate.jsonl"
            duplicate.write_text((root / "data.jsonl").read_text() + "\n")
            with self.assertRaisesRegex(ValueError, "targets distinct"):
                gravity_fit.validate(duplicate, candidate_path)
            independent = root / "independent.jsonl"
            records = [json.loads(line) for line in independent.read_text().splitlines()]
            pairs = {p["id"]: p.get("pair_id") for p in records[0]["plan"]["poses"]}
            records = [r for r in records if r["type"] != "pose_complete" or pairs[r["pose_id"]] < 7]
            independent.write_text("\n".join(json.dumps(r) for r in records))
            with self.assertRaisesRegex(ValueError, "at least 8 complete"):
                gravity_fit.validate(independent, candidate_path)
            with self.assertRaisesRegex(ValueError, "do not match"):
                gravity.GravityModel(calibration=profile_path,
                                     binding=binding("left", meta["reference_rad"], meta["motor_uids"]))
            with self.assertRaisesRegex(ValueError, "do not match"):
                gravity.GravityModel(calibration=profile_path,
                                     binding=binding("right", .01, meta["motor_uids"]))
            other = {motor: uid[::-1] for motor, uid in meta["motor_uids"].items()}
            with self.assertRaisesRegex(ValueError, "do not match"):
                gravity.GravityModel(calibration=profile_path,
                                     binding=binding("right", meta["reference_rad"], other))
            with self.assertRaisesRegex(ValueError, "requires the selected arm binding"):
                gravity.GravityModel(calibration=profile_path)

    def test_poor_validation_only_warns_and_still_saves_profile(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            dataset(root / "data.jsonl", [1, 1.1, 1.2, 1.1, 1.1, 1.1], corrupt_validation=True)
            report = gravity_fit.fit(root / "data.jsonl", root / "fit")
            self.assertTrue(report["accepted"])
            self.assertFalse(report["reasons"])
            self.assertIn("J2", " ".join(report["warnings"]))
            self.assertTrue((root / "fit" / "report.json").exists())
            profile = yaml.safe_load((root / "fit" / "calibration.yaml").read_text())
            self.assertEqual(profile["warnings"], report["warnings"])

    def test_hold_gate_rejects_large_directional_corrections(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scales = np.array([1, 1.1, 1.35, 1.2, 1., 1.25])
            dataset(root / "fit.jsonl", scales)
            gravity_fit.fit(root / "fit.jsonl", root / "fit")
            candidate = yaml.safe_load((root / "fit" / "calibration.yaml").read_text())
            dataset(root / "before.jsonl", scales, independent=True, experiment="baseline")
            dataset(root / "after.jsonl", scales, independent=True, experiment="trial",
                    candidate=candidate, friction=[0, .5, 0, 0, 0, 0])
            result = gravity_trials.compare_holds(root / "before.jsonl", root / "after.jsonl", candidate)
            self.assertFalse(result["accepted"])
            self.assertIn("J2", " ".join(result["reasons"]))
            self.assertGreater(result["rmse_after_nm"][1], .49)

    def test_known_payload_delta_and_mismatched_sessions(self):
        payload = {"mass_kg": .12, "com_link6_m": [.02, 0., .08]}
        with tempfile.TemporaryDirectory() as directory:
            empty, loaded = (Path(directory) / name for name in ("empty", "loaded"))
            dataset(empty, np.ones(6), independent=True, experiment="payload_empty")
            dataset(loaded, np.ones(6), independent=True, experiment="payload_loaded", payload=payload)
            result = gravity_trials.compare_payload(empty, loaded, .12, [.02, 0., .08])
            self.assertEqual(result["pairs"], 12)
            np.testing.assert_allclose(result["reported"]["rmse_nm"], 0., atol=1e-8)
            self.assertAlmostEqual(result["estimated"]["gain"][1], 1.)
            self.assertIsNone(result["estimated"]["gain"][0])
            with self.assertRaisesRegex(ValueError, "known-load"):
                gravity_trials.compare_payload(empty, loaded, .24, [.02, 0., .08])
            lines = loaded.read_text().splitlines()
            meta = json.loads(lines[0])
            meta["reference_rad"][0] = .01
            loaded.write_text("\n".join([json.dumps(meta), *lines[1:]]))
            with self.assertRaisesRegex(ValueError, "do not match"):
                gravity_trials.compare_payload(empty, loaded, .12, [.02, 0., .08])

    def test_saturated_or_moving_frames_discard_their_whole_pair(self):
        for defect in ("saturated", "moving"):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "data.jsonl"
                metadata = dataset(path, np.ones(6))
                poses = {pose["id"]: pose for pose in metadata["plan"]["poses"]}
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                samples = [row for row in rows if row.get("type") == "sample"]
                if defect == "saturated":       # every frame of one approach direction
                    pair = 0
                    for row in samples:
                        pose = poses[row["pose_id"]]
                        if pose.get("pair_id") == 0 and pose.get("direction") == 1:
                            row["saturated"] = True
                else:                           # a single frame
                    pair = poses[samples[0]["pose_id"]]["pair_id"]
                    samples[0]["raw_velocity"][2] = .03
                path.write_text("\n".join(json.dumps(row) for row in rows))
                _, accepted = gravity_fit.load_dataset(path)
                self.assertEqual(len(accepted), 23)
                self.assertNotIn(pair, [row["pair_id"] for row in accepted])

    def test_pair_split_leakage_is_rejected(self):
        cfg = load_settings()
        plan = gravity_fit.make_plan([0, .7, 1.1, 0, 0, 0], cfg)
        samples = [p for p in plan["poses"] if p.get("pair_id") == 0]
        samples[1]["split"] = "validation"
        with self.assertRaisesRegex(ValueError, "one split"):
            gravity_fit.validate_plan(plan, gravity.GravityModel(), cfg.gravity_scale, cfg.torque_limit_nm)

    def test_uninformative_measurements_cannot_export_a_mass_profile(self):
        model = gravity.GravityModel()
        rows = []
        for i in range(24):
            q = np.array([i * .05, .7, 1.1, 0, 0, 0])
            rows.append({"q": q, "target": q, "basis": model.basis(q), "torque": model.gravity(q),
                         "axis": 1, "split": "validation" if i % 4 == 3 else "train"})
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(gravity_fit, "load_dataset", return_value=({}, rows)):
            root = Path(directory)
            report = gravity_fit.fit(root / "unused.jsonl", root / "fit")
            self.assertFalse(report["accepted"])
            self.assertTrue(report["rank"] < 4 or report["condition"] > 1e4)
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
            _, pairs = gravity_fit.load_dataset(path)
            self.assertEqual(len(pairs), 24)
            for pair in pairs:
                np.testing.assert_allclose(pair["basis"] @ np.ones(6), pair["torque"], atol=1e-10)


class WorkflowTests(unittest.TestCase):
    start = [0, .7, 1.1, 0, 0, 0]

    def setUp(self):
        item = patch.object(identify, "preflight")
        self.preflight = item.start()
        self.addCleanup(item.stop)
        scene = patch.object(identify, "workstation_check")
        scene.start()
        self.addCleanup(scene.stop)

    def fake_collect(self, plan, output, **options):
        dataset(Path(output), [1, 1.1, 1.35, 1.2, 1.15, 1.25],
                independent=Path(plan).name == "validation_plan.yaml",
                experiment=options.get("experiment", "identification"), candidate=options.get("candidate"),
                speed=options["speed"])
        return "completed"

    def test_default_run_samples_once_and_saves_profile(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect", side_effect=self.fake_collect) as collect:
            root = Path(directory) / "run"
            result = identify.run_calibration(root, start=self.start)
            self.assertTrue(result["accepted"])
            collect.assert_called_once()
            self.assertEqual(collect.call_args.kwargs["speed"], .8)
            self.assertTrue((root / "calibrated.urdf").exists())
            profile = yaml.safe_load((root / "calibration.yaml").read_text())
            self.assertFalse(profile["verified"])
            self.assertNotIn("hold_comparison", profile)

    def test_verify_runs_matched_holds_and_marks_profile(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect", side_effect=self.fake_collect) as collect:
            root = Path(directory) / "run"
            result = identify.run_calibration(root, start=self.start, verify=True)
            self.assertTrue(result["accepted"])
            self.assertEqual(collect.call_count, 3)
            self.assertTrue((root / "calibration.yaml").exists())
            self.assertTrue((root / "calibrated.urdf").exists())
            self.assertTrue((root / "report.json").exists())
            profile = yaml.safe_load((root / "calibration.yaml").read_text())
            self.assertTrue(profile["verified"])
            self.assertTrue(profile["independent_validation"]["accepted"])
            self.assertTrue(profile["hold_comparison"]["accepted"])
            with self.assertRaisesRegex(ValueError, "already exists"):
                identify.run_calibration(root, start=self.start, verify=True)
            self.assertEqual(collect.call_count, 3)
            self.preflight.assert_called_once()

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

    def test_rejected_fit_exports_nothing_but_failed_checks_keep_profile(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "collect", side_effect=self.fake_collect) as collect, \
                patch.object(identify, "fit", return_value={"accepted": False, "reason": "failure"}):
            root = Path(directory) / "run"
            result = identify.run_calibration(root, start=self.start, verify=True)
            self.assertEqual(result["status"], "rejected")
            # A rejected fit never starts the independent round.
            collect.assert_called_once()
            self.assertFalse((root / "calibration.yaml").exists())
        for stage in ("validate", "compare_holds"):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as directory, \
                    redirect_stdout(io.StringIO()), \
                    patch.object(identify, "collect", side_effect=self.fake_collect) as collect, \
                    patch.object(identify, stage, return_value={"accepted": False, "reasons": ["failure"]}):
                root = Path(directory) / "run"
                result = identify.run_calibration(root, start=self.start, verify=True)
                self.assertTrue(result["accepted"])
                self.assertEqual(collect.call_count, 3)
                profile = yaml.safe_load((root / "calibration.yaml").read_text())
                self.assertFalse(profile["verified"])
                self.assertTrue(any("failure" in w for w in profile["warnings"]))

    def test_payload_cli_matched_rounds_and_arm_selection(self):
        payload = {"mass_kg": .1, "com_link6_m": [0., 0., .08]}

        def collect_payload(plan, output, **options):
            self.assertEqual(options["side"], "left")
            if options["experiment"] == "payload_loaded":
                self.assertEqual(options["payload"], payload)
            planned = yaml.safe_load(Path(plan).read_text())
            self.assertEqual(len([p for p in planned["poses"] if p["stage"] == "sample"]), 24)
            dataset(Path(output), np.ones(6), independent=True, side=options["side"],
                    experiment=options["experiment"], payload=options["payload"], speed=options["speed"])
            return "completed"

        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch.object(identify, "read_current_pose", return_value=np.array(self.start)), \
                patch.object(identify, "select_arms", return_value={"left": binding("left")}), \
                patch.object(identify, "collect", side_effect=collect_payload) as collect:
            empty, loaded = [str(Path(directory) / d) for d in ("empty", "loaded")]
            self.assertEqual(identify.main(["--side", "left", "payload-empty", "--output", empty]), 0)
            args = ["payload-loaded", "--empty", empty, "--output", loaded,
                    "--mass-kg", ".1", "--com-m", "0", "0", ".08"]
            with self.assertRaisesRegex(ValueError, "same arm"):
                identify.main(args)
            self.assertEqual(collect.call_count, 1)
            args += ["--side", "left"]
            self.assertEqual(identify.main(args), 0)
            self.assertTrue((Path(loaded) / "payload_report.json").is_file())
            args[args.index(loaded)] = loaded + "-overload"
            args[args.index(".1")] = "100"
            with self.assertRaisesRegex(ValueError, "torque budget"):
                identify.main(args)
            self.assertEqual(collect.call_count, 2)

    def test_existing_directory_is_rejected_before_motor_access(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(identify, "read_current_pose") as read, patch.object(identify, "collect") as collect:
            with self.assertRaisesRegex(ValueError, "already exists"):
                identify.run_calibration(directory)
            read.assert_not_called()
            collect.assert_not_called()
            self.preflight.assert_not_called()


class CollectTests(unittest.TestCase):
    def setup_bus(self):
        self.virtual = VirtualMotors()
        self.arm = binding("right", np.full(6, .2) - gravity_fit.clearance_pose(0), dict(self.virtual.uids))
        patch_hardware(self, {"can0": self.virtual}, {"right": self.arm}, modules=(identify,))
        configured = patch.object(identify, "load_arms", return_value={"right": {}})
        configured.start()
        self.addCleanup(configured.stop)
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
        goal = np.array(goal if goal is not None else gravity_fit.clearance_pose(0))
        poses = []
        for direction in (1, -1):
            poses.append({"id": len(poses), "stage": "transit", "q": (goal - direction * np.array([0, 0, 0, 0, gravity_fit.APPROACH_RAD, 0])).tolist()})
            poses.append({"id": len(poses), "stage": "sample", "q": goal.tolist(),
                          "pair_id": 0, "axis": 5, "direction": direction, "split": "train"})
        gravity_fit.write_yaml(path, {"version": 2, "method": "paired_static", "units": "rad",
                                  "model_sha256": gravity.GravityModel().sha256,
                                  "start": gravity_fit.clearance_pose(0).tolist(), "poses": poses})

    def assert_holding(self):
        self.assertTrue(self.virtual.closed)
        self.assertEqual(set(self.virtual.modes.values()), {2})
        self.assertFalse(any((m.arbitration_id >> 24) & 31 == 4 for m in self.virtual.sent))
        frames = [m for m in self.virtual.sent if (m.arbitration_id >> 24) & 31 == 1]
        self.assertTrue(all(int.from_bytes(m.data[4:6], "big") > 0 for m in frames[-6:]))
        self.assertTrue(all((m.arbitration_id & 255) <= 6 for m in frames))

    def test_preflight_and_initial_pose_read_never_write_to_the_motors(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.plan(root / "plan.yaml")
            identify.preflight([root / "plan.yaml"])
            self.assertFalse((root / "data.jsonl").exists())
        preflight = len(self.virtual.sent)
        self.assertTrue(preflight)
        np.testing.assert_allclose(identify.read_current_pose(ARM_BINDINGS, "right"),
                                   gravity_fit.clearance_pose(0), atol=1e-6)
        self.assertGreater(len(self.virtual.sent), preflight)
        self.assertTrue(all((m.arbitration_id >> 24) & 31 in (0, 17) for m in self.virtual.sent))

    def test_clear_endpoints_do_not_hide_mast_collision_in_transit(self):
        collision = MotionCollision("right", np.zeros(7))
        collision.check_pose(np.zeros(6), 0)  # Near-touching folded meshes.
        start = np.array([np.radians(-60), 1.25, 1.7, -.35, .45, 0.])
        goal = start.copy()
        goal[0] = np.radians(-105)
        collision.check_pose(start, 1)
        collision.check_pose(goal, 2)
        with self.assertRaisesRegex(ValueError, "waypoint 2: collision .*mast"):
            collision.check_segment(start, goal, 2)

    def test_parked_arm_position_changes_collision_result(self):
        q = [-np.pi/4, 1.53, .8, -.35, -.45, 0.]
        MotionCollision("right", np.zeros(7)).check_pose(q, 1)
        parked = [np.pi/4, 1.15, .57, .35, -.45, 0., 0.]
        with self.assertRaisesRegex(ValueError, "right_link4.*left_gripper"):
            MotionCollision("right", parked).check_pose(q, 1)

    def test_bound_parked_arm_is_read_without_motor_commands(self):
        virtual = VirtualMotors(offset=20)
        arm = binding("left", .05, virtual.uids)
        with patch_hardware(self, {"can0": virtual}, {"left": arm, "right": binding()}, modules=(identify,)), \
                patch.object(identify, "load_arms", return_value={"left": {}, "right": {}}):
            collision = identify.workstation_check(ARM_BINDINGS, "right", robstride.MotorBus)
        self.assertAlmostEqual(collision.data.joint("left_joint6").qpos[0], .15)
        self.assertGreater(collision.data.joint("left_joint_left").qpos[0], 0.)
        self.assertTrue(virtual.closed)
        self.assertTrue(all((m.arbitration_id >> 24) & 31 in (0, 17) for m in virtual.sent))

    def test_inward_sweep_is_rejected_before_motor_setup_or_enable(self):
        self.setup_bus()
        start = np.zeros(6)
        start[0] = np.radians(-75)
        self.arm.reference[:6] = .2 - start
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = root / "plan.yaml"
            # A previously saved trajectory can still have the old inward working yaw.
            old = gravity_fit.make_plan(np.zeros(6), load_settings())
            for q in [old["start"], old["clearance"], *[p["q"] for p in old["poses"]]]:
                q[0] = start[0]
            gravity_fit.write_yaml(plan, old)
            with self.assertRaisesRegex(ValueError, "collision .*(scene_camera_housing|mast)"):
                identify.preflight([plan])
            with self.assertRaisesRegex(ValueError, "collision .*(scene_camera_housing|mast)"):
                identify.collect(plan, root / "data.jsonl")
        self.assertTrue(all((m.arbitration_id >> 24) & 31 in (0, 17) for m in self.virtual.sent))

    def test_inward_starts_join_the_same_fixed_workstation_paths(self):
        cfg = load_settings()
        for independent in (False, True):
            fixed = gravity_fit.make_plan(gravity_fit.CLEARANCE, cfg, independent)["poses"]
            for side, yaw in (("left", 75), ("right", -75)):
                with self.subTest(independent=independent, side=side):
                    start = np.zeros(6)
                    start[0] = np.radians(yaw)
                    scene = MotionCollision(side, np.zeros(7))
                    plan = gravity_fit.make_plan(start, cfg, independent)
                    entry = [p for p in plan["poses"] if p["stage"] == "entry"]
                    np.testing.assert_allclose(entry[-1]["q"], gravity_fit.CLEARANCE)
                    body = [p for p in plan["poses"] if p["stage"] != "entry"]
                    self.assertEqual([dict(p, id=i) for i, p in enumerate(body)], fixed)
                    scene.check_plan(start, plan["poses"])

    def test_rest_pose_slightly_below_hard_stop_plans_inward_only(self):
        # Measured right-arm rest pose: J2 sits 1.79 deg below its zero limit.
        start = np.radians([3.88, -1.79, .07, -.78, -.02, -.21])
        cfg, model = load_settings(), gravity.GravityModel()
        plan = gravity_fit.make_plan(start, cfg)
        np.testing.assert_allclose(plan["poses"][-1]["q"], gravity_fit.CLEARANCE)
        # Every waypoint is inside the limits, so a later, slightly different reading of
        # the same rest pose (preflight, collection) can still reach the first one.
        self.assertTrue(all(not model.limit_excess(np.array(p["q"])).any() for p in plan["poses"]))
        first = np.array(plan["poses"][0]["q"])
        for noise in (-2e-3, -4e-5, 4e-5, 2e-3):
            reread = start.copy()
            reread[1] += noise
            gravity_fit.path_check(model, reread, first, cfg.gravity_scale, cfg.torque_limit_nm)
        further = start.copy()
        further[1] -= .01
        with self.assertRaisesRegex(ValueError, "exceeds joint limits"):
            gravity_fit.path_check(model, start, further, cfg.gravity_scale, cfg.torque_limit_nm)
        inside = np.zeros(6)
        inside[1] = .2
        outside = inside.copy()
        outside[1] = -.01
        with self.assertRaisesRegex(ValueError, "exceeds joint limits"):
            gravity_fit.path_check(model, inside, outside, cfg.gravity_scale, cfg.torque_limit_nm)
        beyond = start.copy()
        beyond[1] = -.15
        with self.assertRaisesRegex(ValueError, r"J2: -8\.59 deg.*tolerance=5\.73 deg"):
            model.validate(beyond)

    def test_preflight_and_collection_only_collision_check_sweep_segments(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            path = root / "plan.yaml"
            self.plan(path)
            plan = yaml.safe_load(path.read_text())
            entry = [-.01, .7, 1.1, 0., 0., 0.]
            plan["poses"][:0] = [{"id": -2, "stage": "entry", "q": plan["start"]},
                                 {"id": -1, "stage": "entry", "q": entry}]
            path.write_text(yaml.safe_dump(plan))
            scene = MotionCollision("right", None)
            with patch.object(identify, "workstation_check", return_value=scene), \
                    patch.object(scene, "check_segment", wraps=scene.check_segment) as check:
                identify.preflight([path])
                self.assertEqual(identify.collect(path, root / "data.jsonl"), "completed")
            sweep = plan["poses"][2:]
            self.assertEqual([c.args[2] for c in check.call_args_list], [p["id"] for p in sweep] * 2)
            for call in (check.call_args_list[0], check.call_args_list[len(sweep)]):
                np.testing.assert_allclose(call.args[0], entry)

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
            _, poses = gravity_fit.load_dataset(root / "data.jsonl")
            self.assertEqual(len(poses), 1)
        self.assert_holding()

    def test_next_pose_starts_at_previous_target_instead_of_feedback(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            poses = []
            for pair_id, angle in enumerate((.008, .012)):
                goal = gravity_fit.clearance_pose(0) + [0, 0, 0, 0, angle, 0]
                for direction in (1, -1):
                    poses.append({"id": len(poses), "stage": "transit", "q": (goal - direction * np.array([0, 0, 0, 0, gravity_fit.APPROACH_RAD, 0])).tolist()})
                    poses.append({"id": len(poses), "stage": "sample", "q": goal.tolist(),
                                  "pair_id": pair_id, "axis": 5, "direction": direction, "split": "train"})
            plan = {"version": 2, "method": "paired_static", "units": "rad",
                    "model_sha256": gravity.GravityModel().sha256, "start": gravity_fit.clearance_pose(0).tolist(), "poses": poses}
            gravity_fit.write_yaml(root / "plan.yaml", plan)
            # At 0.2 rad/s and 50 Hz, a continuous command advances at most 0.004 rad per tick.
            identify.collect(root / "plan.yaml", root / "data.jsonl", speed=.2, settle_s=.5, sample_s=.5)
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

    def test_fault_during_sampling_still_attempts_native_hold(self):
        self.setup_bus()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                redirect_stderr(io.StringIO()):
            root = Path(directory)
            self.plan(root / "plan.yaml")
            with patch.object(identify.TrajectoryTracker, "update", side_effect=RuntimeError("injected fault")):
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
