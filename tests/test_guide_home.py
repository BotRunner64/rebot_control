"""Simulated CAN lifecycle checks for hand guiding and homing; only home may disable."""

from contextlib import redirect_stderr, redirect_stdout
import ast
import io
import json
import math
from pathlib import Path
import signal
import struct
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

import rebot
from rebot.apps import guide as guide_app
from rebot.apps import home as home_app
from rebot.devices import robstride
from rebot.model import trajectory
from rebot.model.gravity import GravityModel
from rebot.model.guiding import HandGuidingController
from rebot.settings import load_settings
from fakes import VirtualMotors, arm_bindings, patch_hardware


def kind_of(msg):
    return (msg.arbitration_id >> 24) & 31


class LifecycleTests(unittest.TestCase):
    def setup_run(self, fail_right=False):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        # MotionLog() writes below RUNS/motion; keep it out of the repository.
        self.journal_dir = Path(directory.name) / "motion"
        self.virtual = {"can0": VirtualMotors(), "can1": VirtualMotors(100, 3 if fail_right else None)}
        self.arms = arm_bindings({"left": ("can0", self.virtual["can0"]),
                                  "right": ("can1", self.virtual["can1"])})
        patch_hardware(self, self.virtual, self.arms, modules=(guide_app, home_app))
        item = patch.object(trajectory, "RUNS", Path(directory.name))
        item.start()
        self.addCleanup(item.stop)

    def test_partial_dual_enable_failure_keeps_enabled_motors_held(self):
        self.setup_run(fail_right=True)
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(TimeoutError):
                guide_app.guide(side="both", duration=.02)
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertFalse(any(kind_of(m) == 4 for m in bus.sent))
        self.assertEqual(set(self.virtual["can0"].modes.values()), {2})
        self.assertEqual(self.virtual["can1"].modes[1], 2)
        self.assertEqual(self.virtual["can1"].modes[3], 0)

    def test_duration_exit_leaves_native_hold_on_both_arms(self):
        self.setup_run()
        with redirect_stdout(io.StringIO()):
            guide_app.guide(side="both", duration=.025)
        for bus in self.virtual.values():
            commands = [m for m in bus.sent if kind_of(m) == 1]
            self.assertGreater(len(commands), 6)
            for msg in commands:
                q_code, _, kp, kd = struct.unpack(">HHHH", msg.data)
                decoded_q = q_code / 65535 * (8 * math.pi) - 4 * math.pi
                self.assertAlmostEqual(decoded_q, .2, delta=.001)
            self.assertTrue(all(struct.unpack(">HHHH", m.data)[2] > 0 for m in commands[-6:]))
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertFalse(any(kind_of(m) == 4 for m in bus.sent))

    def test_stale_pair_is_rejected_before_sending_control_torque(self):
        self.setup_run()
        original = HandGuidingController.update

        def delayed_update(controller, *args):
            result = original(controller, *args)
            time.sleep(.045)
            return result

        with patch.object(HandGuidingController, "update", delayed_update), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "too old"):
                guide_app.guide(side="both", duration=.025)
        for bus in self.virtual.values():
            kinds = [kind_of(m) for m in bus.sent]
            last_enable = max(i for i, kind in enumerate(kinds) if kind == 3)
            after = bus.sent[last_enable + 1:]
            controls = [m for m in after if kind_of(m) == 1]
            self.assertTrue(all(struct.unpack(">HHHH", m.data)[2] > 0 for m in controls))
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertNotIn(4, kinds)

    def test_ctrl_c_exits_to_hold_without_disable(self):
        self.setup_run()
        original = HandGuidingController.update

        def request_stop(controller, *args):
            result = original(controller, *args)
            signal.raise_signal(signal.SIGINT)
            return result

        previous = signal.getsignal(signal.SIGINT)
        with patch.object(HandGuidingController, "update", request_stop), redirect_stdout(io.StringIO()):
            guide_app.guide(side="both")
        self.assertIs(signal.getsignal(signal.SIGINT), previous)
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertFalse(any(kind_of(m) == 4 for m in bus.sent))

    def test_second_run_keeps_gravity_support_and_does_not_reenable_running_arm(self):
        self.setup_run()
        with redirect_stdout(io.StringIO()):
            guide_app.guide(side="right", duration=.025)
        bus = self.virtual["can1"]
        bus.sent.clear()
        with redirect_stdout(io.StringIO()):
            guide_app.guide(side="right", duration=.025)
        kinds = [kind_of(m) for m in bus.sent]
        self.assertNotIn(3, kinds)
        self.assertNotIn(4, kinds)
        expected = GravityModel().gravity(np.full(6, .2)) * load_settings().gravity_scale
        frames = [m for m in bus.sent if kind_of(m) == 1]
        self.assertTrue(frames)
        for msg in frames:
            motor = msg.arbitration_id & 255
            model = self.arms["right"].models[motor - 1]
            bound = robstride.LIMITS[model].torque
            torque = ((msg.arbitration_id >> 8) & 65535) / 65535 * 2 * bound - bound
            self.assertAlmostEqual(torque, expected[motor - 1], delta=.003)

    def test_space_stop_does_not_disable(self):
        self.setup_run()
        keys = MagicMock()
        keys.__enter__.return_value.pressed.return_value = True
        with patch.object(guide_app, "StopKey", return_value=keys), redirect_stdout(io.StringIO()):
            guide_app.guide(side="both")
        for bus in self.virtual.values():
            self.assertEqual(set(bus.modes.values()), {2})
            self.assertFalse(any(kind_of(m) == 4 for m in bus.sent))

    def fake_clock(self, stop_after):
        """Patch the clock used by the home loop; the stop key fires at `stop_after` seconds."""
        clock = [0.]

        def now():
            clock[0] += .00001
            return clock[0]

        def sleep(duration):
            clock[0] += duration

        keys = MagicMock()
        keys.__enter__.return_value.pressed.side_effect = lambda: stop_after(clock[0])
        for item in (patch.object(home_app, "StopKey", return_value=keys),
                     patch.object(home_app.time, "monotonic", now),
                     patch.object(home_app.time, "sleep", sleep)):
            item.start()
            self.addCleanup(item.stop)
        return clock

    def run_completed_home(self, side="both", residual=0.):
        for bus in self.virtual.values():
            bus.position = residual
        self.fake_clock(lambda elapsed: elapsed >= 5)
        with redirect_stdout(io.StringIO()):
            return home_app.home(side=side)

    def test_manual_home_completion_disables_all_seven_motors_on_both_arms(self):
        self.setup_run()
        self.assertTrue(self.run_completed_home())
        journals = list(self.journal_dir.glob("home-*.jsonl"))
        self.assertEqual(len(journals), 1)
        rows = [json.loads(line) for line in journals[0].read_text().splitlines()]
        self.assertEqual(rows[0]["control"], "native_mit")
        self.assertEqual(rows[0]["speed"], .3)
        samples = [row for row in rows if row["type"] == "sample"]
        self.assertEqual({row["side"] for row in samples}, {"left", "right"})
        self.assertTrue(all("native_parameters" in row and "cycle_s" in row for row in samples))
        for bus in self.virtual.values():
            self.assertTrue(bus.closed)
            self.assertEqual(set(bus.modes.values()), {0})
            self.assertEqual({m.arbitration_id & 255 for m in bus.sent if kind_of(m) == 4},
                             set(range(1, 8)))
            kinds = [kind_of(m) for m in bus.sent]
            first_disable = kinds.index(4)
            self.assertNotIn(1, kinds[first_disable:])
            self.assertNotIn(3, kinds[first_disable:])

    def test_single_arm_home_does_not_disable_the_other_arm(self):
        self.setup_run()
        self.assertTrue(self.run_completed_home(side="right"))
        self.assertEqual(set(self.virtual["can1"].modes.values()), {0})
        self.assertFalse(self.virtual["can0"].sent)

    def test_home_residual_decides_whether_the_motors_are_disabled(self):
        for residual, reached in ((.04, True), (.06, False)):
            with self.subTest(residual=residual):
                self.setup_run()
                self.assertEqual(self.run_completed_home(side="right", residual=residual), reached)
                bus = self.virtual["can1"]
                self.assertEqual(set(bus.modes.values()), {0} if reached else {2})
                self.assertEqual({m.arbitration_id & 255 for m in bus.sent if kind_of(m) == 4},
                                 set(range(1, 8)) if reached else set())

    def test_home_disable_failure_is_reported_and_other_motors_are_attempted(self):
        self.setup_run()
        self.virtual["can0"].drop_stops.add(3)
        with redirect_stderr(io.StringIO()), self.assertRaisesRegex(RuntimeError, "not confirmed"):
            self.run_completed_home()
        self.assertEqual(self.virtual["can0"].modes[3], 2)
        self.assertEqual(self.virtual["can0"].modes[7], 0)
        self.assertEqual(set(self.virtual["can1"].modes.values()), {0})

    def test_home_signal_or_motion_error_skips_disable(self):
        for failure in ("signal", "error"):
            with self.subTest(failure=failure):
                self.setup_run()
                original = home_app.TrajectoryTracker.update

                def update(controller, *args):
                    if failure == "error":
                        raise RuntimeError("Motion failed.")
                    record = original(controller, *args)
                    signal.raise_signal(signal.SIGINT)
                    return record

                with patch.object(home_app.TrajectoryTracker, "update", update), \
                        patch.object(home_app, "disable") as disable, redirect_stdout(io.StringIO()):
                    if failure == "error":
                        with self.assertRaisesRegex(RuntimeError, "Motion failed"):
                            home_app.home(side="right")
                    else:
                        self.assertFalse(home_app.home(side="right"))
                    disable.assert_not_called()
                bus = self.virtual["can1"]
                self.assertTrue(bus.closed)
                self.assertEqual(set(bus.modes.values()), {2})
                self.assertFalse(any(kind_of(m) == 4 for m in bus.sent))

    def test_home_tracking_lag_continues_until_manual_stop_and_keeps_enabled(self):
        self.setup_run()
        clock = self.fake_clock(lambda elapsed: elapsed >= 12)
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertFalse(home_app.home(side="right"))
        self.assertGreaterEqual(clock[0], 12)
        self.assertNotIn("Home reached", output.getvalue())
        bus = self.virtual["can1"]
        frames = [msg for msg in bus.sent if kind_of(msg) == 1]
        decoded = [robstride.decode_mit(msg, self.arms["right"].models[(msg.arbitration_id & 255) - 1])
                   for msg in frames]
        self.assertTrue(all(item["kp"] > 0 and item["kd"] > 0 for item in decoded))
        self.assertTrue(any(item["velocity"] < -.005 for item in decoded))
        self.assertTrue(bus.closed)
        self.assertEqual(set(bus.modes.values()), {2})
        self.assertFalse(any(kind_of(m) == 4 for m in bus.sent))

    def test_home_starts_from_enabled_target_without_feedback_reset(self):
        self.setup_run()
        reads, targets = [0], []
        original = robstride.MotorBus.command_position

        def positions(bus):
            reads[0] += 1
            return [.2 if reads[0] == 1 else .23 if reads[0] == 2 else .24] * 6

        def command(bus, positions, *args, **kwargs):
            targets.append(np.array(positions))
            return original(bus, positions, *args, **kwargs)

        self.fake_clock(lambda elapsed: elapsed > .2)
        with patch.object(robstride.MotorBus, "positions", positions), \
                patch.object(robstride.MotorBus, "command_position", command), redirect_stdout(io.StringIO()):
            home_app.home(side="right")
        self.assertTrue(targets)
        np.testing.assert_allclose(targets[0], [.23] * 6, atol=.0001)


PACKAGE = Path(rebot.__file__).parent
HOME = PACKAGE / "apps" / "home.py"
# Browser servers and camera monitors have their own unrelated stop(); they never own CAN.
NON_CONTROL = ("viewer", "media", "data")


def is_disable(node):
    """The DISABLE communication type, by name or as the literal 4."""
    return ((isinstance(node, ast.Constant) and type(node.value) is int and node.value == 4)
            or (isinstance(node, ast.Name) and node.id == "DISABLE")
            or (isinstance(node, ast.Attribute) and node.attr == "DISABLE"))


def disable_uses(source, control=True):
    """Every construct in `source` that could disable motors or reach the home command."""
    found = []
    for node in ast.walk(ast.parse(source)):
        line = getattr(node, "lineno", 0)
        if isinstance(node, ast.Import):
            found += [f"{line}: import {n.name}" for n in node.names
                      if n.name in ("rebot.apps.home", "rebot_home")]
        elif isinstance(node, ast.ImportFrom):
            module, names = node.module or "", [n.name for n in node.names]
            if (module.split(".")[-1] in ("home", "rebot_home") or "DISABLE" in names
                    or (module.split(".")[-1] in ("apps", "") and "home" in names)):
                found.append(f"{line}: from {'.' * node.level}{module} import {', '.join(names)}")
        elif isinstance(node, (ast.Name, ast.Attribute)):
            if is_disable(node) and isinstance(node.ctx, ast.Load):
                found.append(f"{line}: reads DISABLE")
        elif isinstance(node, ast.BinOp):
            if (isinstance(node.op, ast.LShift) and is_disable(node.left)
                    and isinstance(node.right, ast.Constant) and node.right.value == 24):
                found.append(f"{line}: builds a type-4 arbitration ID")
        elif isinstance(node, ast.Call):
            name = (node.func.attr if isinstance(node.func, ast.Attribute)
                    else node.func.id if isinstance(node.func, ast.Name) else None)
            forbidden = ("disable", "disable_all") + (("stop",) if control else ())
            if name in forbidden and (isinstance(node.func, ast.Attribute) or name != "stop"):
                found.append(f"{line}: calls {name}()")
            arguments = node.args[:1] + [k.value for k in node.keywords if k.arg == "kind"]
            if name == "send" and any(is_disable(argument) for argument in arguments):
                found.append(f"{line}: sends type 4")
    return found


class DisableIsolationTests(unittest.TestCase):
    def test_control_modules_cannot_call_disable_or_send_type_four(self):
        modules = sorted(PACKAGE.rglob("*.py"))
        # The modules the original check named must still be covered.
        for required in ("apps/guide.py", "devices/robstride.py", "runtime/arm.py", "runtime/worker.py",
                         "runtime/robot.py", "model/guiding.py", "apps/identify.py", "apps/teleop.py",
                         "apps/ee.py", "apps/policy.py", "cli.py"):
            self.assertIn(PACKAGE / required, modules)
        self.assertIn(HOME, modules)
        for path in modules:
            if path == HOME:
                continue
            relative = path.relative_to(PACKAGE)
            with self.subTest(module=str(relative)):
                self.assertEqual(disable_uses(path.read_text(), relative.parts[0] not in NON_CONTROL), [])

    def test_home_is_the_one_module_that_disables(self):
        uses = disable_uses(HOME.read_text())
        self.assertTrue(any("sends type 4" in use for use in uses), uses)
        self.assertFalse(any("stop()" in use for use in uses), uses)

    def test_detector_recognizes_the_constant_and_the_literal(self):
        for source in ("bus.send(DISABLE, motor)", "bus.send(robstride.DISABLE, motor)",
                       "bus.send(4, motor)", "bus.send(kind=4, motor_id=1)", "self.send(4, 1)",
                       "arb = (4 << 24) | motor", "arb = (DISABLE << 24) | motor",
                       "bus.disable()", "disable(config, side)", "bus.disable_all()", "motor.stop()",
                       "kind = DISABLE", "kind = robstride.DISABLE",
                       "from rebot.devices.robstride import DISABLE",
                       "import rebot.apps.home", "from rebot.apps import home",
                       "from rebot.apps.home import disable", "from . import home",
                       "from .home import disable"):
            with self.subTest(source=source):
                self.assertTrue(disable_uses(source))
        for source in ("bus.send(ENABLE, motor)", "bus.send(3, motor)", "value = 4",
                       "GET_DEVICE_ID, MIT_COMMAND, STATUS, ENABLE, DISABLE = 0, 1, 2, 3, 4",
                       "arb = (kind << 24) | motor", "stop.set()", "status.disable",
                       "from rebot.apps import guide", "bus.send(STATUS, 4)"):
            with self.subTest(source=source):
                self.assertEqual(disable_uses(source), [])
        self.assertEqual(disable_uses("server.stop()", control=False), [])


if __name__ == "__main__":
    unittest.main()
