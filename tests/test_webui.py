"""Safety behavior tests: no test opens a real CAN socket."""
import http.client
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest

from rebot_web.controller import Controller, load_config
from rebot_web.protocol import SimulatedCAN, SocketCAN, mit_payload
from rebot_web.server import make_server


CLIENT = "test-browser-session"


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.config = load_config()
        self.created = []
        def factory(config):
            bus = SimulatedCAN(config)
            bus.positions[1] = math.radians(35)
            self.created.append(bus)
            return bus
        self.controller = Controller(self.config, transport_factory=factory, start_worker=False,
                                     audit_path=Path(self.temp.name) / "audit.jsonl")

    def tearDown(self):
        self.controller.close()
        self.temp.cleanup()

    def connect(self):
        self.controller.connect(CLIENT)
        return self.created[0]

    def enable(self):
        bus = self.connect()
        self.controller.enable(CLIENT, 1)
        return bus

    def test_start_and_status_do_not_create_transport(self):
        for _ in range(10):
            self.controller.snapshot(CLIENT)
            self.controller.tick(0.01)
        self.assertEqual(self.created, [])

    def test_connect_is_queries_only_and_no_motor_enabled(self):
        bus = self.connect()
        self.assertTrue(all(frame[0] in (0, 17) for frame in bus.sent))
        self.assertFalse(any(j["enabled"] for j in self.controller.joints.values()))

    def test_motor_identity_mismatch_cannot_connect(self):
        self.controller.factory = lambda config: SimulatedCAN({**config, "joints": [{**j, "identity": "bad"} for j in config["joints"]]})
        with self.assertRaisesRegex(ValueError, "身份"):
            self.connect()
        self.assertIsNone(self.controller.transport)

    def test_enable_holds_fresh_measured_angle_and_never_zero(self):
        bus = self.enable()
        holds = [f for f in bus.sent if f[0] == 1]
        self.assertTrue(holds)
        self.assertTrue(all(abs(f[2] - math.radians(35)) < 1e-9 for f in holds))
        self.assertTrue(all(f[1] == 1 for f in bus.sent if f[0] in (1, 3, 4)))
        self.assertFalse(any(f[0] in (6, 7, 22) for f in bus.sent))

    def test_goal_requires_enable(self):
        self.connect()
        with self.assertRaisesRegex(ValueError, "使能"):
            self.controller.execute(CLIENT, 1, 36, 3)

    def test_limits_nan_and_step_are_enforced_before_motion(self):
        bus = self.enable()
        count = len(bus.sent)
        for angle, speed in [(151, 3), (float("nan"), 3), (float("inf"), 3), (True, 3), (36, self.config["max_speed_deg_s"] + 1), (36, 0), (55, 3)]:
            with self.assertRaises(ValueError):
                self.controller.execute(CLIENT, 1, angle, speed)
        self.assertEqual(count, len(bus.sent))

    def test_only_execute_changes_goal_and_velocity_is_bounded(self):
        bus = self.enable()
        before = self.controller.joints[1]["sent"]
        self.controller.execute(CLIENT, 1, 40, 3)
        self.assertEqual(self.controller.joints[1]["sent"], before)
        for _ in range(80):
            old = self.controller.joints[1]["sent"]
            self.controller.tick(.01)
            new = self.controller.joints[1]["sent"]
            self.assertLessEqual(abs(math.degrees(new - old)), .030001)
        self.assertGreater(bus.positions[1], before)

    def test_stale_browser_freezes_goal_but_does_not_disable(self):
        bus = self.enable()
        self.controller.execute(CLIENT, 1, 40, 3)
        self.controller.tick(.01)
        before = self.controller.joints[1]["sent"]
        self.controller.last_heartbeat = time.monotonic() - 3
        count = len(bus.sent)
        self.controller.tick(.01)
        self.assertEqual(self.controller.joints[1]["sent"], before)
        self.assertEqual(self.controller.joints[1]["target"], before)
        self.assertTrue(self.controller.joints[1]["enabled"])
        self.assertTrue(all(f[0] != 4 for f in bus.sent[count:]))

    def test_fault_and_stale_feedback_stop_target_progress(self):
        bus = self.enable()
        self.controller.execute(CLIENT, 1, 40, 3)
        bus.stale = True
        before = self.controller.joints[1]["sent"]
        self.controller.tick(.01)
        self.assertEqual(self.controller.joints[1]["sent"], before)
        self.assertIn("反馈超时", self.controller.joints[1]["fault"])
        with self.assertRaises(ValueError):
            self.controller.execute(CLIENT, 1, 36, 3)

    def test_stop_holds_last_command_and_does_not_home(self):
        bus = self.enable()
        self.controller.execute(CLIENT, 1, 40, 3)
        self.controller.tick(.01)
        angle = self.controller.joints[1]["sent"]
        self.controller.stop(CLIENT)
        self.controller.tick(.01)
        self.assertEqual(self.controller.joints[1]["sent"], angle)
        self.assertGreater(math.degrees(angle), 34)
        self.assertTrue(bus.active[1])

    def test_one_moving_joint_at_a_time(self):
        self.enable()
        self.controller.enable(CLIENT, 2)
        self.controller.execute(CLIENT, 1, 40, 3)
        with self.assertRaisesRegex(ValueError, "另一个关节"):
            self.controller.execute(CLIENT, 2, 5, 3)

    def test_control_ownership_and_disconnect_require_disable(self):
        self.enable()
        with self.assertRaises(ValueError):
            self.controller.execute("another-browser", 1, 36, 3)
        with self.assertRaises(ValueError):
            self.controller.disconnect(CLIENT)
        self.controller.disable(CLIENT)
        self.controller.disconnect(CLIENT)
        self.assertIsNone(self.controller.transport)

    def test_close_does_not_emit_motion_disable_or_home(self):
        bus = self.enable()
        before = len(bus.sent)
        self.controller.close()
        self.assertEqual(len(bus.sent), before)

    def test_stale_owner_can_be_explicitly_taken_over_without_movement(self):
        bus = self.enable()
        self.controller.execute(CLIENT, 1, 40, 3)
        self.controller.last_heartbeat -= 3
        before = len(bus.sent)
        self.controller.takeover("new-browser-session")
        self.assertEqual(len(bus.sent), before)
        self.assertEqual(self.controller.joints[1]["sent"], self.controller.joints[1]["target"])

    def test_failed_disable_does_not_report_confirmed_shutdown(self):
        bus = self.enable()
        def fail(motor, enabled):
            raise RuntimeError("no response")
        bus.set_enabled = fail
        with self.assertRaises(ValueError):
            self.controller.disable(CLIENT, 1)
        self.assertIn("失能未确认", self.controller.joints[1]["fault"])
        with self.assertRaises(ValueError):
            self.controller.disconnect(CLIENT)

    def test_enable_failure_preserves_diagnostics_before_rollback(self):
        bus = self.connect()
        def failed_hold(joint, position):
            bus.faults[joint["id"]] = 1
            return bus.snapshot(joint["id"])
        bus.enable_hold = failed_hold
        with self.assertRaisesRegex(ValueError, "故障位=0x01"):
            self.controller.enable(CLIENT, 1)
        self.assertFalse(self.controller.joints[1]["enabled"])
        self.assertEqual(self.controller.joints[1]["last_enable_status"]["fault_bits"], 1)
        self.assertEqual(self.controller.events[0]["feedback"]["fault_bits"], 1)

    def test_direct_drag_accepts_full_range_but_remains_speed_limited(self):
        self.enable()
        angle = self.controller.joints[1]["sent"]
        self.controller.execute(CLIENT, 1, 100, 3, direct=True, revision=self.controller.control_revision)
        self.assertEqual(self.controller.joints[1]["sent"], angle)
        self.assertAlmostEqual(math.degrees(self.controller.joints[1]["target"]), 100)
        self.controller.tick(.01)
        self.assertLessEqual(abs(math.degrees(self.controller.joints[1]["sent"] - angle)), .030001)

    def test_stop_rejects_late_drag_requests(self):
        bus = self.enable()
        old_revision = self.controller.control_revision
        self.controller.stop(CLIENT)
        before = len(bus.sent)
        with self.assertRaisesRegex(ValueError, "操作已停止"):
            self.controller.execute(CLIENT, 1, 100, 3, direct=True, revision=old_revision)
        self.assertEqual(len(bus.sent), before)
        self.assertEqual(self.controller.joints[1]["sent"], self.controller.joints[1]["target"])

    def test_direct_drag_requires_current_revision_and_valid_limits(self):
        self.enable()
        for revision in (None, True, "0", -1):
            with self.assertRaises(ValueError):
                self.controller.execute(CLIENT, 1, 40, 3, direct=True, revision=revision)
        for angle,speed in [(151,3),(40,self.config["max_speed_deg_s"] + 1),(float("nan"),3)]:
            with self.assertRaises(ValueError):
                self.controller.execute(CLIENT, 1, angle, speed, direct=True, revision=self.controller.control_revision)

    def test_manual_group_enable_holds_each_measured_position(self):
        bus = self.connect()
        initial = dict(bus.positions)
        self.controller.enable_all(CLIENT)
        self.assertTrue(all(j["enabled"] for j in self.controller.joints.values()))
        self.assertEqual(bus.positions, initial)
        self.assertEqual([f[1] for f in bus.sent if f[0] == 3], list(range(1,8)))
        self.assertFalse(any(f[0] in (6,7,22) for f in bus.sent))

    def test_partial_group_enable_does_not_drop_already_enabled_joints(self):
        bus = self.connect()
        original = bus.enable_hold
        def fail_third(joint, position):
            if joint["id"] == 3:
                raise RuntimeError("模拟第三关节故障")
            return original(joint,position)
        bus.enable_hold = fail_third
        with self.assertRaisesRegex(ValueError, "整臂使能未完成"):
            self.controller.enable_all(CLIENT)
        self.assertTrue(bus.active[1] and bus.active[2])
        self.assertFalse(any(bus.active[i] for i in range(3,8)))

    def test_motion_velocity_reference_matches_sent_trajectory_and_stop_is_zero(self):
        bus = self.enable()
        self.controller.execute(CLIENT, 1, 40, 15)
        before = self.controller.joints[1]["sent"]
        self.controller.tick(.005)
        sent = self.controller.joints[1]["sent"]
        self.assertGreater(bus.velocity_commands[-1][1], 0)
        self.assertAlmostEqual(bus.velocity_commands[-1][1], (sent-before)/.005)
        self.controller.stop(CLIENT)
        self.controller.tick(.005)
        self.assertEqual(bus.velocity_commands[-1][1], 0)

    def test_reset_requires_all_enabled_and_never_enables_implicitly(self):
        bus = self.enable()
        before = len(bus.sent)
        with self.assertRaisesRegex(ValueError, "先全部使能"):
            self.controller.reset_all(CLIENT,15,self.controller.control_revision)
        self.assertEqual(before,len(bus.sent))
        self.assertFalse(self.controller.resetting)

    def test_reset_uses_smooth_targets_and_keeps_enable_after_completion(self):
        bus = self.connect()
        bus.positions[2] = math.radians(20)
        self.controller.enable_all(CLIENT)
        initial = {i:j['sent'] for i,j in self.controller.joints.items()}
        before = len(bus.sent)
        self.controller.reset_all(CLIENT,15,self.controller.control_revision)
        self.assertTrue(self.controller.resetting)
        self.assertEqual(before,len(bus.sent))
        self.assertEqual(initial,{i:j['sent'] for i,j in self.controller.joints.items()})
        self.assertTrue(all(j['target']==0 for j in self.controller.joints.values()))
        for _ in range(500):
            self.controller.heartbeat(CLIENT)
            self.controller.tick(.01)
        self.assertFalse(self.controller.resetting)
        self.assertTrue(all(j['enabled'] for j in self.controller.joints.values()))
        self.assertTrue(all(abs(j['sent']) < 1e-9 for j in self.controller.joints.values()))
        self.assertFalse(any(f[0] in (4,6,7,22) for f in bus.sent[before:]))
        self.assertEqual(self.controller.events[0]['action'],'reset_complete')

    def test_reset_validation_is_atomic_and_stop_invalidates_old_reset(self):
        bus = self.connect()
        self.controller.enable_all(CLIENT)
        initial = {i:j['target'] for i,j in self.controller.joints.items()}
        bus.faults[7]=1
        with self.assertRaises(ValueError):
            self.controller.reset_all(CLIENT,15,self.controller.control_revision)
        self.assertEqual(initial,{i:j['target'] for i,j in self.controller.joints.items()})
        bus.faults[7]=0
        old_revision=self.controller.control_revision
        self.controller.stop(CLIENT)
        with self.assertRaisesRegex(ValueError,'操作已停止'):
            self.controller.reset_all(CLIENT,15,old_revision)

    def test_stop_and_fault_cancel_entire_reset(self):
        bus = self.connect()
        self.controller.enable_all(CLIENT)
        self.controller.reset_all(CLIENT,15,self.controller.control_revision)
        self.controller.stop(CLIENT)
        self.assertFalse(self.controller.resetting)
        self.assertTrue(all(j['sent']==j['target'] for j in self.controller.joints.values()))
        self.controller.reset_all(CLIENT,15,self.controller.control_revision)
        bus.faults[2]=1
        self.controller.tick(.01)
        self.assertFalse(self.controller.resetting)
        self.assertTrue(all(j['sent']==j['target'] for j in self.controller.joints.values()))
        self.assertIn('复位中止',self.controller.reset_error)

    def test_reset_timeout_freezes_targets(self):
        self.connect()
        self.controller.enable_all(CLIENT)
        self.controller.reset_all(CLIENT,15,self.controller.control_revision)
        self.controller.reset_deadline=time.monotonic()-1
        self.controller.tick(.01)
        self.assertFalse(self.controller.resetting)
        self.assertIn('验证超时',self.controller.reset_error)
        self.assertTrue(all(j['sent']==j['target'] for j in self.controller.joints.values()))


class EnableAcknowledgementTests(unittest.TestCase):
    """Exercise real request synchronization with in-memory status replies."""
    def setUp(self):
        self.config = load_config()
        self.bus = SocketCAN(self.config)  # Constructor does not open a socket.
        self.frames = []

    def publish(self, mode, fault=0):
        with self.bus.condition:
            self.bus.replies[(1, "status")] = (time.monotonic(), {
                "mode_state":mode, "fault_bits":fault, "fault_raw":0,
                "frame_id":f"0x{(2 << 24) | (mode << 22) | (fault << 16) | (1 << 8) | 253:08X}",
            })
            self.bus.condition.notify_all()

    def test_enable_ignores_reset_reply_and_drives_same_hold_until_run(self):
        def send(kind, motor, data=bytes(8), extra=None):
            self.frames.append((kind, motor, data, extra))
            if kind == 3:
                self.publish(0)  # Old firmware reports reset on ENABLE.
            elif kind == 1:
                self.publish(2 if sum(f[0] == 1 for f in self.frames) >= 2 else 0)
        self.bus.send = send
        angle = math.radians(35)
        reply = self.bus.enable_hold(self.config["joints"][0], angle)
        self.assertEqual(reply["mode_state"], 2)
        holds = [f for f in self.frames if f[0] == 1]
        self.assertGreaterEqual(len(holds), 2)
        self.assertTrue(all(f[2:] == holds[0][2:] for f in holds))
        self.assertEqual(holds[0][2:], (mit_payload("rs-06", angle, 50, 3)[1], 32767))
        self.assertEqual(sum(f[0] == 3 for f in self.frames), 1)
        self.assertIsNone(self.bus.sock)

    def test_reset_is_never_accepted_as_success_and_timeout_explains_it(self):
        def send(*args, **kwargs):
            self.publish(0)
        self.bus.send = send
        with self.assertRaisesRegex(RuntimeError, "状态=失能/复位\\(0\\)"):
            self.bus._request(3, 1, "status", timeout=.02,
                              accept=lambda s: s["mode_state"] == 2, operation="使能")

    def test_fault_reply_is_returned_for_controller_to_reject(self):
        self.bus.send = lambda *args, **kwargs: self.publish(0, 4)
        reply = self.bus.enable_hold(self.config["joints"][0], 0)
        self.assertEqual(reply["fault_bits"], 4)

    def test_disable_waits_past_late_running_reply(self):
        def send(*args, **kwargs):
            self.publish(2)
            self.timer = threading.Timer(.01, lambda: self.publish(0))
            self.timer.start()
        self.bus.send = send
        try:
            self.assertEqual(self.bus.set_enabled(1, False)["mode_state"], 0)
        finally:
            self.timer.join()


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.controller = Controller(load_config(), simulate=True, start_worker=False,
                                     audit_path=Path(self.temp.name) / "audit.jsonl")
        self.server = make_server(self.controller, 0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.server.server_port
        self.http = http.client.HTTPConnection("127.0.0.1", self.port)

    def tearDown(self):
        self.http.close()
        self.server.shutdown()
        self.server.server_close()
        self.controller.close()
        self.temp.cleanup()

    def request(self, method, path, body=None, headers=None):
        self.http.request(method, path, body=body, headers=headers or {})
        r = self.http.getresponse()
        return r.status, r.read()

    def test_page_and_state_have_no_can_side_effects(self):
        self.assertEqual(self.request("GET", "/")[0], 200)
        status, body = self.request("GET", "/api/state?client=" + CLIENT)
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["tx_count"], 0)
        self.assertIsNone(self.controller.transport)

    def test_cross_origin_and_missing_token_cannot_connect(self):
        _, body = self.request("GET", "/api/state")
        token = json.loads(body)["csrf"]
        for origin, value in [("https://example.com", token), (f"http://127.0.0.1:{self.port}", "wrong")]:
            headers = {"Content-Type":"application/json", "Origin":origin, "X-Control-Token":value}
            status, _ = self.request("POST", "/api/connect", json.dumps({"client":CLIENT}), headers)
            self.assertEqual(status, 403)
        self.assertIsNone(self.controller.transport)

    def test_correct_local_manual_connect_uses_simulation_only(self):
        _, body = self.request("GET", "/api/state")
        headers = {"Content-Type":"application/json", "Origin":f"http://127.0.0.1:{self.port}", "X-Control-Token":json.loads(body)["csrf"]}
        status, data = self.request("POST", "/api/connect", json.dumps({"client":CLIENT}), headers)
        self.assertEqual(status, 200)
        state = json.loads(data)["state"]
        self.assertTrue(state["simulate"])
        self.assertFalse(any(j["enabled"] for j in state["joints"]))


class CodecTests(unittest.TestCase):
    def test_mit_motion_encodes_velocity_reference(self):
        import struct
        for model, vmax in [("rs-06",50),("rs-00",33)]:
            _,data = mit_payload(model,0,50,3,math.radians(15))
            encoded = struct.unpack(">HHHH",data)[1]
            self.assertEqual(encoded,int((math.radians(15)/vmax+1)*32767))
            self.assertGreater(encoded,32767)

    def test_mit_hold_uses_zero_torque_not_host_id(self):
        import struct
        extra, data = mit_payload("rs-06", 0, 150, 10)
        p, velocity, kp, kd = struct.unpack(">HHHH", data)
        self.assertEqual(extra, 32767)
        self.assertEqual((p, velocity), (32767, 32767))
        self.assertEqual(kp, int(150 / 5000 * 65535))
        self.assertEqual(kd, int(10 / 100 * 65535))


if __name__ == "__main__":
    unittest.main()
