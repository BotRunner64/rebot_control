"""Remote control CLI, asynchronous scheduling, and camera transport checks."""

from contextlib import contextmanager, redirect_stdout
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from rebot import cli
from rebot.policy import cameras, client as policy
import rebot_policy_link as link

POSE = [0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 1.]


def observation(now=None):
    now = time.monotonic() if now is None else now
    state = [*POSE, .5, *POSE, .5]
    snapshot = {"state": state, "observed_at": now, "acquired_at": {"left": now, "right": now}}
    frames = {name: (b"\xff\xd8jpeg", now) for name in link.REBOT_RGB}
    return policy.build_observation(snapshot, frames, "transport test")


def hold_reply(obs=None, horizon=50):
    return link.HoldPolicy(0., horizon=horizon)(observation() if obs is None else obs)


def hold_chunk(observed_at, x=None, horizon=50):
    """Parsed 30 Hz chunk anchored at observed_at; x optionally sets left x per row."""
    reply = hold_reply(observation(observed_at), horizon)
    if x is not None:
        reply["state"]["left_eef"]["pose"] = [[float(v), *POSE[1:]] for v in x]
    return policy.parse_actions(reply)

class SlowClient:
    timeout = .5

    def __init__(self, latency=.12, fail=False):
        self.latency, self.fail = latency, fail
        self.active = self.max_active = self.calls = 0
        self.closed = False

    def infer(self, observation):
        self.active += 1
        self.max_active = max(self.max_active, self.active)
        self.calls += 1
        try:
            time.sleep(self.latency)
            if self.fail:
                raise TimeoutError("injected inference timeout")
            return hold_reply(observation)
        finally:
            self.active -= 1

    def close(self):
        self.closed = True

@contextmanager
def endpoint(infer, metadata=None):
    servers = []
    thread = threading.Thread(target=link.serve, args=(infer, metadata or link.metadata(), "127.0.0.1", 0),
                              kwargs={"ready": servers.append})
    thread.start()
    try:
        while not servers and thread.is_alive():
            time.sleep(.005)
        yield servers[0].socket.getsockname()[1]
    finally:
        for server in servers:
            server.shutdown()
        thread.join(timeout=2)

def capture_stream(directory, source):
    """Start a CameraStream whose capture child is the given module source."""
    (Path(directory) / "fake_policy_capture.py").write_text(source)
    with patch.object(cameras, "CAPTURE_MODULE", "fake_policy_capture"), \
            patch.dict(os.environ, {"PYTHONPATH": directory}):
        return cameras.CameraStream("synthetic")


class CLITests(unittest.TestCase):
    def test_mock_service_is_rejected_before_connecting_hardware(self):
        with endpoint(lambda obs: {}, link.metadata(mock=True)) as port, \
                patch("rebot.runtime.robot.Robot") as robot, patch.object(cameras, "PolicyCameras") as capture, \
                patch("sys.stderr", new_callable=io.StringIO):
            result = cli.main(["policy", "--port", str(port), "--instruction", "hold",
                               "--camera", "top=0", "--camera", "left_wrist=2",
                               "--camera", "right_wrist=4"])
        self.assertEqual(result, 2)
        robot.assert_not_called()
        capture.assert_not_called()


class TimelineTests(unittest.TestCase):
    def test_latency_skips_expired_rows_without_replaying_them(self):
        timeline = policy.ActionTimeline()
        timeline.accept(hold_chunk(10., x=range(50)), 10., 10.08)
        action, index = timeline.sample(10.08)
        self.assertEqual(index, 2)
        self.assertEqual(action[0], 2)
        self.assertEqual(action.shape, (26,))
        self.assertEqual(timeline.sample(10.31)[1], 9)
        with self.assertRaises(TimeoutError):
            timeline.sample(12.)

    def test_new_plan_keeps_its_own_observation_anchor(self):
        timeline = policy.ActionTimeline()
        timeline.accept(hold_chunk(10.), 10., 10.1)
        new = hold_chunk(10.5, x=np.arange(50) + 100)
        timeline.accept(new, 10.5, 10.6)
        action, index = timeline.sample(10.61)
        self.assertEqual(index, 3)
        self.assertEqual(action[0], 103)
        with self.assertRaises(ValueError):
            timeline.accept(new, 10.4, 10.7)

    def test_horizon_and_rate_come_from_the_reply_timestamps(self):
        reply = hold_reply(observation(10.))
        for name, table in reply["state"].items():
            column = link.state_column(name)
            # Eight targets at 10 Hz instead of fifty at 30 Hz.
            table["timestamp_ns"] = link.future_timestamps(10_000_000_000, 8, 10)
            table[column] = table[column][:8]
        reply["state"]["right_gripper"]["openness"] = [.1 * k for k in range(8)]
        timeline = policy.ActionTimeline()
        timeline.accept(policy.parse_actions(reply), 10., 10.05, replan_after=.5)
        action, index = timeline.sample(10.35)
        self.assertEqual(index, 3)
        self.assertAlmostEqual(action[25], .3)
        with self.assertRaisesRegex(ValueError, "replanning"):
            policy.ActionTimeline().accept(policy.parse_actions(reply), 10., 10.05, replan_after=.8)

    def test_malformed_replies_and_expired_chunks_fail(self):
        missing = hold_reply()
        del missing["state"]["right_gripper"]
        ragged = hold_reply()
        ragged["state"]["left_eef"]["pose"][3] = POSE[:11]
        nonfinite = hold_reply()
        nonfinite["state"]["left_eef"]["pose"][0] = [float("nan"), *POSE[1:]]
        skewed = hold_reply()
        skewed["state"]["left_eef"]["pose"][0] = [0., 0., 0., 2., 0., 0., 0., 1., 0., 0., 0., 1.]
        repeated = hold_reply()
        repeated["state"]["left_gripper"]["timestamp_ns"][1] = repeated["state"]["left_gripper"]["timestamp_ns"][0]
        for reply in (missing, ragged, nonfinite, skewed, repeated, {"actions": []}):
            with self.assertRaises(ValueError):
                policy.parse_actions(reply)
        wide = hold_reply()
        wide["state"]["left_gripper"]["openness"][0] = 1.4
        self.assertEqual(policy.parse_actions(wide)["left_gripper"][1][0], 1.)
        with self.assertRaises(TimeoutError):
            policy.ActionTimeline().accept(hold_chunk(10.), 10., 12.)
        with self.assertRaisesRegex(ValueError, "later than"):
            policy.ActionTimeline().accept(hold_chunk(10.), 10.04, 10.05)


class NetworkTests(unittest.TestCase):
    def test_recording_shaped_roundtrip_and_metadata(self):
        obs = observation()
        received = []

        def infer(request):
            received.append(request)
            return hold_reply(request)

        with endpoint(infer) as port:
            client = policy.connect_policy('127.0.0.1', port, 0.5)
            try:
                self.assertEqual(client.metadata["rgb"], list(link.REBOT_RGB))
                output = client.infer(obs)
            finally:
                client.close()
        self.assertEqual(received, [obs])
        self.assertEqual(set(obs), {"timestamp_ns", "instruction", "state", "rgb"})
        self.assertEqual(obs["state"]["left_eef"], {"timestamp_ns": [obs["timestamp_ns"]], "pose": [POSE]})
        self.assertEqual(obs["rgb"]["top"]["jpeg"], [b"\xff\xd8jpeg"])
        times, poses = policy.parse_actions(output)["right_eef"]
        self.assertEqual(poses.shape, (50, 12))
        np.testing.assert_allclose(times - obs["timestamp_ns"] / 1e9, (np.arange(50) + 1) / 30, atol=1e-6)

    def test_incompatible_servers_are_refused(self):
        from websockets.sync.server import serve

        with serve(lambda socket: socket.send(link.pack({"protocol": "something-else/1"})),
                   "127.0.0.1", 0) as server:
            thread = threading.Thread(target=server.serve_forever)
            thread.start()
            try:
                with self.assertRaisesRegex(ValueError, 'protocol'):
                    policy.connect_policy('127.0.0.1', server.socket.getsockname()[1], 0.5)
            finally:
                server.shutdown()
                thread.join(timeout=2)
        with endpoint(lambda obs: {}, link.metadata(state=["left_eef", "head_eef"])) as port:
            with self.assertRaisesRegex(ValueError, 'head_eef'):
                policy.connect_policy('127.0.0.1', port, 0.5)

    def test_server_side_failures_reach_the_client_as_errors(self):
        def broken(obs):
            raise RuntimeError("model exploded")

        with endpoint(broken) as port:
            client = policy.connect_policy('127.0.0.1', port, 0.5)
            try:
                with self.assertRaisesRegex(RuntimeError, "model exploded"):
                    client.infer(observation())
            finally:
                client.close()
        with endpoint(hold_reply) as port:
            client = policy.connect_policy('127.0.0.1', port, 0.5)
            try:
                incomplete = observation()
                del incomplete["rgb"]["top"]
                with self.assertRaisesRegex(RuntimeError, "lacks required rgb"):
                    client.infer(incomplete)
            finally:
                client.close()

    def test_socket_timeout_is_bounded(self):

        def delay(obs):
            time.sleep(0.15)
            return hold_reply(obs)
        with endpoint(delay) as port:
            client = policy.connect_policy('127.0.0.1', port, 0.03)
            began = time.monotonic()
            try:
                with self.assertRaises(TimeoutError):
                    client.infer(observation())
                self.assertLess(time.monotonic() - began, 0.12)
            finally:
                client.close()

class AsyncTests(unittest.TestCase):
    def test_control_continues_during_inference_with_one_request_in_flight(self):
        client = SlowClient()
        during = []

        class Robot:
            def check(self):
                pass

            def command(self, actions):
                during.append(client.active)

        with redirect_stdout(io.StringIO()):
            report = policy.control_loop(Robot(), client,
                lambda: (observation(), time.monotonic()), threading.Event(),
                duration=.8, replan_steps=6)
        self.assertGreaterEqual(report["commands"], 15)
        self.assertGreaterEqual(client.calls, 3)
        self.assertEqual(client.max_active, 1)
        self.assertIn(1, during)
        self.assertTrue(client.closed)

    def test_blocked_targets_do_not_end_async_inference(self):
        client = SlowClient(.02)
        stop = threading.Event()

        class Robot:
            calls = 0

            def check(self):
                pass

            def command(self, actions):
                self.calls += 1
                return self.calls > 5

        robot = Robot()
        with redirect_stdout(io.StringIO()):
            report = policy.control_loop(robot, client,
                lambda: (observation(), time.monotonic()), stop,
                duration=.5, replan_steps=3)
        self.assertGreaterEqual(client.calls, 3)
        self.assertGreater(report["commands"], 0)
        self.assertLess(report["commands"], robot.calls)
        self.assertFalse(stop.is_set())

    def test_inference_failure_sets_stop_and_is_reported(self):
        client = SlowClient(.02, fail=True)
        stop = threading.Event()
        with policy.InferenceWorker(client, lambda: (observation(), time.monotonic()), stop) as worker:
            self.assertTrue(worker.request())
            self.assertFalse(worker.request())
            self.assertTrue(stop.wait(.5))
            with self.assertRaisesRegex(RuntimeError, "injected inference timeout"):
                worker.check()

    def test_outer_deadline_also_bounds_a_blocked_send_or_observation(self):
        released = threading.Event()
        client = SlowClient()
        client.timeout = .03
        client.infer = lambda obs: (released.wait(2), hold_reply())[1]
        client.close = released.set
        stop = threading.Event()
        with policy.InferenceWorker(client, lambda: (observation(), time.monotonic()), stop) as worker:
            worker.request()
            time.sleep(.05)
            with self.assertRaises(TimeoutError):
                worker.check()
        self.assertTrue(stop.is_set())
        self.assertFalse(worker.thread.is_alive())

class CameraTests(unittest.TestCase):
    def test_capture_jpeg_is_forwarded_untouched_and_process_is_cleaned_up(self):
        from PIL import Image

        jpeg = io.BytesIO()
        Image.fromarray(np.full((240, 320, 3), [20, 40, 60], np.uint8)).save(jpeg, format='JPEG')
        script = "import argparse,socket,struct,time\np=argparse.ArgumentParser();p.add_argument('--source');p.add_argument('--socket-fd',type=int)\na=p.parse_args();s=socket.socket(fileno=a.socket_fd)\nwhile True:\n payload=struct.pack('!d',time.monotonic())+JPEG\n s.sendall(struct.pack('!I',len(payload))+payload);time.sleep(.02)\n".replace('JPEG', repr(jpeg.getvalue()))
        with tempfile.TemporaryDirectory() as directory:
            stream = capture_stream(directory, script)
            try:
                self.assertTrue(stream.ready.wait(3.0))
                frame, stamp = stream.snapshot(1.0)
                self.assertEqual(frame, jpeg.getvalue())
                self.assertLess(time.monotonic() - stamp, 1.0)
            finally:
                stream.close()
            self.assertIsNotNone(stream.process.poll())
            self.assertFalse(stream.thread.is_alive())

    def test_capture_child_failure_is_reported_without_waiting_ten_seconds(self):
        with tempfile.TemporaryDirectory() as directory:
            stream = capture_stream(directory, 'raise SystemExit(3)\n')
            try:
                self.assertTrue(stream.ready.wait(3.0))
                with self.assertRaisesRegex(RuntimeError, 'failed'):
                    stream.snapshot(1.0)
            finally:
                stream.close()
            self.assertFalse(stream.thread.is_alive())

    def test_cameras_must_match_the_streams_the_policy_declares(self):
        for sources in (['top=0'], ['top=0', 'left_wrist=0', 'right_wrist=2'],
                        ['top=0', 'left_wrist=2', 'right_wrist=4', 'extra=6']):
            with self.assertRaises(ValueError):
                cameras.PolicyCameras(sources, link.REBOT_RGB)
        cameras.PolicyCameras(['front=0', 'side=2'], ['side', 'front'])

    def test_stale_camera_frame_fails(self):
        stream = cameras.CameraStream.__new__(cameras.CameraStream)
        stream.lock = threading.Lock()
        stream.error = None
        stream.source = 'synthetic'
        stream.frame = b'\xff\xd8'
        stream.acquired_at = time.monotonic() - 1
        with self.assertRaises(TimeoutError):
            stream.snapshot(0.25)
