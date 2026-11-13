"""Remote control CLI, asynchronous scheduling, and camera transport checks."""

from contextlib import contextmanager, redirect_stdout
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
from openpi_client.msgpack_numpy import packb as pack, unpackb as unpack
from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve

import rebot_policy as policy
import rebot_policy_camera as cameras

def observation():
    state = np.zeros(26, np.float32)
    for offset in (0, 13):
        state[offset + 3:offset + 12] = np.eye(3).ravel(order="F")
        state[offset + 12] = .5
    return {"state": state,
            "image": {key: np.zeros((224, 224, 3), np.uint8) for key in cameras.CAMERA_KEYS},
            "image_mask": {key: np.bool_(True) for key in cameras.CAMERA_KEYS},
            "prompt": "transport test"}

def hold_actions(state=None):
    state = observation()["state"] if state is None else state
    return np.repeat(np.asarray(state)[None], 50, axis=0)

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
            return {"actions": hold_actions(observation["state"])}
        finally:
            self.active -= 1

    def close(self):
        self.closed = True

@contextmanager
def endpoint(infer, metadata=None):
    def handler(socket):
        try:
            socket.send(pack(policy.METADATA if metadata is None else metadata))
            for request in socket:
                result = infer(unpack(request))
                socket.send(pack(result))
        except ConnectionClosed:
            pass

    with serve(handler, "127.0.0.1", 0, compression=None, close_timeout=.1) as server:
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            yield server.socket.getsockname()[1]
        finally:
            server.shutdown()
            thread.join(timeout=2)

class CLITests(unittest.TestCase):
    def test_original_command_runs_against_a_remote_policy(self):
        prompts = []

        def infer(obs):
            prompts.append(obs["prompt"])
            self.assertEqual(set(obs["image"]), set(cameras.CAMERA_KEYS))
            time.sleep(.04)
            return {"actions": hold_actions(obs["state"])}

        with endpoint(infer) as port:
            result = subprocess.run([
                sys.executable, str(Path(policy.__file__)), "--host", "127.0.0.1", "--port", str(port),
                "--instruction", "hold test", "--mock-cameras", "--duration", ".4",
            ], capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Policy SIMULATION", result.stdout)
        self.assertIn("Policy control:", result.stdout)
        self.assertEqual(prompts, ["hold test"])

    def test_mock_service_is_rejected_before_connecting_hardware(self):
        metadata = {**policy.METADATA, "mock_policy": True}
        with endpoint(lambda obs: {}, metadata=metadata) as port, \
                patch("rebot_control.Robot") as robot, patch.object(cameras, "PolicyCameras") as capture, \
                patch("sys.stderr", new_callable=io.StringIO):
            result = policy.main(["--port", str(port), "--instruction", "hold", "--hardware",
                                  "--camera", "top=0", "--camera", "left_wrist=2",
                                  "--camera", "right_wrist=4"])
        self.assertEqual(result, 2)
        robot.assert_not_called()
        capture.assert_not_called()


class TimelineTests(unittest.TestCase):
    def test_latency_skips_expired_rows_without_replaying_them(self):
        chunk = hold_actions()
        chunk[:, 0] = np.arange(50)
        timeline = policy.ActionTimeline()
        timeline.accept(chunk, 10., 10.08)
        action, index = timeline.sample(10.08)
        self.assertEqual(index, 2)
        self.assertEqual(action[0], 2)
        self.assertEqual(timeline.sample(10.30)[1], 9)
        with self.assertRaises(TimeoutError):
            timeline.sample(12.)

    def test_new_plan_keeps_its_own_observation_anchor(self):
        timeline = policy.ActionTimeline()
        timeline.accept(hold_actions(), 10., 10.1)
        new = hold_actions()
        new[:, 0] = np.arange(50) + 100
        timeline.accept(new, 10.5, 10.6)
        action, index = timeline.sample(10.61)
        self.assertEqual(index, 3)
        self.assertEqual(action[0], 103)
        with self.assertRaises(ValueError):
            timeline.accept(new, 10.4, 10.7)

    def test_bad_shapes_nonfinite_relative_actions_and_expired_chunks_fail(self):
        for shape in ((50, 20), (50, 32), (49, 26)):
            with self.assertRaises(ValueError):
                policy.validate_actions(np.zeros(shape))
        value = hold_actions()
        value[0, 0] = np.nan
        with self.assertRaises(ValueError):
            policy.validate_actions(value)
        with self.assertRaises(TimeoutError):
            policy.ActionTimeline().accept(hold_actions(), 10., 12.)

class NetworkTests(unittest.TestCase):

    def test_numpy_protocol_roundtrip_and_metadata(self):
        obs = observation()
        with endpoint(lambda obs: {'actions': hold_actions(obs['state'])}) as port:
            client = policy.connect_policy('127.0.0.1', port, 0.5)
            try:
                output = client.infer(obs)
                np.testing.assert_array_equal(output['actions'], hold_actions())
            finally:
                client.close()
        with endpoint(lambda obs: {}, metadata={}) as port:
            with self.assertRaisesRegex(ValueError, 'metadata'):
                policy.connect_policy('127.0.0.1', port, 0.5)

    def test_socket_timeout_is_bounded(self):

        def delay(obs):
            time.sleep(0.15)
            return {'actions': hold_actions()}
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
        client.infer = lambda obs: (released.wait(2), {"actions": hold_actions()})[1]
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

    def test_existing_preview_timestamp_transport_and_process_cleanup(self):
        jpeg = io.BytesIO()
        cameras.Image.fromarray(np.full((240, 320, 3), [20, 40, 60], np.uint8)).save(jpeg, format='JPEG')
        script = "import argparse,socket,struct,time\np=argparse.ArgumentParser();p.add_argument('--source');p.add_argument('--socket-fd',type=int)\na=p.parse_args();s=socket.socket(fileno=a.socket_fd)\nwhile True:\n payload=struct.pack('!d',time.monotonic())+JPEG\n s.sendall(struct.pack('!I',len(payload))+payload);time.sleep(.02)\n".replace('JPEG', repr(jpeg.getvalue()))
        with tempfile.TemporaryDirectory() as directory:
            helper = Path(directory) / 'camera.py'
            helper.write_text(script)
            with patch.object(cameras, 'CAPTURE_SCRIPT', helper):
                stream = cameras.CameraStream('synthetic')
            try:
                self.assertTrue(stream.ready.wait(3.0))
                frame, stamp = stream.snapshot(1.0)
                self.assertEqual(frame.shape, (224, 224, 3))
                self.assertLess(time.monotonic() - stamp, 1.0)
                np.testing.assert_allclose(frame[112, 112], [20, 40, 60], atol=3)
            finally:
                stream.close()
            self.assertIsNotNone(stream.process.poll())
            self.assertFalse(stream.thread.is_alive())

    def test_capture_child_failure_is_reported_without_waiting_ten_seconds(self):
        with tempfile.TemporaryDirectory() as directory:
            helper = Path(directory) / 'camera.py'
            helper.write_text('raise SystemExit(3)\n')
            with patch.object(cameras, 'CAPTURE_SCRIPT', helper):
                stream = cameras.CameraStream('synthetic')
            try:
                self.assertTrue(stream.ready.wait(3.0))
                with self.assertRaisesRegex(RuntimeError, 'failed'):
                    stream.snapshot(1.0)
            finally:
                stream.close()
            self.assertFalse(stream.thread.is_alive())

    def test_missing_and_duplicate_camera_roles_fail(self):
        for sources in (['top=0'], ['top=0', 'left_wrist=0', 'right_wrist=2']):
            with self.assertRaises(ValueError):
                cameras.PolicyCameras(sources)

    def test_stale_camera_frame_fails(self):
        stream = cameras.CameraStream.__new__(cameras.CameraStream)
        stream.lock = threading.Lock()
        stream.error = None
        stream.source = 'synthetic'
        stream.frame = np.zeros((224, 224, 3), np.uint8)
        stream.acquired_at = time.monotonic() - 1
        with self.assertRaises(TimeoutError):
            stream.snapshot(0.25)
