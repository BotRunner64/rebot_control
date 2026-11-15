#!/usr/bin/env python3
"""Policy-agnostic link between a robot controller and a policy server.

Messages reuse the recording contract. Every stream is a small table with the
columns of its recorded Parquet file. An observation carries the most recent
rows of each stream; an action reply carries predicted future rows of the state
streams, stamped in the controller's clock. Nothing here knows about a model.

Wire format: WebSocket binary frames holding plain MessagePack. The server sends
its metadata first, then answers each observation with one reply. A text frame
from the server is an error report.
"""

import argparse
import math
import re
import socket
import threading
import time
import traceback

import msgpack

PROTOCOL = "rebot-policy-link/1"
# Stream kind is the name suffix, as in state/<side>_eef.parquet and state/<side>_gripper.parquet.
STATE_COLUMNS = {"eef": "pose", "gripper": "openness"}
REBOT_STATE = ("left_eef", "right_eef", "left_gripper", "right_gripper")
REBOT_RGB = ("top", "left_wrist", "right_wrist")
NAME = re.compile(r"[a-z0-9_]+")


def pack(value):
    return msgpack.packb(value, use_bin_type=True)


def unpack(frame):
    return msgpack.unpackb(frame, raw=False)


def state_column(name):
    """Return the value column of a state stream, e.g. left_eef -> pose."""
    column = STATE_COLUMNS.get(name.rsplit("_", 1)[-1]) if isinstance(name, str) else None
    if column is None or not NAME.fullmatch(name):
        raise ValueError(f"Unknown state stream {name!r}; names end in _eef or _gripper.")
    return column


def future_timestamps(timestamp_ns, count, hz):
    """Row k is the target for timestamp_ns + (k + 1) / hz, in integer nanoseconds."""
    return [timestamp_ns + (k + 1) * 1_000_000_000 // hz for k in range(count)]


def metadata(state=REBOT_STATE, rgb=REBOT_RGB, *, mock=False, **info):
    """Server metadata: the streams this policy needs in every observation."""
    return validate_metadata({**info, "protocol": PROTOCOL, "state": list(state),
                              "rgb": list(rgb), "mock": mock})


def validate_metadata(value):
    if not isinstance(value, dict) or value.get("protocol") != PROTOCOL:
        raise ValueError(f"Policy metadata must declare protocol {PROTOCOL!r}.")
    for key in ("state", "rgb"):
        names = value.get(key)
        if (not isinstance(names, list) or len(set(map(str, names))) != len(names)
                or not all(isinstance(name, str) and NAME.fullmatch(name) for name in names)):
            raise ValueError(f"Policy metadata {key} must list unique lowercase stream names.")
    for name in value["state"]:
        state_column(name)
    if type(value.get("mock")) is not bool:
        raise ValueError("Policy metadata must declare mock as a boolean.")
    return value


def _integer(value):
    return type(value) is int


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _table(value, column, label, *, increasing=False):
    if not isinstance(value, dict) or set(value) != {"timestamp_ns", column}:
        raise ValueError(f"{label} must contain exactly timestamp_ns and {column}.")
    times, rows = value["timestamp_ns"], value[column]
    if (not isinstance(times, list) or not isinstance(rows, list) or not times
            or len(times) != len(rows) or not all(map(_integer, times))):
        raise ValueError(f"{label} needs equally long, nonempty timestamp_ns and {column} lists.")
    if any(b < a or (increasing and b == a) for a, b in zip(times, times[1:])):
        raise ValueError(f"{label} timestamps are out of order.")
    if column == "pose":
        valid = all(isinstance(row, list) and len(row) == 12 and all(map(_number, row)) for row in rows)
    elif column == "openness":
        valid = all(map(_number, rows))
    else:
        valid = all(isinstance(row, bytes) and row for row in rows)
    if not valid:
        raise ValueError(f"{label} has malformed {column} rows.")
    return value


def validate_observation(value, meta):
    """Check an observation against the streams the server declared it needs."""
    if (not isinstance(value, dict) or not _integer(value.get("timestamp_ns"))
            or not isinstance(value.get("instruction"), str) or not value["instruction"].strip()
            or not isinstance(value.get("state"), dict) or not isinstance(value.get("rgb"), dict)):
        raise ValueError("Observation needs timestamp_ns, a nonempty instruction, state and rgb.")
    for key in ("state", "rgb"):
        missing = set(meta[key]) - set(value[key])
        if missing:
            raise ValueError(f"Observation lacks required {key} streams: {sorted(missing)}.")
    for name, table in value["state"].items():
        _table(table, state_column(name), f"state/{name}")
    for name, table in value["rgb"].items():
        _table(table, "jpeg", f"rgb/{name}")
    return value


def validate_actions(value):
    """Check an action reply: future rows of state streams with strictly increasing time."""
    if not isinstance(value, dict) or not isinstance(value.get("state"), dict) or not value["state"]:
        raise ValueError("Action reply must contain a nonempty state mapping.")
    for name, table in value["state"].items():
        _table(table, state_column(name), f"action state/{name}", increasing=True)
    return value


class PolicyClient:
    """One connection, one request in flight; every socket operation is bounded by timeout."""

    def __init__(self, host, port, timeout=1.):
        from websockets.sync.client import connect

        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        self.timeout = timeout
        self._socket = connect(f"ws://{host}:{port}", compression=None, max_size=None,
                               open_timeout=timeout, close_timeout=1.)
        try:
            self.metadata = validate_metadata(self._receive())
        except BaseException:
            self.close()
            raise

    def _receive(self):
        frame = self._socket.recv(timeout=self.timeout)
        if isinstance(frame, str):
            raise RuntimeError(f"Policy server error:\n{frame}")
        return unpack(frame)

    def infer(self, observation):
        self._socket.send(pack(observation))
        return validate_actions(self._receive())

    def close(self):
        """Close the transport and wake an inference blocked on socket I/O."""
        try:
            self._socket.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._socket.close()


def serve(infer, meta, host="127.0.0.1", port=8000, *, ready=None):
    """Serve infer(observation) -> action reply until interrupted.

    Inference is serialized across connections. ready(server) is called once listening.
    """
    from websockets.exceptions import ConnectionClosed
    from websockets.sync.server import serve as listen

    meta = validate_metadata(meta)
    lock = threading.Lock()

    def handler(connection):
        try:
            connection.send(pack(meta))
            for frame in connection:
                try:
                    observation = validate_observation(unpack(frame), meta)
                    with lock:
                        reply = infer(observation)
                    payload = pack(validate_actions(reply))
                except Exception:
                    connection.send(traceback.format_exc())
                    connection.close(1011, "Policy error; traceback sent in the previous frame.")
                    return
                connection.send(payload)
        except ConnectionClosed:
            pass

    with listen(handler, host, port, compression=None, max_size=None) as server:
        if ready is not None:
            ready(server)
        server.serve_forever()


class HoldPolicy:
    """Hold the latest observed state; for transport tests without a model."""

    def __init__(self, latency=.08, hz=30, horizon=50):
        self.latency, self.hz, self.horizon = latency, hz, horizon

    def __call__(self, observation):
        time.sleep(self.latency)
        times = future_timestamps(observation["timestamp_ns"], self.horizon, self.hz)
        return {"state": {
            name: {"timestamp_ns": times, state_column(name): [table[state_column(name)][-1]] * self.horizon}
            for name, table in observation["state"].items()}}


def main(argv=None):
    cli = argparse.ArgumentParser(description="Serve a mock hold policy for transport tests.")
    cli.add_argument("--host", default="127.0.0.1")
    cli.add_argument("--port", type=int, default=8001)
    cli.add_argument("--latency", type=float, default=.08, help="Simulated inference seconds")
    cli.add_argument("--camera", action="append", metavar="NAME",
                     help=f"Required camera stream; repeatable (default: {' '.join(REBOT_RGB)})")
    args = cli.parse_args(argv)
    if not 1 <= args.port <= 65535 or not math.isfinite(args.latency) or args.latency < 0:
        cli.error("invalid port or latency")
    meta = metadata(rgb=args.camera or REBOT_RGB, mock=True)
    print(f"Mock hold policy: ws://{args.host}:{args.port}; cameras {meta['rgb']}.", flush=True)
    try:
        serve(HoldPolicy(args.latency), meta, args.host, args.port)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
