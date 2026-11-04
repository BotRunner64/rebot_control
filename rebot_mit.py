"""Bounded RobStride private-CAN transport for experimental MIT control."""

from dataclasses import dataclass
import math
import struct
import sys
import time

import can

from rebot import HOST_ID, MOTOR_IDS, check_channel


@dataclass(frozen=True)
class Limits:
    position: float
    velocity: float
    torque: float
    kp: float
    kd: float


# Manufacturer RS06/RS00 manuals: do not use the installed SDK's different
# torque/velocity scaling tables for these B601-RS motors.
LIMITS = {"rs-06": Limits(4 * math.pi, 50.0, 36.0, 5000.0, 100.0),
          "rs-00": Limits(4 * math.pi, 33.0, 14.0, 500.0, 5.0)}
WATCHDOG = 0x7028
MODE = 0x7005
POSITION = 0x7019


def encode_mit(motor_id, model, position, torque, kp=0.0, kd=0.0, velocity=0.0,
               *, allow_gripper=False):
    limits = LIMITS[model]
    if (type(motor_id) is not int or not 1 <= motor_id <= (7 if allow_gripper else 6)
            or not math.isfinite(position) or abs(position) > limits.position
            or not math.isfinite(torque) or abs(torque) > limits.torque
            or not math.isfinite(velocity) or abs(velocity) > limits.velocity
            or not math.isfinite(kp) or not 0 <= kp <= limits.kp
            or not math.isfinite(kd) or not 0 <= kd <= limits.kd):
        raise ValueError("Invalid MIT motor ID, position, torque, or gains.")

    def unsigned(value, bound):
        return int((value + bound) / (2 * bound) * 65535)

    payload = struct.pack(">HHHH", unsigned(position, limits.position),
                          unsigned(velocity, limits.velocity), int(kp / limits.kp * 65535),
                          int(kd / limits.kd * 65535))
    return can.Message(arbitration_id=(1 << 24) | (unsigned(torque, limits.torque) << 8) | motor_id,
                       is_extended_id=True, data=payload)


def decode_mit(frame, model):
    """Decode transmitted commands for quasi-static effort estimation."""
    limits = LIMITS[model]
    position, velocity, kp, kd = struct.unpack(">HHHH", frame.data)
    return {"position": position / 65535 * 2 * limits.position - limits.position,
            "velocity": velocity / 65535 * 2 * limits.velocity - limits.velocity,
            "kp": kp / 65535 * limits.kp, "kd": kd / 65535 * limits.kd,
            "gravity": ((frame.arbitration_id >> 8) & 65535) / 65535 * 2 * limits.torque - limits.torque}


class MotorBus:
    def __init__(self, channel, models, timeout=0.03, max_temperature=65.0, motor_count=6):
        if type(motor_count) is not int or motor_count not in (6, 7):
            raise ValueError("Control requires six arm motors or seven motors including the gripper.")
        if len(models) < motor_count or any(model not in LIMITS for model in models[:motor_count]):
            raise ValueError("Missing or unsupported controlled motor models.")
        check_channel(channel)
        self.motor_ids = tuple(range(1, motor_count + 1))
        self.channel = channel
        self.models = models
        self.timeout = timeout
        self.max_temperature = max_temperature
        self.bus = can.Bus(interface="socketcan", channel=channel, receive_own_messages=False)
        self.enabled = set()
        self.states = {}
        self.feedback = {}
        self.hold_records = {}
        self.hold_gravity = None

    def encode(self, motor, position, torque, kp=0., kd=0., velocity=0.):
        if motor not in self.motor_ids:
            raise ValueError("Motor is outside the selected control group.")
        return encode_mit(motor, self.models[motor - 1], position, torque, kp, kd, velocity,
                          allow_gripper=7 in self.motor_ids)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        try:
            self.freeze()
        finally:
            self.bus.shutdown()

    def send(self, kind, motor_id, payload=bytes(8)):
        self.bus.send(can.Message(arbitration_id=(kind << 24) | (HOST_ID << 8) | motor_id,
                                  is_extended_id=True, data=payload), timeout=0.01)

    def inspect(self, frame):
        if frame.is_error_frame:
            raise RuntimeError(f"{self.channel}: CAN error frame.")
        if (not frame.is_extended_id or frame.is_remote_frame or len(frame.data) != 8
                or not math.isfinite(frame.timestamp)
                or not -0.02 <= time.time() - frame.timestamp <= 0.15):
            return None
        arb = frame.arbitration_id
        kind, motor, host = (arb >> 24) & 31, (arb >> 8) & 255, arb & 255
        if motor not in MOTOR_IDS or host != (0xFE if kind == 0 else HOST_ID):
            return None
        if kind == 21 and any(frame.data):
            raise RuntimeError(f"{self.channel} motor {motor}: fault/warning {bytes(frame.data).hex()}")
        if kind in (2, 24):
            faults, mode = (arb >> 16) & 63, (arb >> 22) & 3
            temperature = struct.unpack_from(">H", frame.data, 6)[0] * 0.1
            if faults or temperature >= self.max_temperature:
                raise RuntimeError(f"{self.channel} motor {motor}: faults=0x{faults:02x}, "
                                   f"temperature={temperature:.1f} C")
            self.states[motor] = mode
            if motor in self.motor_ids:
                limits = LIMITS[self.models[motor - 1]]
                _, velocity, torque, _ = struct.unpack(">HHHH", frame.data)
                self.feedback[motor] = {
                    "timestamp": frame.timestamp,
                    "velocity": velocity / 65535 * 2 * limits.velocity - limits.velocity,
                    "torque": torque / 65535 * 2 * limits.torque - limits.torque,
                    "temperature": temperature,
                }
        return kind, motor, bytes(frame.data)

    def drain(self):
        # Inspect queued fault frames, rather than silently discarding them.
        for _ in range(512):
            frame = self.bus.recv(timeout=0)
            if frame is None:
                return
            self.inspect(frame)
        raise RuntimeError(f"{self.channel}: receive queue overloaded.")

    def transaction(self, motor_id, expected_kind, send, parameter=None, expected_mode=None):
        self.drain()
        sent_at = time.time()
        send()
        deadline = time.monotonic() + self.timeout
        while (remaining := deadline - time.monotonic()) > 0:
            frame = self.bus.recv(timeout=remaining)
            if frame is None:
                break
            parsed = self.inspect(frame)
            if parsed is None or frame.timestamp < sent_at - 0.001:
                continue
            kind, source, data = parsed
            if kind != expected_kind or source != motor_id:
                continue
            # A reply to the preceding preload can still be in flight after
            # drain(). Wait for the requested state, rather than treating the
            # first matching motor/status frame as the enable acknowledgement.
            if expected_mode is not None and self.states.get(motor_id) != expected_mode:
                continue
            if parameter is not None and struct.unpack_from("<H", data)[0] != parameter:
                continue
            return data
        detail = (f" in mode {expected_mode}; last observed mode={self.states.get(motor_id)}"
                  if expected_mode is not None else "")
        raise TimeoutError(f"{self.channel} motor {motor_id}: missing fresh type {expected_kind} reply{detail}.")

    def scan(self):
        found = {}
        for motor in MOTOR_IDS:
            found[motor] = self.transaction(motor, 0, lambda: self.send(0, motor)).hex()
        return found

    def read_parameter(self, motor, parameter, fmt):
        data = self.transaction(motor, 17, lambda: self.send(
            17, motor, struct.pack("<HHI", parameter, 0, 0)), parameter)
        return struct.unpack_from("<" + fmt, data, 4)[0]

    def write_parameter(self, motor, parameter, value, fmt):
        packed = struct.pack("<" + fmt, value).ljust(4, b"\0")
        self.drain()
        self.send(18, motor, struct.pack("<HH", parameter, 0) + packed)
        actual = self.read_parameter(motor, parameter, fmt)
        if actual != value:
            raise RuntimeError(f"{self.channel} motor {motor}: parameter 0x{parameter:04x} "
                               f"readback {actual}, expected {value}.")

    def positions(self):
        positions = [self.read_parameter(motor, POSITION, "f") for motor in self.motor_ids]
        if any(not math.isfinite(q) or abs(q) > 4 * math.pi for q in positions):
            raise ValueError(f"{self.channel}: invalid mechanical positions.")
        return positions

    def prepare(self):
        # Never stop motors or switch a running motor's mode automatically.
        modes = {motor: self.read_parameter(motor, MODE, "b") for motor in self.motor_ids}
        if any(mode != 0 for mode in modes.values()):
            raise ValueError(f"{self.channel}: motors must already be in MIT mode. "
                             "Use rebot_disable.py --prepare-mit manually while supported.")
        for motor in self.motor_ids:
            # Explicit user policy: a normal host exit must not reset the motors.
            # Native position hold is installed before closing an active bus.
            self.write_parameter(motor, WATCHDOG, 0, "I")

    def set_hold(self, positions, gravity, kp, kd):
        if any(len(values) != len(self.motor_ids) for values in (positions, gravity, kp, kd)):
            raise ValueError("Native hold requires one position, torque, and gains per controlled motor.")
        records = {i: (float(q), float(tau), float(p), float(d))
                   for i, (q, tau, p, d) in enumerate(zip(positions, gravity, kp, kd), 1)}
        for motor, values in records.items():
            if values[2] <= 0 or values[3] <= 0:
                raise ValueError("Native hold gains must be positive.")
            self.encode(motor, *values)
        self.hold_records = records

    def enable(self, positions):
        if len(positions) != len(self.motor_ids):
            raise ValueError("Expected one position per controlled motor before enabling.")
        if set(self.hold_records) != set(self.motor_ids):
            raise ValueError("Configure native position hold before enabling motors.")
        frames = [self.encode(i, q, self.hold_records[i][1],
                              self.hold_records[i][2], self.hold_records[i][3])
                  for i, q in enumerate(positions, 1)]
        for motor, frame in enumerate(frames, 1):
            # The preceding process may have left this motor actively holding.
            # Preload full support and adopt a confirmed running motor without
            # sending a redundant enable command or ever sending zero support.
            self.enabled.add(motor)
            try:
                self.transaction(motor, 2, lambda: self.bus.send(frame, timeout=0.01),
                                 expected_mode=2)
            except TimeoutError:
                pass
            else:
                print(f"{self.channel} motor {motor}: running hold adopted (mode=2).", flush=True)
                continue

            def enable_and_hold():
                self.send(3, motor)
                # The documented operation-mode sequence produces feedback in
                # response to the MIT frame after enable. Do not require the
                # enable frame alone to produce a running-state reply.
                self.bus.send(frame, timeout=0.01)

            self.transaction(motor, 2, enable_and_hold, expected_mode=2)
            print(f"{self.channel} motor {motor}: enable confirmed (mode=2).", flush=True)

    def command(self, positions, torques, deadline=None):
        if len(positions) != len(self.motor_ids) or len(torques) != len(self.motor_ids):
            raise ValueError("Expected one position and torque per controlled motor.")
        frames = [self.encode(i, q, tau)
                  for i, (q, tau) in enumerate(zip(positions, torques), 1)]

        def send_fresh(frame):
            remaining = 0.01 if deadline is None else deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"{self.channel}: command feedback deadline expired.")
            self.bus.send(frame, timeout=min(0.01, remaining))

        for motor, frame in enumerate(frames, 1):
            self.transaction(motor, 2, lambda: send_fresh(frame), expected_mode=2)

    def command_position(self, positions, velocities, gravity, kp, kd, deadline):
        """Send native MIT position/velocity/PD commands as one selected-motor batch."""
        if any(len(values) != len(self.motor_ids) for values in (positions, velocities, gravity, kp, kd)):
            raise ValueError("Expected native targets, gains, and feedforward per controlled motor.")
        frames = [self.encode(i, q, tau, p, d, velocity=v)
                  for i, (q, v, tau, p, d) in enumerate(zip(positions, velocities, gravity, kp, kd), 1)]
        self.drain()
        sent_at = time.time()
        # Do not wait for each axis before updating the next axis.
        for frame in frames:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"{self.channel}: motion feedback deadline expired.")
            self.bus.send(frame, timeout=min(.01, remaining))
        pending = set(self.motor_ids)
        reply_deadline = min(deadline, time.monotonic() + self.timeout)
        while pending and (remaining := reply_deadline - time.monotonic()) > 0:
            frame = self.bus.recv(timeout=remaining)
            if frame is None:
                break
            parsed = self.inspect(frame)
            if parsed is None or frame.timestamp < sent_at - .001:
                continue
            kind, motor, _ = parsed
            if kind == 2 and motor in pending and self.states.get(motor) == 2:
                pending.remove(motor)
        if pending:
            raise TimeoutError(f"{self.channel}: missing native motion feedback from motors {sorted(pending)}.")
        return [decode_mit(frame, model) for frame, model in zip(frames, self.models)]

    def freeze(self):
        """Leave enabled motors holding position; never send a disable command."""
        if not self.enabled:
            return
        records = dict(self.hold_records)
        try:
            positions = self.positions()
            gravity = self.hold_gravity(positions) if self.hold_gravity else None
            candidate = dict(records)
            for motor in self.enabled:
                old = records[motor]
                candidate[motor] = (positions[motor - 1],
                                    float(gravity[motor - 1]) if gravity is not None else old[1],
                                    old[2], old[3])
                self.encode(motor, *candidate[motor])
            records = candidate
        except Exception as exc:
            print(f"{self.channel}: hold uses last validated pose ({exc}).", file=sys.stderr, flush=True)
        frames = {motor: self.encode(motor, *records[motor])
                  for motor in sorted(self.enabled)}
        # First send hold to all motors, then spend time checking acknowledgements.
        for motor, frame in frames.items():
            try:
                self.bus.send(frame, timeout=0.01)
            except Exception as exc:
                print(f"{self.channel} motor {motor}: hold send failed: {exc}", file=sys.stderr, flush=True)
        for motor, frame in frames.items():
            try:
                self.transaction(motor, 2, lambda: self.bus.send(frame, timeout=0.01), expected_mode=2)
            except Exception as exc:
                print(f"{self.channel} motor {motor}: hold not verified: {exc}", file=sys.stderr, flush=True)
            else:
                print(f"{self.channel} motor {motor}: native position hold confirmed; enabled.", flush=True)
