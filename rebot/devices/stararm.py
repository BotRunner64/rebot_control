"""Star Arm leader serial access: discovery, fresh angle reads, and torque release."""

from contextlib import contextmanager
import math
from pathlib import Path
import statistics
import time

import serial
from fashionstar_uart_sdk import UartServoManager

BAUDRATE = 1_000_000
SERVO_IDS = tuple(range(7))              # J1-J6 and the gripper handle


class AngleFeedbackError(ValueError):
    """A monitor request did not return valid, fresh angles."""


@contextmanager
def open_leader(port, baudrate=BAUDRATE):
    """Exclusive serial connection wrapped in the vendor servo manager."""
    with serial.Serial(port, baudrate=baudrate, timeout=0, exclusive=True) as uart:
        yield UartServoManager(uart)


def scan_port(port, baudrate=BAUDRATE):
    with open_leader(port, baudrate) as bus:
        online = []
        for device_id in range(8):
            bus.uart.reset_input_buffer()
            if bus.ping(device_id):
                online.append(device_id)
    return online


def stable_port(port):
    """Prefer the /dev/serial/by-path name, which survives ttyUSB renumbering."""
    paths = [p for p in Path("/dev/serial/by-path").glob("*")
             if p.resolve() == Path(port).resolve()]
    return str(min(paths, key=lambda p: ("-usbv" in str(p), len(str(p))))) if paths else port


def read_angles(bus, servo_ids=SERVO_IDS):
    """Require fresh, checksum-validated monitor data (degrees) for the selected servos."""
    servo_ids = list(servo_ids)
    bus.uart.reset_input_buffer()
    bus.pkt_buffer.empty_buffer()
    for servo_id in servo_ids:
        bus.servos[servo_id].angle_monitor = None
    states = bus.send_sync_servo_monitor(servo_ids, realtime=True)
    angles = [getattr(states.get(i), "angle_monitor", None) for i in servo_ids]
    invalid = [i for i, value in zip(servo_ids, angles)
               if value is None or not math.isfinite(value) or abs(value) > 368640]
    if invalid:
        raise AngleFeedbackError(f"Missing or invalid angle feedback from servo IDs {invalid}.")
    return angles


def release_torque(bus):
    # Use the official broadcast stop instead of a burst of per-servo packets.
    bus.stop_on_control_mode(0xff, 0x10, 0)
    bus.uart.flush()
    time.sleep(.01)


def angle_difference(current, reference):
    # Shortest signed difference for circular calibration and hover comparisons.
    return [(a - b + 180) % 360 - 180 for a, b in zip(current, reference)]


def capture_pose(bus):
    """Median of ten still samples of all seven servo angles."""
    import numpy as np

    samples = []
    for _ in range(10):
        samples.append(read_angles(bus))
        time.sleep(.02)
    changes = np.array([angle_difference(sample, samples[0]) for sample in samples])
    if np.max(np.ptp(changes, axis=0)) > 2:
        raise ValueError("The arm moved during capture. Hold it still and retry.")
    return (np.array(samples[0]) + np.median(changes, axis=0)).tolist()


def capture_gripper(bus):
    """Median closed-handle angle from twenty valid samples within thirty attempts."""
    samples = []
    failures = 0
    last_error = None
    for _ in range(30):
        try:
            samples.append(read_angles(bus, (6,))[0])
        except AngleFeedbackError as error:
            failures += 1
            last_error = error
        time.sleep(.02)
        if len(samples) == 20:
            break
    if len(samples) != 20:
        raise ValueError(f"Collected only {len(samples)}/20 valid gripper samples in 30 attempts; "
                         "calibration was not saved. Check the leader connection and retry.") from last_error
    if max(samples) - min(samples) > .2 + 1e-9:
        raise ValueError("The gripper moved during capture. Hold it closed and retry.")
    if failures:
        print(f"Gripper capture recovered from {failures} invalid feedback sample(s).")
    return statistics.median(samples)
