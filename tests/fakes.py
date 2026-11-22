"""Shared test doubles: an in-process RobStride firmware model and binding helpers."""

from contextlib import ExitStack
from pathlib import Path
import struct
import tempfile
import time
from unittest.mock import patch

import can
import numpy as np

from rebot import bindings, paths
from rebot.bindings import ArmBinding
from rebot.devices import robstride
from rebot.devices.robstride import MODELS


def isolate_config(test):
    """Give virtual arms their own directory for machine-local gravity profiles."""
    directory = tempfile.TemporaryDirectory()
    test.addCleanup(directory.cleanup)
    root = Path(directory.name)
    config = patch.object(paths, "CONFIG", root)
    config.start()
    test.addCleanup(config.stop)
    return root


def reply(kind, motor, data=bytes(8), mode=0):
    host = 0xFE if kind == 0 else 0xFD
    return can.Message(arbitration_id=(kind << 24) | (mode << 22) | (motor << 8) | host,
                       is_extended_id=True, data=data, timestamp=time.time())


class VirtualMotors:
    """In-process firmware model; never opens SocketCAN or drives hardware."""

    def __init__(self, offset=0, fail_enable=None):
        self.queue, self.sent = [], []
        self.params = {(i, robstride.WATCHDOG): 4000 for i in range(1, 7)}
        self.params.update({(i, robstride.MODE): 0 for i in range(1, 7)})
        self.modes = dict.fromkeys(range(1, 7), 0)
        self.uids = {i: (offset + i).to_bytes(8, "little").hex() for i in range(1, 8)}
        self.fail_enable = fail_enable
        self.drop_stops = set()
        self.drop_holds = set()
        self.position = .2
        self.temperature = 250
        self.closed = False

    def recv(self, timeout):
        return self.queue.pop(0) if self.queue else None

    def send(self, msg, timeout):
        self.sent.append(msg)
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if kind == 0:
            self.queue.append(reply(0, motor, bytes.fromhex(self.uids[motor])))
        elif kind == 17:
            parameter = struct.unpack_from("<H", msg.data)[0]
            if parameter == robstride.POSITION:
                value = struct.pack("<f", self.position)
            else:
                fmt = "b" if parameter == robstride.MODE else "I"
                value = struct.pack("<" + fmt, self.params[(motor, parameter)]).ljust(4, b"\0")
            self.queue.append(reply(17, motor, struct.pack("<HH", parameter, 0) + value))
        elif kind == 18:
            parameter = struct.unpack_from("<H", msg.data)[0]
            fmt = "b" if parameter == robstride.MODE else "I"
            self.params[(motor, parameter)] = struct.unpack_from("<" + fmt, msg.data, 4)[0]
        elif kind in (1, 3, 4):
            if kind == 1 and motor in self.drop_holds:
                return
            if kind == 3:
                if motor == self.fail_enable:
                    return
                self.modes[motor] = 2
            elif kind == 4:
                if motor in self.drop_stops:
                    return
                self.modes[motor] = 0
            self.queue.append(reply(2, motor, struct.pack(">HHHH", 32767, 32767, 32767, self.temperature),
                                    mode=self.modes[motor]))
        else:
            raise AssertionError(f"Unexpected command type: {kind}")

    def shutdown(self):
        self.closed = True


def arm_bindings(virtual, reference=(0.,) * 7):
    """Calibrated `ArmBinding`s for {side: (channel, VirtualMotors)}."""
    return {side: ArmBinding(side, channel, MODELS, dict(motors.uids), np.array(reference, dtype=float))
            for side, (channel, motors) in virtual.items()}


def patch_hardware(test, virtual, selected=None, modules=()):
    """Route SocketCAN to `{channel: VirtualMotors}` for one test.

    `selected` ({side: ArmBinding}) replaces `select_arms` in each of `modules`
    (the modules that imported it by name).
    """
    stack = ExitStack()
    test.addCleanup(stack.close)
    stack.enter_context(patch.object(robstride, "check_channel"))
    stack.enter_context(patch.object(robstride.can, "Bus",
                                     side_effect=lambda **kw: virtual[kw["channel"]]))

    def select(path=None, side="both", *, calibrated=True):
        return {name: selected[name] for name in bindings.select_sides(side)}

    for module in modules:
        stack.enter_context(patch.object(module, "select_arms", side_effect=select))
    return stack
