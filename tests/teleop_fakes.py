"""Picklable virtual hardware factories for real spawn-process tests."""

from dataclasses import dataclass
from contextlib import redirect_stdout, redirect_stderr
import io
import os
import signal
import struct
import time
from unittest.mock import patch

import numpy as np

from fakes import VirtualMotors, reply
from rebot.bindings import ArmBinding, LeaderBinding, select_sides
from rebot.devices import robstride as mit
from rebot.devices.robstride import MODELS
from rebot.devices.stararm import AngleFeedbackError
from rebot.runtime.console import handle_signals

CHANNELS = {"left": "can0", "right": "can1"}
UID_OFFSETS = {"left": 0, "right": 100}


def virtual_bindings(side="both"):
    """Calibrated follower and leader bindings matching `BusFactory`/`SourceFactory`."""
    arms = {name: ArmBinding(name, CHANNELS[name], MODELS, dict(MeasuredMotors(UID_OFFSETS[name]).uids),
                             np.full(7, .2)) for name in select_sides(side)}
    leaders = {name: LeaderBinding(name, f"fake-{name}", 1000000, (10.,) * 7) for name in arms}
    return arms, leaders


def signal_while_stop_lock_is_held(stop, result):
    """Reproduce a signal interrupting the shared stop event's critical section."""
    handle_signals(stop)
    with stop._event._cond:
        for sig in (signal.SIGTERM, signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            os.kill(os.getpid(), sig)
    result.send(stop.is_set())
    result.close()


def wait_for_signal(stop, result):
    handle_signals(stop)
    result.send('ready')
    result.send(stop.wait(60.))
    result.close()


class MeasuredMotors(VirtualMotors):
    def __init__(self, offset=0):
        super().__init__(offset)
        self.params[(7, mit.WATCHDOG)] = 4000
        self.params[(7, mit.MODE)] = 0
        self.modes[7] = 0
        self.wait_for_timeout = False
        self.drop_remaining = {}

    def recv(self, timeout):
        frame = super().recv(timeout)
        if frame is None and timeout > 0 and self.wait_for_timeout:
            time.sleep(timeout)
        return frame

    def send(self, msg, timeout):
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if kind == 1 and self.drop_remaining.get(motor, 0):
            self.drop_remaining[motor] -= 1
            self.sent.append(msg)
            return
        queued = len(self.queue)
        super().send(msg, timeout)
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if kind in (1, 3, 4) and len(self.queue) > queued:
            bound = mit.LIMITS['rs-06' if motor <= 3 else 'rs-00'].position
            code = int((self.position + bound) / (2 * bound) * 65535)
            self.queue[-1] = reply(2, motor, struct.pack('>HHHH', code, 32767, 32767, 250),
                                   mode=self.modes[motor])


class RecordingBus(mit.MotorBus):
    def capture_output(self):
        # `Arm` owns the bus without entering it, so the factory starts the capture.
        self.stdout = redirect_stdout(io.StringIO())
        self.stderr = redirect_stderr(io.StringIO())
        self.stdout.__enter__()
        self.stderr.__enter__()

    def command_position(self, positions, velocities, gravity, kp, kd, deadline, **kwargs):
        if self.channel == 'can1' and self.calls == 3:
            if self.factory.failure == 'crash':
                os._exit(23)
            if self.factory.failure == 'command':
                self.bus.drop_holds.add(7)
            if self.factory.failure == 'transient_command':
                self.bus.drop_remaining[7] = 14
        try:
            result = super().command_position(positions, velocities, gravity, kp, kd, deadline, **kwargs)
        finally:
            self.bus.drop_holds.clear()
        if self.factory.failure == 'gripper_feedback' and self.calls == 3:
            self.feedback[7]['torque'] = float('nan')
        self.calls += 1
        with self.factory.counts.get_lock():
            self.factory.counts[self.index] += 1
        if self.first is None:
            self.first = list(positions)
        elif self.second is None:
            self.second = list(positions)
        if result is not None:      # None: a stop request interrupted this batch
            self.last_parameters = result
        return result

    def __exit__(self, *args):
        try:
            return super().__exit__(*args)
        finally:
            frames = [m for m in self.bus.sent if (m.arbitration_id >> 24) & 31 == 1]
            queries = [m for m in self.bus.sent if (m.arbitration_id >> 24) & 31 == 17
                       and struct.unpack_from('<H', m.data)[0] == mit.POSITION]
            self.factory.reports.put({'kind': 'motor', 'channel': self.channel, 'pid': os.getpid(),
                'calls': self.calls, 'first': self.first, 'second': self.second,
                'parameters': self.last_parameters,
                'modes': self.bus.modes, 'closed': self.bus.closed,
                'enabled': sorted(self.enabled), 'position_queries': len(queries),
                'types': sorted({(m.arbitration_id >> 24) & 31 for m in self.bus.sent}),
                'arm_tau_zero': all((m.arbitration_id >> 8) & 65535 == 32767
                                    for m in frames if m.arbitration_id & 255 != 7),
                'velocity_zero': all(struct.unpack('>HHHH', m.data)[1] == 32767 for m in frames),
                'hold': [mit.decode_mit(m, self.models[(m.arbitration_id & 255)-1])
                         for m in frames[-len(self.motor_ids):]],
                'gripper_torques': [mit.decode_mit(m, 'rs-00')['gravity']
                                    for m in frames if m.arbitration_id & 255 == 7
                                    and struct.unpack('>HHHH', m.data)[2:] == (0, 0)],
                'gripper_commands': [mit.decode_mit(m, 'rs-00')['position']
                                     for m in frames if m.arbitration_id & 255 == 7]})
            self.stderr.__exit__(None, None, None)
            self.stdout.__exit__(None, None, None)


@dataclass
class BusFactory:
    reports: object
    counts: object
    failure: str = ''
    grasp: str = ''
    wait_for_timeout: bool = False

    def __call__(self, channel, models, **kwargs):
        motor_class = GraspingMotors if self.grasp else MeasuredMotors
        virtual = motor_class(0 if channel == 'can0' else 100)
        virtual.wait_for_timeout = self.wait_for_timeout
        if channel == 'can1':
            if self.failure == 'uid':
                virtual.uids[7] = 'ff' * 8
            elif self.failure == 'prepare':
                virtual.params[(7, mit.MODE)] = 1
            elif self.failure == 'enable':
                virtual.fail_enable = 3
        with patch.object(mit, 'check_channel'), patch.object(mit.can, 'Bus', return_value=virtual):
            bus = RecordingBus(channel, models, **kwargs)
        bus.factory = self
        bus.index = 0 if channel == 'can0' else 1
        bus.calls, bus.first, bus.second, bus.last_parameters = 0, None, None, None
        bus.capture_output()
        return bus


@dataclass
class SourceFactory:
    reports: object
    stall: bool = False
    grasp: str = ''
    counts: object = None
    failure: str = ''

    def __call__(self, leader):
        return FakeSource(leader, self)


class FakeSource:
    def __init__(self, leader, factory):
        self.leader, self.factory = leader, factory
        self.reads, self.released = 0, False
        self.failures = 0

    def __enter__(self):
        return self

    def read(self):
        self.reads += 1
        time.sleep(.002)
        if self.factory.stall and self.leader.port.endswith('right') and self.released:
            time.sleep(.15)
        if self.released and self.leader.port.endswith('right'):
            if (self.factory.failure == 'leader_feedback'
                    or self.factory.failure == 'leader_transient' and self.failures < 14):
                self.failures += 1
                time.sleep(.01)
                raise AngleFeedbackError('Missing or invalid angle feedback from servo IDs [3, 4, 5, 6].')
        finger = 30.
        if self.factory.grasp:
            finger = 55. if self.factory.grasp == 'open' and sum(self.factory.counts) >= 100 else 10.
        if self.factory.failure == 'tracking_jump':
            return [90.] + [30.] * 5 + [finger]
        return [30.] * 6 + [finger]

    def release(self):
        self.released = True

    def __exit__(self, *args):
        self.factory.reports.put({'kind': 'source', 'port': self.leader.port, 'pid': os.getpid(),
                                  'released': self.released, 'reads': self.reads,
                                  'failures': self.failures, 'closed': True})


class GraspingMotors(MeasuredMotors):
    """A blocked gripper reporting native PD plus feedforward torque."""

    def send(self, msg, timeout):
        queued = len(self.queue)
        super().send(msg, timeout)
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if motor != 7 or len(self.queue) <= queued:
            return
        position = 1.2
        if kind == 17 and struct.unpack_from('<H', msg.data)[0] == mit.POSITION:
            self.queue[-1] = reply(17, motor, struct.pack('<HHf', mit.POSITION, 0, position))
        elif kind in (1, 3, 4):
            command = mit.decode_mit(msg, 'rs-00') if kind == 1 else {
                'kp': 0., 'position': position, 'gravity': 0.}
            torque = max(-14., min(14., command['kp'] * (command['position'] - position)
                                   + command['gravity']))
            bound = mit.LIMITS['rs-00'].position
            q_code = int((position + bound) / (2 * bound) * 65535)
            tau_code = int((torque + 14.) / 28. * 65535)
            self.queue[-1] = reply(2, 7, struct.pack('>HHHH', q_code, 32767, tau_code, 250),
                                   mode=self.modes[7])
