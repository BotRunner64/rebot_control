"""Picklable virtual hardware factories for real spawn-process tests."""

from dataclasses import dataclass
from contextlib import redirect_stdout, redirect_stderr
import io
import os
import struct
import time
from unittest.mock import patch

import rebot_mit as mit
from test_rebot_hand_guiding import VirtualMotors, reply


class MeasuredMotors(VirtualMotors):
    def __init__(self, offset=0):
        super().__init__(offset)
        self.params[(7, mit.WATCHDOG)] = 4000
        self.params[(7, mit.MODE)] = 0
        self.modes[7] = 0

    def send(self, msg, timeout):
        queued = len(self.queue)
        super().send(msg, timeout)
        kind, motor = (msg.arbitration_id >> 24) & 31, msg.arbitration_id & 255
        if kind in (1, 3, 4) and len(self.queue) > queued:
            bound = mit.LIMITS['rs-06' if motor <= 3 else 'rs-00'].position
            code = int((self.position + bound) / (2 * bound) * 65535)
            self.queue[-1] = reply(2, motor, struct.pack('>HHHH', code, 32767, 32767, 250),
                                   mode=self.modes[motor])


class RecordingBus(mit.MotorBus):
    def __enter__(self):
        self.stdout = redirect_stdout(io.StringIO())
        self.stderr = redirect_stderr(io.StringIO())
        self.stdout.__enter__()
        self.stderr.__enter__()
        return super().__enter__()

    def command_position(self, positions, velocities, gravity, kp, kd, deadline):
        if self.channel == 'can1' and self.calls == 3:
            if self.factory.failure == 'crash':
                os._exit(23)
            if self.factory.failure == 'command':
                self.bus.drop_holds.add(7)
        try:
            result = super().command_position(positions, velocities, gravity, kp, kd, deadline)
        finally:
            self.bus.drop_holds.clear()
        self.calls += 1
        with self.factory.counts.get_lock():
            self.factory.counts[self.index] += 1
        if self.first is None:
            self.first = list(positions)
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
                'calls': self.calls, 'first': self.first, 'parameters': self.last_parameters,
                'modes': self.bus.modes, 'closed': self.bus.closed,
                'enabled': sorted(self.enabled), 'position_queries': len(queries),
                'types': sorted({(m.arbitration_id >> 24) & 31 for m in self.bus.sent}),
                'tau_zero': all((m.arbitration_id >> 8) & 65535 == 32767 for m in frames),
                'velocity_zero': all(struct.unpack('>HHHH', m.data)[1] == 32767 for m in frames),
                'hold': [mit.decode_mit(m, self.models[(m.arbitration_id & 255)-1])
                         for m in frames[-len(self.motor_ids):]]})
            self.stderr.__exit__(None, None, None)
            self.stdout.__exit__(None, None, None)


@dataclass
class BusFactory:
    reports: object
    counts: object
    failure: str = ''

    def __call__(self, channel, models, **kwargs):
        virtual = MeasuredMotors(0 if channel == 'can0' else 100)
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
        bus.calls, bus.first, bus.last_parameters = 0, None, None
        return bus


@dataclass
class SourceFactory:
    reports: object
    stall: bool = False

    def __call__(self, arm):
        return FakeSource(arm, self)


class FakeSource:
    def __init__(self, arm, factory):
        self.arm, self.factory = arm, factory
        self.reads, self.released = 0, False

    def __enter__(self):
        return self

    def read(self):
        self.reads += 1
        time.sleep(.002)
        if self.factory.stall and self.arm['port'].endswith('right') and self.released:
            time.sleep(.15)
        return [30.] * 7

    def release(self):
        self.released = True

    def __exit__(self, *args):
        self.factory.reports.put({'kind': 'source', 'port': self.arm['port'], 'pid': os.getpid(),
                                  'released': self.released, 'reads': self.reads, 'closed': True})
