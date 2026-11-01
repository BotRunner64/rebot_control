"""RobStride private CAN codec and transport. Construction never opens CAN.

Reference: https://github.com/motorbridge/motorbridge/tree/main/motor_vendors/robstride/src
The allowlist intentionally excludes set-zero, set-ID, save and protocol changes.
"""
import json
import math
import select
import socket
import struct
import subprocess
import threading
import time


PMAX = 4 * math.pi


def status_description(status):
    mode = status.get("mode_state")
    names = {0: "失能/复位", 1: "标定", 2: "运行", 3: "保留状态"}
    bits, raw = status.get("fault_bits", 0), status.get("fault_raw", 0)
    flags = [name for bit, name in enumerate(("欠压", "过流", "过温", "磁编码器故障", "堵转", "编码器未标定")) if bits & (1 << bit)]
    frame = status.get("frame_id", "未知")
    return f"状态={names.get(mode, '未知')}({mode})，故障位=0x{bits:02X}，故障码=0x{raw:08X}，回包={frame}" + ("，" + "/".join(flags) if flags else "")


def mit_payload(model, position, kp, kd, velocity=0.0):
    kpmax, kdmax = (5000, 100) if model == "rs-06" else (500, 5)
    def signed(value, maximum):
        return int((max(-maximum, min(maximum, value)) / maximum + 1) * 32767)
    data = struct.pack(
        ">HHHH", signed(position, PMAX), signed(velocity, 50 if model == "rs-06" else 33),
        int(kp / kpmax * 65535), int(kd / kdmax * 65535),
    )
    # Feedforward torque = 0; extra field is torque in type-1 frames, NOT host ID.
    return 32767, data


class SocketCAN:
    def __init__(self, config):
        self.config = config
        self.sock = None
        self.samples = {}
        self.replies = {}
        self.condition = threading.Condition()
        self.send_lock = threading.Lock()
        self.request_lock = threading.Lock()
        self.running = False
        self.error = ""
        self.tx_count = 0

    def open(self):
        info = json.loads(subprocess.check_output(
            ["ip", "-json", "-details", "link", "show", "dev", self.config["channel"]],
            text=True,
        ))[0]
        if ("UP" not in info.get("flags", []) or
            info.get("linkinfo", {}).get("info_data", {}).get("bittiming", {}).get("bitrate") != 1000000):
            raise RuntimeError("CAN 接口未以 1 Mbps 启动，请先配置接口。")
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.settimeout(0.04)
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            self.sock.bind((self.config["channel"],))
        except Exception:
            self.sock.close()
            self.sock = None
            raise
        self.running = True
        self.receiver = threading.Thread(target=self._receive, daemon=True, name="can-feedback")
        self.receiver.start()

    def send(self, kind, motor, data=bytes(8), extra=None):
        if kind not in (0, 1, 3, 4, 17, 18) or not 1 <= motor <= 7:
            raise ValueError("该 CAN 操作不在 WebUI 允许范围内。")
        if kind == 18 and data != struct.pack("<H2xb3x", 0x7005, 0):
            raise ValueError("仅允许在手动使能时选择 MIT 模式。")
        if kind in (0, 3, 4) and data != bytes(8):
            raise ValueError("无效控制帧。")
        extra = self.config["host_id"] if extra is None else extra
        canid = (kind << 24) | (extra << 8) | motor
        with self.send_lock:
            if self.sock is None:
                raise RuntimeError("尚未连接 CAN。")
            self.sock.send(struct.pack("=IB3x8s", socket.CAN_EFF_FLAG | canid, 8, data))
            self.tx_count += 1

    def _receive(self):
        while self.running:
            try:
                if not select.select([self.sock], [], [], 0.05)[0]:
                    continue
                raw = self.sock.recv(16)
                if len(raw) != 16:
                    continue
                canid, dlc, data = struct.unpack("=IB3x8s", raw)
                if not canid & socket.CAN_EFF_FLAG or canid & (socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG) or dlc != 8:
                    continue
                kind, extra = (canid >> 24) & 31, (canid >> 8) & 65535
                motor, now = extra & 255, time.monotonic()
                if not 1 <= motor <= 7:
                    continue
                with self.condition:
                    if kind == 0:
                        self.replies[(motor, "ping")] = (now, data.hex())
                    elif kind == 17:
                        param = struct.unpack_from("<H", data)[0]
                        self.replies[(motor, param)] = (now, (extra >> 8, data[4:]))
                        if extra >> 8 == 0 and param in (0x7019, 0x701C):
                            name = "position" if param == 0x7019 else "voltage"
                            value = struct.unpack("<f", data[4:])[0]
                            if math.isfinite(value):
                                self.samples.setdefault(motor, {}).update({name: value, name + "_time": now})
                    elif kind in (2, 24):
                        p, v, torque, temperature = struct.unpack(">HHHH", data)
                        model = self.config["joints"][motor - 1]["model"]
                        self.samples.setdefault(motor, {}).update(
                            position=(p / 32767 - 1) * PMAX, position_time=now,
                            velocity=(v / 32767 - 1) * (50 if model == "rs-06" else 33),
                            temperature=temperature / 10, mode_state=(extra >> 14) & 3,
                            fault_bits=(extra >> 8) & 63, status_time=now,
                            frame_id=f"0x{canid & socket.CAN_EFF_MASK:08X}", frame_type=kind,
                        )
                        self.replies[(motor, "status")] = (now, dict(self.samples[motor]))
                    elif kind == 21:
                        fault, warning = struct.unpack("<II", data)
                        self.samples.setdefault(motor, {}).update(fault_raw=fault, warning_raw=warning)
                    self.condition.notify_all()
            except (OSError, ValueError) as exc:
                if self.running:
                    self.error = str(exc)
                    time.sleep(0.05)

    def _request(self, kind, motor, key, data=bytes(8), timeout=0.5,
                 accept=None, on_wait=None, operation="查询"):
        with self.request_lock:
            started = time.monotonic()
            self.send(kind, motor, data)
            next_hold = started
            last_status = None
            with self.condition:
                while True:
                    reply = self.replies.get((motor, key))
                    if reply and reply[0] >= started:
                        if accept is None or accept(reply[1]):
                            return reply[1]
                        last_status = reply[1]
                    remaining = timeout - (time.monotonic() - started)
                    if remaining <= 0:
                        if isinstance(last_status, dict):
                            raise RuntimeError(f"J{motor} {operation}未确认：{status_description(last_status)}")
                        raise RuntimeError(f"J{motor} {operation}无回应，请检查供电和 CAN。")
                    # Some firmware reports Reset on ENABLE and enters Run only
                    # after an operation-control frame. Repeat only the freshly
                    # measured, unchanging hold target during this manual action.
                    if on_wait and time.monotonic() >= next_hold:
                        on_wait()
                        next_hold = time.monotonic() + 0.01
                    self.condition.wait(min(remaining, 0.01 if on_wait else remaining))

    def ping(self, motor):
        return self._request(0, motor, "ping")

    def read(self, motor, param, fmt="f"):
        status, data = self._request(17, motor, param, struct.pack("<H6x", param))
        if status:
            raise RuntimeError(f"J{motor} 参数 0x{param:04X} 读取失败，状态 {status}。")
        value = struct.unpack_from("<" + fmt, data)[0]
        if not math.isfinite(value):
            raise RuntimeError(f"J{motor} 返回了无效读数。")
        return value

    def set_enabled(self, motor, enabled):
        expected = 2 if enabled else 0
        return self._request(3 if enabled else 4, motor, "status",
                             accept=lambda s: s.get("mode_state") == expected or bool(s.get("fault_bits") or s.get("fault_raw")),
                             operation="使能" if enabled else "失能")

    def enable_hold(self, joint, position):
        return self._request(3, joint["id"], "status", timeout=0.6,
                             accept=lambda s: s.get("mode_state") == 2 or bool(s.get("fault_bits") or s.get("fault_raw")),
                             on_wait=lambda: self.hold(joint, position), operation="使能并保持当前位置")

    def snapshot(self, motor):
        with self.condition:
            return dict(self.samples.get(motor, {}))

    def poll(self, motor):
        self.send(17, motor, struct.pack("<H6x", 0x7019))
        self.send(17, motor, struct.pack("<H6x", 0x701C))

    def hold(self, joint, position, velocity=0.0):
        extra, data = mit_payload(joint["model"], position, joint["kp"], joint["kd"], velocity)
        self.send(1, joint["id"], data, extra)

    def close(self):
        # Never send disable/home/zero implicitly when a socket or server closes.
        self.running = False
        if self.sock:
            self.receiver.join(timeout=0.2)
            self.sock.close()
            self.sock = None


class SimulatedCAN:
    """Deterministic software motor used only when --simulate is explicit."""
    def __init__(self, config):
        self.config = config
        self.opened = False
        self.tx_count = 0
        self.sent = []
        self.velocity_commands = []
        self.error = ""
        self.positions = {i: 0.0 for i in range(1, 8)}
        self.active = {i: False for i in range(1, 8)}
        self.faults = {i: 0 for i in range(1, 8)}
        self.stale = False

    def open(self):
        self.opened = True

    def _record(self, kind, motor, value=None):
        if not self.opened:
            raise RuntimeError("模拟连接尚未打开。")
        self.sent.append((kind, motor, value))
        self.tx_count += 1

    def ping(self, motor):
        self._record(0, motor)
        return self.config["joints"][motor - 1]["identity"]

    def read(self, motor, param, fmt="f"):
        self._record(17, motor, param)
        return {0x7019: self.positions[motor], 0x701C: 48.0, 0x7005: 0}[param]

    def send(self, kind, motor, data=bytes(8), extra=None):
        self._record(kind, motor, data.hex())

    def set_enabled(self, motor, enabled):
        self._record(3 if enabled else 4, motor)
        self.active[motor] = enabled
        return self.snapshot(motor)

    def enable_hold(self, joint, position):
        self.set_enabled(joint["id"], True)
        self.hold(joint, position)
        return self.snapshot(joint["id"])

    def snapshot(self, motor):
        now = time.monotonic() - (10 if self.stale else 0)
        return dict(position=self.positions[motor], position_time=now, voltage=48.0,
                    temperature=28.0, mode_state=2 if self.active[motor] else 0,
                    fault_bits=self.faults[motor], fault_raw=0, status_time=now)

    def poll(self, motor):
        self._record(17, motor, 0x7019)

    def hold(self, joint, position, velocity=0.0):
        self._record(1, joint["id"], position)
        self.velocity_commands.append((joint["id"], velocity))
        if self.active[joint["id"]]:
            self.positions[joint["id"]] = position

    def close(self):
        self.opened = False
