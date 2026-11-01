"""Manual control ownership, bounded trajectories and feedback checks."""
from collections import deque
import json
import math
from pathlib import Path
import struct
import threading
import time

from .protocol import SocketCAN, SimulatedCAN, status_description


ROOT = Path(__file__).resolve().parents[1]


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{label}必须是有效数字。")
    return float(value)


def load_config(path=None):
    config = json.loads(Path(path or ROOT / "config.json").read_text())
    assert [j["id"] for j in config["joints"]] == list(range(1, 8)), "需要依序配置 J1～J7"
    for joint in config["joints"]:
        assert joint["model"] in ("rs-00", "rs-06")
        assert -720 <= joint["min_deg"] < joint["max_deg"] <= 720
        kpmax, kdmax = (5000, 100) if joint["model"] == "rs-06" else (500, 5)
        assert 0 < joint["kp"] <= kpmax and 0 < joint["kd"] <= kdmax
        assert len(joint["identity"]) == 16
    for key, low, high in [("control_hz", 50, 200), ("max_speed_deg_s", 0.1, 30),
                           ("max_step_deg", 0.1, 10), ("acceleration_deg_s2", 1, 90),
                           ("lease_seconds", 0.3, 2), ("feedback_timeout_seconds", 0.1, 1),
                           ("max_following_error_deg", 1, 10), ("max_temperature_c", 30, 80)]:
        assert low <= number(config[key], key) <= high, f"{key} 超出范围"
    return config


class Controller:
    def __init__(self, config, simulate=False, transport_factory=None, audit_path=None, start_worker=True):
        self.config, self.simulate = config, simulate
        self.factory = transport_factory or (SimulatedCAN if simulate else SocketCAN)
        self.transport = None
        self.owner = None
        self.last_heartbeat = 0.0
        self.lease_expired = False
        self.control_revision = 0
        self.resetting = False
        self.reset_deadline = 0.0
        self.reset_error = ""
        self.lock = threading.RLock()
        self.action_lock = threading.RLock()
        self.joints = {j["id"]: dict(enabled=False, arming=False, sent=None, target=None,
                                      velocity=0.0, speed=15.0, fault="", identity_ok=False, last_enable_status=None)
                       for j in config["joints"]}
        self.events = deque(maxlen=80)
        self.audit_path = Path(audit_path or ROOT / "logs" / "actions.jsonl")
        self.closed = threading.Event()
        self.event("服务已就绪。等待你在页面点击连接；尚未打开 CAN。")
        self.worker = None
        if start_worker:
            self.worker = threading.Thread(target=self._run, daemon=True, name="manual-control")
            self.worker.start()

    def event(self, message, **details):
        with self.lock:
            row = {"time": time.time(), "message": message, "simulate": self.simulate, **details}
            self.events.appendleft(row)
            self.audit_path.parent.mkdir(parents=True, exist_ok=True)
            with self.audit_path.open("a") as f:
                f.write(json.dumps(row, ensure_ascii=False) + "\n")

    def _claim(self, client, allow_new=False):
        if not isinstance(client, str) or not 8 <= len(client) <= 80:
            raise ValueError("页面会话无效，请刷新。")
        with self.lock:
            if self.owner != client:
                if not allow_new or any(j["enabled"] or j["arming"] for j in self.joints.values()):
                    raise ValueError("当前控制属于另一个页面。请先在原页面失能，或使用接管并保持。")
                self.owner = client
            self.last_heartbeat = time.monotonic()
            self.lease_expired = False

    def heartbeat(self, client):
        self._claim(client)

    def connect(self, client):
        with self.action_lock:
            self._claim(client, allow_new=True)
            if self.transport:
                return
            bus = self.factory(self.config)
            try:
                bus.open()
                positions = {}
                for joint in self.config["joints"]:
                    i = joint["id"]
                    if bus.ping(i) != joint["identity"]:
                        raise ValueError(f"J{i} 身份与校零记录不一致，拒绝控制。")
                    positions[i] = bus.read(i, 0x7019)
                    bus.read(i, 0x701C)
                with self.lock:
                    for i, position in positions.items():
                        self.joints[i].update(sent=position, target=position, identity_ok=True)
                    self.transport = bus
                    self.last_heartbeat = time.monotonic()
                self.event("已连接，7 个电机身份已核对。全部等待手动使能。", action="connect", client=client)
            except Exception:
                bus.close()
                raise

    def takeover(self, client):
        if not isinstance(client, str) or not 8 <= len(client) <= 80:
            raise ValueError("页面会话无效。")
        with self.action_lock, self.lock:
            if self.owner != client and time.monotonic() - self.last_heartbeat < self.config["lease_seconds"]:
                raise ValueError("原控制页面仍在线，请先在原页面停止。")
            self._freeze()
            self.owner = client
            self.last_heartbeat = time.monotonic()
            self.lease_expired = False
            self.event("手动接管，所有目标已冻结在最后发送位置。", action="takeover", client=client)

    def _joint(self, value):
        if isinstance(value, bool) or not isinstance(value, int) or value not in self.joints:
            raise ValueError("关节编号必须是 1～7 的整数。")
        if not self.transport:
            raise ValueError("请先点击连接机械臂。")
        return self.config["joints"][value - 1], self.joints[value]

    def enable(self, client, motor):
        with self.action_lock:
            self._claim(client)
            with self.lock:
                config, joint = self._joint(motor)
                if joint["enabled"]:
                    raise ValueError("该关节已使能。")
                joint["arming"] = True
                bus = self.transport
            try:
                # Explicit user enable: reset this joint only, select mode while disabled,
                # and initialize its hold target from a fresh measured position.
                disabled = bus.set_enabled(motor, False)
                if disabled.get("mode_state") != 0:
                    raise ValueError(f"J{motor} 未进入失能状态：{status_description(disabled)}")
                position = bus.read(motor, 0x7019)
                degrees = math.degrees(position)
                if not config["min_deg"] - 0.5 <= degrees <= config["max_deg"] + 0.5:
                    raise ValueError("实际角度超出配置范围，请核对零点和机械方向。")
                if bus.read(motor, 0x7005, "b") != 0:
                    bus.send(18, motor, struct.pack("<H2xb3x", 0x7005, 0))
                    if bus.read(motor, 0x7005, "b") != 0:
                        raise ValueError("无法切换 MIT 模式。")
                if disabled.get("fault_bits", 0) or disabled.get("fault_raw", 0):
                    raise ValueError("电机报告故障，请排查后重试。")
                if disabled.get("temperature", 0) >= self.config["max_temperature_c"]:
                    raise ValueError("电机温度超限。")
                bus.hold(config, position)
                active = bus.enable_hold(config, position)
                with self.lock:
                    joint["last_enable_status"] = dict(active)
                if active.get("mode_state") != 2 or active.get("fault_bits", 0) or active.get("fault_raw", 0):
                    raise ValueError("电机未进入正常运行状态：" + status_description(active))
                with self.lock:
                    joint.update(enabled=True, sent=position, target=position, velocity=0.0, fault="")
                    self.last_heartbeat = time.monotonic()
                self.event(f"J{motor} 手动使能，保持当前位置 {degrees:.2f}°。", action="enable", joint=motor, client=client)
            except Exception as exc:
                failure_status = bus.snapshot(motor)
                with self.lock:
                    joint["last_enable_status"] = dict(failure_status)
                    joint["enabled"] = False
                try:
                    bus.set_enabled(motor, False)
                except Exception:
                    with self.lock:
                        joint["fault"] = "失能未确认，电机可能仍有力矩"
                self.event(f"J{motor} 使能失败：{exc}", action="enable_failed", joint=motor, client=client,
                           feedback=failure_status)
                raise
            finally:
                with self.lock:
                    joint["arming"] = False

    def enable_all(self, client):
        with self.action_lock:
            self._claim(client)
            for motor in self.joints:
                with self.lock:
                    self._joint(motor)
                    already_enabled = self.joints[motor]["enabled"]
                if already_enabled:
                    continue
                try:
                    self.enable(client, motor)
                except Exception as exc:
                    with self.lock:
                        enabled = [i for i,j in self.joints.items() if j["enabled"]]
                    raise ValueError(f"整臂使能未完成，J{motor}：{exc}。已使能 {enabled}，未自动撤去这些关节的力矩。") from exc
            self.event("手动整臂使能完成，7 个关节保持各自当前位置。", action="enable_all", client=client)

    def execute(self, client, motor, target_deg, speed, *, direct=False, revision=None):
        target_deg, speed = number(target_deg, "角度"), number(speed, "速度")
        with self.action_lock, self.lock:
            self._claim(client)
            if direct and (isinstance(revision, bool) or not isinstance(revision, int) or revision != self.control_revision):
                raise ValueError("操作已停止或状态已改变，请重新拖动。")
            config, joint = self._joint(motor)
            if self.resetting:
                raise ValueError("正在复位，请先停止再拖动。")
            if not joint["enabled"] or joint["arming"]:
                raise ValueError("请先手动使能这个关节。")
            if joint["fault"]:
                raise ValueError(joint["fault"] + "；请排查后失能并重新使能。")
            sample = self.transport.snapshot(motor)
            reason = self._feedback_problem(config, joint, sample, time.monotonic())
            if reason:
                self._freeze(joint)
                joint["fault"] = reason
                raise ValueError(reason)
            if not config["min_deg"] <= target_deg <= config["max_deg"]:
                raise ValueError(f"目标必须位于 {config['min_deg']}°～{config['max_deg']}°。")
            if not 0.1 <= speed <= self.config["max_speed_deg_s"]:
                raise ValueError(f"速度必须位于 0.1～{self.config['max_speed_deg_s']}°/s。")
            measured = math.degrees(sample["position"])
            if not direct and max(abs(target_deg - measured), abs(target_deg - math.degrees(joint["sent"]))) > self.config["max_step_deg"] + 0.05:
                raise ValueError(f"每次最多移动 {self.config['max_step_deg']}°，请分步执行。")
            for i, other in self.joints.items():
                if i != motor and other["enabled"] and abs(other["target"] - other["sent"]) > math.radians(0.05):
                    raise ValueError("已有另一个关节在运动，请先停止或等待完成。")
            joint.update(target=math.radians(target_deg), speed=speed)
            self.event(f"J{motor} 手动{'拖动' if direct else '执行'} → {target_deg:.2f}°，{speed:.1f}°/s。", action="drag" if direct else "execute", joint=motor,
                       target_deg=target_deg, speed_deg_s=speed, client=client)

    def reset_all(self, client, speed, revision):
        speed = number(speed, "速度")
        with self.action_lock, self.lock:
            self._claim(client)
            if isinstance(revision, bool) or not isinstance(revision, int) or revision != self.control_revision:
                raise ValueError("操作已停止或状态已改变，请重新点击复位。")
            if not 0.1 <= speed <= self.config["max_speed_deg_s"]:
                raise ValueError("复位速度超出允许范围。")
            if self.resetting:
                raise ValueError("正在复位。")
            # Validate every joint before changing any target. This operation
            # never enables motors or writes their calibrated zero points.
            now = time.monotonic()
            for config in self.config["joints"]:
                i = config["id"]
                _, joint = self._joint(i)
                if not joint["enabled"] or joint["arming"]:
                    raise ValueError("请先全部使能，再复位。")
                if not config["min_deg"] <= 0 <= config["max_deg"]:
                    raise ValueError(f"J{i} 配置范围不包含零位。")
                reason = joint["fault"] or self._feedback_problem(config, joint, self.transport.snapshot(i), now)
                if reason:
                    raise ValueError(f"J{i} 无法复位：{reason}")
            self._freeze()
            travel = max(abs(math.degrees(j["sent"])) for j in self.joints.values())
            for joint in self.joints.values():
                joint.update(target=0.0, speed=speed)
            self.resetting = True
            self.reset_deadline = now + travel / speed + 10
            self.event(f"手动复位：全部关节返回 0°，限速 {speed:.1f}°/s，完成后保持使能。",
                       action="reset", speed_deg_s=speed, client=client)

    def _freeze(self, joint=None):
        self.control_revision += 1
        self.resetting = False
        if joint is None:
            self.reset_error = ""
        for state in [joint] if joint is not None else self.joints.values():
            state["target"] = state["sent"]
            state["velocity"] = 0.0

    def stop(self, client):
        with self.action_lock, self.lock:
            self._claim(client)
            self._freeze()
            self.event("手动停止：冻结最后发送位置，已使能关节继续保持。", action="stop", client=client)

    def disable(self, client, motor=None):
        with self.action_lock:
            self._claim(client)
            with self.lock:
                if motor is not None:
                    self._joint(motor)
                if not self.transport:
                    raise ValueError("尚未连接。")
                ids = [motor] if motor is not None else list(self.joints)
                # Mark first so the worker cannot race a disable frame with a new MIT target.
                for i in ids:
                    self._freeze(self.joints[i])
                    self.joints[i]["enabled"] = False
            failures = []
            for i in ids:
                try:
                    feedback = self.transport.set_enabled(i, False)
                    if feedback.get("mode_state") != 0:
                        raise ValueError("未确认失能")
                    with self.lock:
                        self.joints[i]["fault"] = ""
                except Exception as exc:
                    with self.lock:
                        self.joints[i]["fault"] = "失能未确认，电机可能仍有力矩"
                    failures.append(f"J{i}: {exc}")
            self.event("手动失能 " + (f"J{motor}" if motor else "全部关节") + "。", action="disable", client=client)
            if failures:
                raise ValueError("；".join(failures))

    def disconnect(self, client):
        with self.action_lock, self.lock:
            self._claim(client)
            if any(j["enabled"] or j["arming"] or "失能未确认" in j["fault"] for j in self.joints.values()):
                raise ValueError("请先手动失能全部关节并确认，再断开连接。")
            if self.transport:
                self.transport.close()
                self.transport = None
            self.event("手动断开连接。", action="disconnect", client=client)

    def _feedback_problem(self, config, joint, sample, now):
        if now - sample.get("status_time", 0) > self.config["feedback_timeout_seconds"]:
            return "运行反馈超时，目标已冻结；保持力矩无法保证"
        if sample.get("fault_bits", 0) or sample.get("fault_raw", 0):
            return "电机报告故障，目标已冻结"
        if sample.get("mode_state") != 2:
            return "电机已退出运行状态，目标已冻结"
        if sample.get("temperature", 0) >= self.config["max_temperature_c"]:
            return "温度超过调试阈值，目标已冻结"
        position = sample.get("position")
        if position is None or not math.isfinite(position):
            return "角度读数无效，目标已冻结"
        degrees = math.degrees(position)
        if not config["min_deg"] - 0.5 <= degrees <= config["max_deg"] + 0.5:
            return "实际角度超出配置范围，目标已冻结"
        if abs(math.degrees(position - joint["sent"])) > self.config["max_following_error_deg"]:
            return "实际位置与指令偏差过大，目标已冻结"
        return ""

    def tick(self, dt, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.transport:
                return
            if self.resetting and now > self.reset_deadline:
                self._freeze()
                self.reset_error = "复位到位验证超时，已停止推进目标并保持最后指令。"
                self.event(self.reset_error, action="reset_failed")
            if now - self.last_heartbeat > self.config["lease_seconds"]:
                if not self.lease_expired:
                    self._freeze()
                    self.lease_expired = True
                    self.event("页面心跳中断：停止推进目标，保留最后保持指令。", action="lease_stop")
            for config in self.config["joints"]:
                joint = self.joints[config["id"]]
                if not joint["enabled"] or joint["arming"]:
                    continue
                previous_position = joint["sent"]
                reason = self._feedback_problem(config, joint, self.transport.snapshot(config["id"]), now)
                if reason and not joint["fault"]:
                    joint["fault"] = reason
                    if self.resetting:
                        self._freeze()
                        self.reset_error = f"复位中止，J{config['id']}：{reason}"
                    else:
                        self._freeze(joint)
                    self.event(f"J{config['id']}：{reason}", action="feedback_stop")
                if not joint["fault"]:
                    delta = math.degrees(joint["target"] - joint["sent"])
                    acceleration = self.config["acceleration_deg_s2"]
                    desired = math.copysign(min(joint["speed"], math.sqrt(2 * acceleration * abs(delta))), delta)
                    change = max(-acceleration * dt, min(acceleration * dt, desired - joint["velocity"]))
                    joint["velocity"] += change
                    step = joint["velocity"] * dt
                    if abs(step) >= abs(delta):
                        joint["sent"], joint["velocity"] = joint["target"], 0.0
                    else:
                        joint["sent"] += math.radians(step)
                    joint["sent"] = max(math.radians(config["min_deg"]), min(math.radians(config["max_deg"]), joint["sent"]))
                try:
                    # MIT's velocity reference follows the position trajectory.
                    # Keeping it at zero while moving adds opposing damping.
                    velocity = (joint["sent"] - previous_position) / dt if dt > 0 else 0.0
                    self.transport.hold(config, joint["sent"], velocity)
                except Exception as exc:
                    if not joint["fault"]:
                        joint["fault"] = "CAN 发送失败，目标已冻结"
                        if self.resetting:
                            self._freeze()
                            self.reset_error = f"复位中止，J{config['id']} CAN 发送失败。"
                        else:
                            self._freeze(joint)
                        self.event(f"J{config['id']} CAN 发送失败：{exc}", action="can_error")
            if self.resetting and all(
                abs(math.degrees(j["sent"])) <= 0.05 and
                abs(math.degrees(self.transport.snapshot(i).get("position", math.inf))) <= 1.0
                for i,j in self.joints.items()
            ):
                self.resetting = False
                self.event("复位完成，全部关节在零位附近并保持使能。", action="reset_complete")

    def _run(self):
        period = 1 / self.config["control_hz"]
        previous, next_poll, poll_index = time.monotonic(), 0.0, 0
        next_tick = previous + period
        while not self.closed.wait(max(0, next_tick - time.monotonic())):
            now = time.monotonic()
            self.tick(min(now - previous, 0.05), now)
            previous = now
            with self.lock:
                if self.transport and now >= next_poll:
                    try:
                        self.transport.poll(poll_index % 7 + 1)
                    except Exception:
                        pass  # Per-joint feedback timeouts are surfaced by tick/snapshot.
                    poll_index += 1
                    next_poll = now + 0.1
            next_tick += period
            if next_tick < time.monotonic():
                next_tick = time.monotonic() + period

    def snapshot(self, client=None):
        with self.lock:
            now = time.monotonic()
            rows = []
            for config in self.config["joints"]:
                state = self.joints[config["id"]]
                sample = self.transport.snapshot(config["id"]) if self.transport else {}
                def degrees(value):
                    return math.degrees(value) if value is not None and math.isfinite(value) else None
                age = now - sample.get("position_time", 0)
                rows.append({**{k: config[k] for k in ("id", "name", "model", "min_deg", "max_deg")},
                             "enabled": state["enabled"], "arming": state["arming"], "fault": state["fault"],
                             "identity_ok": state["identity_ok"], "online": age < 2,
                             "last_enable_status": state["last_enable_status"],
                             "feedback_age_s": round(age, 2) if sample else None,
                             "actual_deg": degrees(sample.get("position")), "target_deg": degrees(state["target"]),
                             "command_deg": degrees(state["sent"]), "temperature": sample.get("temperature"),
                             "voltage": sample.get("voltage"),
                             "moving": state["enabled"] and abs(state["target"] - state["sent"]) > math.radians(0.05)})
            return {"connected": self.transport is not None, "simulate": self.simulate,
                    "channel": self.config["channel"], "owner": self.owner == client,
                    "has_owner": self.owner is not None, "lease_expired": self.lease_expired,
                    "control_revision": self.control_revision,
                    "resetting": self.resetting, "reset_error": self.reset_error,
                    "can_takeover": self.owner != client and now - self.last_heartbeat >= self.config["lease_seconds"],
                    "max_step_deg": self.config["max_step_deg"], "max_speed_deg_s": self.config["max_speed_deg_s"],
                    "tx_count": self.transport.tx_count if self.transport else 0,
                    "joints": rows, "events": list(self.events)}

    def close(self):
        self.closed.set()
        if self.worker:
            self.worker.join(timeout=1)
        with self.lock:
            if self.transport:
                self.transport.close()
