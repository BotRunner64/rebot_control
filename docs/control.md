# 控制功能

所有会驱动电机的命令遵守同一套规则，先读这一节。

## 共同行为

**启动**（每只臂都一样）：打开 CAN → 核验 7 个电机 UID → 读当前姿态并检查限位 → 确认电机在 MIT 模式、关闭看门狗 → 再读一次姿态 → 使能。双臂时，两臂都通过检查后才使能任何一只。运行即使能，没有额外的“使能开关”。

**停止**：`空格` / `Esc` / `Q` / `Ctrl-C`（或 SIGTERM/SIGHUP）。停止、报错、到时退出走同一条路：把电机切到**当前位置的电机侧位置保持**，保持使能后关闭 CAN。不归位、不失能。以终端逐电机打印的 `native position hold confirmed` 为准。

**保护**：

- 电机上报故障 → 立即停止。
- 反馈缺失或过期 → 停止。
- 任一 J1–J6 的目标与实测相差超过 `max_tracking_error_deg`（默认 45°）→ 拒绝发送并停止。阈值以内直接发送，**没有限速、插值或避障**。
- 实测姿态超出关节限位（容差 0.1 rad）→ 停止。

**关节限位**（度，相对软件零位）：J1 ±145，J2 0–170，J3 0–200，J4 −80–90，J5 ±90，J6 ±130，夹爪电机 0–270。

**位置控制增益**（电机侧 MIT PD）：`kp=[50,150,150,50,50,50]`，`kd=[3,10,10,5,4,4]`。

参数在 `config/control.yaml`，可用 `--settings` 换文件。

---

## 拖动示教 `rebot guide`

手拖机械臂时提供重力补偿和阻尼，松手静止后自动保持位置。只控制 J1–J6，夹爪不动。

```bash
rebot guide --side right          # left / right / both，默认 right
rebot guide --side both --duration 60
```

状态：`STARTUP`（前 2 秒保持初始姿态）→ `HOLD`（保持）⇄ `DRAG`（被推动后跟随，静止 0.5 秒后回到保持）。

- 重力模型来自 `models/rebot_rs_dynamics.urdf`，假定底座水平、竖直安装、原配末端、空载。换了末端或负载后补偿会偏。
- `--gravity-scale 0.9` 整体减小补偿；`--calibration <yaml>` 使用 `rebot identify` 的辨识结果。
- 总力矩（重力 + 保持 + 阻尼）按 `torque_limit_nm` 限幅；重力本身超限会直接报错而不是悄悄少补。
- 先单臂、空载、有人托稳地测试。

## 归位并卸力 `rebot home`

```bash
rebot home                              # 默认双臂；--side left|right 只归位一侧
rebot home --speed 0.1                  # 峰值关节速度 rad/s，默认 0.3
```

J1–J6 沿最小 jerk 轨迹回到软件零位。全部所选臂误差连续 0.5 秒小于 0.05 rad 后，**失能该臂全部 7 个电机（含夹爪）**。

- 运行前在参考姿态处准备好支撑：失能后电机不再出力。
- 关节空间直线插值，不避障，先确认路径畅通。
- 中途停止或出错：保持当前姿态，**跳过失能**，返回码 130。
- 这是全仓库唯一会发失能指令的命令。
- 每次运行的逐周期日志写到 `runs/motion/`。

## 遥操作 `rebot teleop`

Star Arm 主手控制实体从臂和夹爪。不开重力补偿，不启动 3D 视图。

```bash
rebot teleop                              # 双臂 + 夹爪
rebot teleop --side right
rebot teleop --side right --no-gripper    # 只控制 J1–J6
rebot teleop --gripper-open-deg 180 --gripper-torque-nm 0.3
```

启动前把主从臂摆到接近的对应姿态，托稳主手（程序会释放主手锁力）。

**映射**：`从臂目标 = radians(主手角度 − 主手零位) × [+1,+1,−1,−1,−1,+1]`，裁剪到关节限位。主手越过限位时从臂停在限位端。

**夹爪**：主手手柄 45° 行程映射到从臂夹爪电机 0–270°（`--gripper-open-deg` 可缩小）。夹爪力矩由上位机计算并限幅，电机侧增益为零：

```text
tau = clip(kp·(目标 − 实测) − kd·速度, ±torque_limit_nm)
```

闭合末段（最后 5° 电机行程）平滑加入 1° 的闭合预压，让夹爪合紧。参数在 `config/gripper.yaml`：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `torque_limit_nm` | 1.0 | 跟随时的总力矩上限（不是指尖力） |
| `kp` / `kd` | 12 / 0.5 | 上位机 PD 增益（基于电机角度） |
| `leader_close_tolerance_deg` | 1.0 | 主手离闭合位这么近就算完全闭合 |
| `close_preload_deg` | 1.0 | 完全闭合时的预压角；0 关闭 |
| `close_preload_window_deg` | 5.0 | 预压渐入的行程 |

注意：启动和退出时夹爪用电机侧位置保持，**不受** `torque_limit_nm` 约束。

**进程结构**：每个设备一个进程，互不阻塞。

| 进程 | 频率 | 工作 |
| --- | --- | --- |
| `leader-<side>` | 串口实际速度 | 读主手 7 路角度 |
| `target` | 500 Hz | 映射并发布一对完整目标 |
| `motor-<side>` | 200 Hz | 取最新目标发给从臂，收反馈 |
| 主进程 | ~100 Hz | 分阶段启动、监控、处理停止 |

**容错**：主手读取失败最多重试 15 次，期间从臂保持最后目标；从臂某轴没回复时只重发该轴，每轮等 10 ms，最多 15 次。任一进程异常退出，其余全部停止并保持。主进程失联 0.5 秒，工作进程自行停止并保持。

退出时打印各进程的实测频率和最大延迟。

## 末端位姿控制 `rebot ee`

单臂，在浏览器里拖动末端目标，Mink IK 求解后直接下发。默认是运动学仿真（不碰硬件），加 `--hardware` 才上实机。只控制 J1–J6。

```bash
rebot ee                                          # 仿真，自动打开浏览器
rebot ee --hardware --side right                  # 实机右臂
rebot ee --headless --offset 0.01 0 0 --duration 2            # 仿真：沿基座 +X 移动 1 cm
rebot ee --hardware --headless --offset 0.01 0 0 --duration 2 # 实机同上
rebot ee --pose 0.30 0.07 0.65 0 -30 20           # 绝对位姿：x y z(m) roll pitch yaw(deg)
```

浏览器操作：

| 操作 | 方式 |
| --- | --- |
| 平移 / 旋转 | `T` / `R`，拖动操作柄；按住 Shift 为 1 mm / 1° 步进 |
| 输入数值 | `Set pose`（mm、deg） |
| 放弃目标，停在当前位置 | `Hold` |
| 结束并保持 | `Stop & hold` 或终端 `Ctrl-C` |
| 重置视角 | `F` |

- 目标在**该臂基座坐标系**下。末端坐标系是夹爪安装面中心 `gripper_mount`：+X 接近方向，+Z 掌背方向（定义见 [数据格式](data-format.md#末端坐标系)）。`--tcp-offset x y z` 在末端坐标系下加偏移。
- 姿态欧拉角为固定轴 XYZ：`R = Rz(yaw)·Ry(pitch)·Rx(roll)`。
- 实机带重力前馈（按实测姿态计算）。
- 目标会导致自碰撞（穿透 > 1 mm）时不下发，保留上一个目标，页面有提示。**不检查桌面和环境**。
- 不可达目标会停在 IK 的最近似解，不会自动退出。
- 关闭或刷新页面不会停止控制，可重连。
- `--headless` 必须同时给目标和 `--duration`；到时误差 ≤ 2 mm 且 ≤ 2° 返回 0，否则返回 2。
- 目标循环 50 Hz，CAN 线程 250 Hz，互不等待。

## 远端策略控制 `rebot policy`

把相机图像和双臂状态发给策略服务，按 30 Hz 执行返回的绝对末端目标。只有实机模式：运行即使能双臂和夹爪。

```bash
rebot policy --host 127.0.0.1 --port 8000 \
  --instruction "训练时的任务原文" \
  --camera top=/dev/video4 --camera left_wrist=/dev/video0 --camera right_wrist=/dev/video2
```

- 相机名必须恰好是服务端声明需要的那几路。
- 声明自己是 mock 的策略服务会被拒绝。
- 每 16 个控制周期（`--replan-steps`）在后台请求下一段动作，等待时继续执行当前段，过期的目标按时间戳跳过。
- 请求超时（`--request-timeout`，默认 1 s）、相机过期（`--camera-max-age` 0.25 s）、相机与状态时间差过大（`--camera-max-skew` 0.10 s）、非法动作、CAN 故障都会停止并保持。
- `--duration 10` 限制运行时长。`--gripper-open-deg` 应与采集时一致（默认 270）。
- 消息格式和如何接入新策略见 [策略链路协议](policy-link.md)。

## 重力辨识 `rebot identify`

自动让单臂走一组固定姿态，拟合各连杆质量比例，修正重力模型。

```bash
rebot identify --side right
```

流程：采样 24 个姿态（每个姿态正反两个方向各到一次，抵消摩擦）→ 拟合 → 再采 12 个独立姿态验证 → 通过后写出 `runs/calibration/<side>/<时间>/calibration.yaml`。之后可 `rebot guide --calibration <该文件>`。

- 要求：夹爪闭合空载、无外部支撑、周围留空。固定的肘上扫掠，不避障。全程约十几分钟。
- 辨识结果与该臂的电机 UID 和软件零位绑定，换臂或重新标定零位后失效。
- 拟合或验证不达标会给出原因并拒绝，不产生可用配置。
- 力矩来自位置环指令的估算，不是力矩传感器。
