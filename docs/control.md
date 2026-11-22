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

**关节限位**（度，相对软件零位）：J1 ±145，J2 0–170，J3 0–200，J4 −80–90，J5 ±90，J6 ±130，夹爪电机 0–270。所有控制器（含 identify/home/guide 的重力模型）共用这一套，读数允许超出 0.1 rad（J2/J3 静止时压在零位硬限位上会略为负）。URDF 里的限位来自 Seeed 仿真描述，不作为真实行程使用；URDF 文件本身不改，以免改变绑定在计划和标定文件里的模型哈希。

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
- `--gravity-scale 0.9` 整体减小补偿。启动时自动读取所选臂的本机重力标定，`--calibration <yaml>` 可临时指定其他辨识结果。
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

Star Arm 主手控制实体从臂和夹爪。`teleop` 和 `collect` 都根据从臂 J1–J6 的实测角度计算重力前馈，叠加到电机的位置控制上。退出保持也使用该臂的重力模型。

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
- 使用拧瓶盖 RTC 服务时，指令传 `Unscrew the bottle cap`。控制端自动读取服务端的 `rtc_prefix_steps`，首块用空前缀，后续把推理期间要执行的旧目标随观测发送。默认承诺 3 步（100 ms），收到回复后在承诺期结束时切换，晚到时按时间戳跳过过期目标。
- 每 16 个控制周期（`--replan-steps`）在后台请求下一段动作，等待时继续执行当前段，过期的目标按时间戳跳过。
- 请求超时（`--request-timeout`，默认 1 s）、相机过期（`--camera-max-age` 0.25 s）、相机与状态时间差过大（`--camera-max-skew` 0.10 s）、非法动作、CAN 故障都会停止并保持。
- `--duration 10` 限制运行时长。`--gripper-open-deg` 应与采集时一致（默认 270）。
- `--record datasets/cap_debug` 把这次运行录成一个 episode，方便排查，见下。

### 录制运行过程 `--record`

```bash
rebot policy --port 8000 --instruction "Unscrew the bottle cap" \
  --camera top=4 --camera left_wrist=0 --camera right_wrist=2 \
  --record datasets/cap_debug
```

输出与 `rebot collect` [格式相同](data-format.md)，每次运行往数据集追加一个 episode，可以直接用 `rebot process` 回放、切分并导出。

- **录的是策略看到的东西**：视频由策略相机的同一批图像按采集时刻对齐到 30 fps，状态是实测的末端位姿和夹爪开合度（按 `--gripper-open-deg` 归一化）。
- **另外带一份调试数据**，在 episode 的 `debug/` 目录下，见下表。它不属于[数据格式](data-format.md)，切分导出时跳过该目录；`rebot collect` 的采集数据里没有它。
- 录制不改变策略的输入；录制进程就绪后才使能电机。
- 结束后要封装和校验视频，**看到 `Saved episode ...` 才算保存成功**。此时双臂已处于保持状态。
- 控制中途出错停止（推理超时、动作用尽等）时，这段录制**照样保存**，再报告原始错误。
- 录制本身出错（写盘、编码跟不上）会停止控制并保持，数据留在 `<dataset>/.pending/`。
- 消息格式和如何接入新策略见 [策略链路协议](policy-link.md)。

#### 调试数据 `debug/`

| 文件 | 频率 | 内容 |
| --- | --- | --- |
| `left_joints.parquet`、`right_joints.parquet` | 每个伺服周期（250 Hz） | 7 个电机的实测 `position`、`velocity`、`torque`，以及这一周期下发的关节目标 `target` 和力矩 `feedforward` |
| `targets.parquet` | 每个控制周期（30 Hz） | 交给机械臂的末端目标 `left_pose`/`right_pose` 和夹爪目标 `left_openness`/`right_openness`，它来自第几个动作块的第几行（`chunk`、`row`），以及是否真的下发（`sent`） |
| `chunks.parquet` | 每次推理 | 完整的动作块：四路流各自的目标和相对观测时刻的时间偏移，观测时刻、推理耗时、收到时已过期的行数 `first_row` |
| `run.json` | 每次运行 | 命令行参数、策略服务的元数据、`control.yaml` 和 `gripper.yaml` 的内容、关节增益、时序统计、停止原因或错误 |

- 所有 `*timestamp_ns` 以同步帧 0 为原点，帧 0 之前的诊断样本时间为负数。
- 关节角是相对软件零位的弧度，夹爪（第 7 个）是电机角；位姿和开合度的定义与[数据格式](data-format.md)相同。
- `torque` 是电机上报的力矩。`feedforward` 对前 6 轴是重力前馈，电机在此之上再加自己的 PD 项；对夹爪是上位机算出的全部力矩（已限幅）。
- `targets` 减去同一时刻的 `state` 就是末端跟踪误差。`sent` 为假表示目标会自碰撞而被拒绝，机械臂保持上一个目标。
- `chunks` 里第 `chunk` 行的 `left_eef[row]` 就是 `targets` 里那一行的 `left_pose`；每个动作块只有 `first_row` 之后、下一块到来之前的那几行被执行。
- 策略还没返回第一个动作块之前不下发目标，`targets` 也没有行。

## 重力辨识 `rebot identify`

夹爪闭合、空载，清空肘部和腕部的扫掠空间。运行下面的命令会驱动所选机械臂。采集中按空格、Esc、Q 或 Ctrl-C 会停止并保持当前位置。

```bash
rebot identify --side right
```

默认峰值关节速度为 0.8 rad/s，可用 `--speed` 调整。启动时显示当前采样计划的预计时长，实际用时还取决于各姿态的稳定等待时间。

左右臂都在 J1 = 0°、朝工作区正前方执行固定轨迹。每轮的目标、顺序、正反向接近和中间过渡都预先确定。从开机姿态进入统一起点的 `entry` 段随实测关节角变化，只检查关节限位和重力矩预算。启动前请确认这段路径畅通。

几何碰撞预检从进入第一个 `transit` 或 `sample` 路点的线段开始，覆盖两套固定扫掠轨迹，每轮使能前再次复核。检查使用工作站模型，沿关节路径以最大 0.5° 步长采样。若非相邻连杆、另一只臂、桌面或相机支架的模型相互穿入超过 1 mm，程序会报告路点和部件。双臂工位应绑定两只臂，另一只臂在采样期间保持静止。仅绑定一只臂时按单臂布局检查。

第一轮在不同肩、肘和腕部姿态下扫 J2–J5。24 个目标各从两个方向到达，取稳定窗口的中位数，再按方向求均值。16 对用于拟合 J2、J3、J4、J6 对应连杆的质量倍率，8 对用于留出验证。J1、J5 沿用 CAD 质量，各连杆质心沿用 CAD 值。日志同时记录原生位置环估算力矩和驱动器回报力矩。

拟合完成后直接保存结果目录下的 `calibration.yaml` 和 `calibrated.urdf`。只有拟合本身不可用时才不保存：姿态激励不足、求解失败或质量倍率碰到 0.5/2 边界。留出误差超过 `--max-rmse`（默认 0.25 N·m）、验证误差变差或整体改善不足只作为警告打印，并写入报告和配置的 `warnings`。线缆拉扯、齿槽等非重力力矩无法由质量倍率解释，会在这里表现为某个关节的警告。最终效果以 `rebot guide` 拖动和松手后的漂移为准。

需要额外的实机核对时加 `--verify`：

```bash
rebot identify --side right --verify
```

`--verify` 在另外 12 个目标上用默认前馈测一轮，检查新参数的预测误差，再用新参数前馈实测同一批目标。两轮采用相同位置环增益。报告中的 `hold_comparison` 比较被扫关节两个方向分别需要的额外 PD 力矩，同时列出所有保持姿态上的六轴 RMS，并记录从轨迹到点至采样结束，末端相对目标高度的最大下沉（毫米）。这是位置保持下的测量。每轴力矩 RMS 上限为 0.25 N·m，允许的单轴恶化为 0.02 N·m，整体要求降至基线的 90% 或 0.05 N·m 以内。未达标的项同样只记为警告，不删除配置；全部达标时配置中 `verified: true`。

将要使用的辨识结果复制到对应臂的配置文件。下面以右臂为例，请把路径中的“本次运行”换成实际目录名。左臂使用 `config/gravity.left.local.yaml`。

```bash
cp runs/calibration/right/本次运行/calibration.yaml config/gravity.right.local.yaml
rebot guide --side right
```

`guide`、`home`、`teleop`、`collect`、实机 `ee` 和实机 `policy` 每次启动时按臂读取配置。某侧文件缺失时，该臂使用 CAD 模型和 `control.yaml` 的 `gravity_scale`。加载成功时使用辨识的连杆质量倍率，逐轴 `gravity_scale` 取 1。文件内容无效或与所选臂的电机 UID、软件零位、原始模型不匹配时，启动报错。`guide` 和 `home` 的 `--calibration` 优先于本机文件。重新运行 `identify` 时仍以 CAD 模型为基线。

### 用已知配重核对力矩

首次调试或更换末端工具后，可对同一组姿态做空载和已知配重的差分测量。两条命令分开运行。记录好配重及固定件的总质量和质心，空载轮结束后，在支撑好机械臂、能安全安装时固定配重。第二轮使用与第一轮相同的工具、机械零位、速度和控制设置。将下面的变量替换为实测值，质量单位为 kg，质心坐标为 URDF 的 **link6 坐标系**下的米。两次输出均使用新目录。

```bash
rebot identify payload-empty --side right --output runs/payload-empty
rebot identify payload-loaded --side right --empty runs/payload-empty --output runs/payload-loaded \
  --mass-kg "$MASS_KG" --com-m "$COM_X" "$COM_Y" "$COM_Z"
```

两轮使用相同的 CAD 前馈；带载轮额外按已知配重检查路径的重力矩预算。碰撞检查使用标准夹爪外形，额外配重、固定件和现场物体的空间需单独确认。`payload_report.json` 列出每个姿态的理论力矩增量、估算增量和驱动回报增量，以及各轴的误差和相对理论值的增益。受力明显的轴上，增益应接近 +1；理论增量 RMS 小于 0.05 N·m 的轴显示 `null`。优先核对肩肘的符号和尺度，再决定使用哪种力矩信号。此诊断独立于空载辨识，仅输出测量结果。正式辨识前卸下配重。

### 离线复算

```bash
rebot identify replay runs/calibration/right/<历史运行>/data.jsonl --output runs/replay-right
```

复算只读取已完成的版本 3 双向采样日志。查看 `report.json` 中的留出误差、正反向半差和两种力矩信号的差异。拟合可用时在输出目录生成 `calibration.yaml`，可直接用于 `rebot guide --calibration`。
