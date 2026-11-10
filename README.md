## rebot arm rs版的设置，控制，遥操作

### reBot Arm RS 初始化

`rebot.py` 沿用 Star Arm 的 `scan → bind → calibrate` 流程，使用当前环境已有的 `robstride-dynamics` SDK，通过原生 SocketCAN 查询 B601-RS。默认查询电机 ID 1–7（J1–J6 和夹爪），主机 ID 为 `0xFD`；型号按参考仓库配置为前三轴 `rs-06`、其余 `rs-00`。型号来自配置约定，扫描不会自动识别型号。参考本机 `real_projets/reBotArm_control_py/config/rebotarm_rs.yaml` 与 `example/2_zero_and_read.py`，以及 [Seeed RS 官方说明](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_pinocchio_meshcat/)。

`scan` 自动检查并配置 CAN：接口已启动且为 1 Mbps 时直接扫描；未启动、速率不符或处于 BUS-OFF/STOPPED 状态时，自动执行 `ip link` 的 down、设置 1 Mbps、up，再检查配置结果。普通用户通过 `sudo` 执行这三步，必要时终端会提示输入系统密码；仅这三条网络配置命令提权，Python 程序仍使用当前用户身份运行。直接执行：

```bash
conda activate rebot_control
python rebot.py scan
```

默认处理所有原生 CAN 接口；单臂实验可用 `python rebot.py scan --channel can0` 仅配置并扫描指定接口。重启或重新拔插适配器后，再运行 `scan` 即可，无需手动重复设置。配置失败会显示系统错误并跳过该接口，不向其发送电机查询。`bind`、`calibrate`、`read` 仍只检查接口，若未就绪先运行 `scan`。需先停止占用这些电机的其他控制程序。每臂使用独立 CAN 适配器，两臂电机 ID 相同，不能直接合并在一条总线上。

核对实物与适配器的对应关系后绑定左右身份。下面仅以 `can0` 对应左臂、`can1` 对应右臂为例，不能由接口编号推断实体左右：

```bash
python rebot.py bind left can0
python rebot.py bind right can1
python rebot.py read left
python rebot.py read right
```

绑定要求七个 ID 全部在线且能读取角度，记录 USB 设备路径、电机 UID 和接口参数到被 Git 忽略的 `rebot.local.yaml`。USB 口不变时可根据设备路径找到重新编号后的 CAN 接口；换 USB 口后需重新绑定。更换适配器、机械臂或重新绑定加 `--replace`，旧软件标定同时清除。扫描仅覆盖预期 ID 1–7，不检测其他 ID，也不能可靠发现同一总线上重复 ID 的电机。

若显示 `IDs []`，扫描会额外输出本次收到的帧数、CAN 状态以及系统累计收发包数和错误计数。本次收帧为 0 时，先检查机械臂独立 48V 电源是否开启，再检查 CAN 连接和终端电阻；USB-CAN 连接成功不表示电机已供电。收到其他流量但没有匹配的 ID 回复时，再检查电机 ID、协议配置。SDK 的 `WARNING: No calibration provided` 表示此查询连接没有加载运动标定，不会阻止扫描。设备 ID 查询（类型 0）的回复地址固定为 `0xFE`，参数读取仍校验请求的主机地址 `0xFD`，不能混用两者；见 [RobStride 电机协议手册](https://www.robstride.com/assets/product_manual_robStride04-37549d59.pdf)。

**本机排障记录（2026-11-02）：CAN 已启动，但两臂均扫描不到 ID。** 两只机械臂的 48V 电源均已开启，用户确认接线和设备未更改，前一天可以正常读取。故障时 `can0`、`can1` 均为 UP、1 Mbps、`ERROR-ACTIVE`，发送包数增加，但接收包数始终为 0，`scan` 输出 `IDs []`。扩大 ID 和主机地址范围的只读探测、直接调用原版 RobStride SDK 查询也均无回复。因此，不能仅凭接口已启动或没有 CAN 错误计数就判断电机通信正常。

本次经用户验证，**重新拔插 USB-CAN 后恢复了 ID 识别**。遇到同样现象，确认电机已供电后，可先按以下步骤恢复连接：

1. 退出正在使用机械臂的控制或调试程序。
2. 拔插电脑端的 USB-CAN USB 连接，不插拔带电的电机线束。
3. 重新运行 `python rebot.py scan`，由脚本重新配置 CAN 并扫描；核对 USB 路径及左右臂对应关系，接口编号可能变化。

本次恢复没有依赖重写电机 ID、硬件零位或重新标定。具体原因尚未确定，暂不能归因于驱动、适配器固件或 USB 连接中的某一项；重新拔插是本次验证有效的恢复方法。

将机械臂和夹爪摆到约定的参考姿态（与 StarArm 遥操参考零姿态相对应，夹爪闭合），托稳并保持静止后采样：

```bash
python rebot.py calibrate left
python rebot.py calibrate right
python rebot.py read left --watch
python rebot.py read right --watch
```

`calibrate` 立即读取十组新鲜角度；任一电机变化超过 2° 则拒绝保存。结果为七路 `reference_rad`，重新标定加 `--replace`。`read` 显示原始角度和减去参考角后的相对角度，单位为度，保留连续角度；`--watch` 持续读取，Ctrl-C 退出。此处输出的是电机坐标，未套用 StarArm 的方向映射或夹爪角度—行程换算。

若六轴零位正确，但已经完全闭合的夹爪相对角度偏离零，可以仅更新夹爪的闭合参考：

```bash
python rebot.py calibrate right --gripper-only --replace
```

执行前确认实体夹爪完全闭合并保持静止。命令核验绑定电机 UID，仅读取十次第七路位置，取中位数写入 `reference_rad[6]`，保留 J1–J6 参考角和其他绑定信息。需要已有完整软件标定；夹爪变化超过 2°、通信失败或绑定在采样期间改变时不保存。只修改本地参考，不发送使能、运动或硬件零位指令。程序会打印旧、新夹爪参考角；持续出现闭合读数变化时仍需排查机械闭合状态或电机零位状态，不能用重复软件标定证明问题已消失。

这些命令只发送设备 ID 查询和位置参数 `0x7019` 读取，不发送使能、卸力、运动、模式切换、硬件零位或参数写入；退出仅关闭 CAN socket。它们不释放已有锁力，也不提供重力补偿。缺失反馈、UID 不匹配、无效角度或收到故障报告时停止当前操作；不会用零值补齐。双臂按实物核对后分别执行，不要求同时连接。

此软件参考标定与官方示例向电机发送的零位设置不同。尚未完成出厂/硬件零位设置的机械臂，需要按 [RS 快速入门](https://wiki.seeedstudio.com/cn/rebot_b601_rs_getting_started/) 完成对应步骤；本脚本的本地偏置不能替代该步骤。`stararm_teleop.py` 的 Rebot 目标只控制 Three.js 中的虚拟机械臂；实体遥操使用下文的 `rebot_teleop.py`。

### 实体 reBot 双臂姿态同步到 Three.js

项目的 3D viewer 统一使用本地 Three.js，覆盖 Star Arm 预览、reBot 双臂镜像和末端控制。MuJoCo 保留为计算后端，不创建原生窗口；Mink 和 quadprog 保留用于末端 IK。Three.js 与控件已随项目打包，WebSocket 服务使用基础依赖 `websockets`。

完成两臂的 `bind` 和 `calibrate` 后，运行新脚本：

```bash
conda activate rebot_control
python rebot_mirror.py
```

程序读取 `rebot.local.yaml`，按 USB 路径解析两块 CAN 适配器，自动检查并按需将两路 CAN 配置为 UP、1 Mbps，再核验两臂七个电机的 UID，打开本机 Three.js 双臂页面。拔插适配器后直接重新运行 `python rebot_mirror.py`，无需先运行 `scan` 或手动执行 `ip link`；接口已就绪时不重新配置，需配置时可能提示输入 sudo 密码，仅网络配置命令提权。两路接口都配置成功后才开始查询电机，配置失败则停止启动。手动拖动实体 Rebot Arm 左右臂，仿真中的对应机械臂跟随显示。沿用现有 RS 模型、桌面和双臂布局。

实体臂应先处于可手动拖动状态，并在拖动时托稳。此脚本只查询角度，启动、运行和退出均不发送使能、卸力、运动、模式切换或硬件零位指令，也不提供重力补偿。如果实物已锁力，脚本不会自动解除锁力。点击页面 `Close viewer` 或在终端按 Ctrl-C 退出并关闭两路 CAN 连接，不改写本地标定。

六轴按 `当前电机角度（弧度）− reference_rad` 直接赋给对应模型关节，不应用 StarArm 的方向映射或角度倍率，不折回 ±π。标定时的实体参考姿态需对应模型零姿态，否则显示会有偏差；超出模型关节范围时裁剪并在终端提示。这里只更新关节位置并计算运动学，不进行动力学积分。

夹爪用第七路电机相对于闭合参考角的变化显示开合，默认将 0–270° 映射为模型单侧 0–50 mm 行程，不使用 StarArm 手柄的 `×6` 倍率。该角度—行程关系仅用于比例预览，尚未实测标定，可按需要调整：

```bash
python rebot_mirror.py --fps 30 --gripper-open-deg 270
python rebot_mirror.py --config /path/to/rebot.local.yaml
python rebot_mirror.py --no-browser --port 8080
```

页面默认使用 `127.0.0.1:8080`，端口占用时自动向后选择；`--port` 指定起始端口，`--no-browser` 仅打印地址。关闭或刷新浏览器后允许重新连接，退出请点击 `Close viewer` 或在终端按 Ctrl-C。

默认请求 30 Hz，每次顺序读取左、右臂各七路角度，不保证硬件同步；实际刷新率取决于 CAN 反馈速度。仅在两臂反馈均完整时更新姿态，任一侧读取失败则冻结两只模型并报告失败侧，短暂缺帧会重试；连续失败至少三次且距上次完整反馈已超过两秒时退出。收到电机故障报告则立即退出，不等待重试。

### 遥操作数据采集

独立入口 `rebot_collect.py` 同时启动遥操作与数据采集，采集从臂的实际 EEF 位姿、实际夹爪开合和带逐帧时间索引的 H.264 视频，按 LOOM 目标格式保存。一次运行生成一个 episode，要求双臂、双夹爪和至少一路相机：

```bash
conda activate rebot_control
python -m pip install 'pyarrow>=17'
python rebot_collect.py --dataset datasets/cup \
  --instruction 'pick up the cup' \
  --camera top=/dev/video0 --camera left_wrist=/dev/video2
```

按 Space、Esc、Q 或 Ctrl-C 停止遥操作并保持，随后完成视频核验和导出；看到 `Saved episode ...` 表示成功。下次运行向同一数据集追加新 episode。相机 ID 必须与物理设备稳定对应，所有流按原生时间戳独立记录、统一归零。数据坐标轴与末端控制坐标轴之间有固定变换；物理夹爪全开角度与控制行程分别配置。系统 GStreamer 依赖、完整参数、源坐标系及采样时刻定义、输出结构与失败数据处理见 [采集说明与 reBot 源契约](docs/recording.md)。原有 `rebot_teleop.py --dataset ...` 入口仍然可用；仅遥操作继续使用 `rebot_teleop.py`。

### Star Arm 遥操实体 reBot

`rebot_teleop.py` 将 Star Arm 的七路串口角度映射为实体 RS 的原生 MIT 位置、速度和 PD 指令；遥操不启用重力补偿，前馈力矩设为零，不启动 viewer。默认控制左右双臂及两只夹爪，也支持只连接单侧。运行即使能选中的电机，无需使能开关。

先按前文完成所选两种机械臂的 `bind` 和 `calibrate`；默认分别读取 `stararm.local.yaml` 和 `rebot.local.yaml`。两者标定姿态应对应同一模型零姿态，两只手柄及实体夹爪均以闭合位置为参考。CAN 未就绪时先运行 `python rebot.py scan`。停止其他真机控制程序，将主从臂摆到接近的对应姿态并留出运动空间；程序释放 Star Arm 锁力，启动前需托稳主臂。

```bash
conda activate rebot_control
python rebot_teleop.py --side right                  # 右侧主臂控制右侧真机和夹爪
python rebot_teleop.py --side left                   # 左侧
python rebot_teleop.py                               # 双臂和夹爪；默认
python rebot_teleop.py --side right --no-gripper     # 仅控制 J1–J6，不改变夹爪控制状态
```

电机需事先处于 MIT 模式；脚本不自动失能或切换运行模式，模式不符时拒绝启动。

主进程使用 `multiprocessing` 的 `spawn` 启动子进程，不向子进程继承串口或 CAN socket。开始运动前，所有硬件进程先核验从臂七个电机 UID、主臂完整反馈、软件参考角、独立串口和 CAN 通道及当前位置；全部通过后释放主臂锁力，并检查所有从臂电机的 MIT 模式及设置看门狗。只有各侧均完成准备后才允许使能，所有选中从臂使能完成后再开始跟随。一侧身份或模式检查失败时不会使能另一侧。支持继续采用上次运行留下的电机侧位置保持。

六轴目标为 `radians(Star Arm 多圈角度 − reference_deg) × [+1,+1,−1,−1,−1,+1]`，不折回 ±180°，再裁剪到官方 RS 遥操限位和 MIT 原始角度编码范围的交集；发送时加回从臂 `reference_rad`。目标裁剪和实测姿态校验均采用这套遥操限位，不再使用动力学 URDF 的关节范围，因此 J3 可到 200°。主臂越过限位后从臂保持在该端，反向回到范围内时继续跟随。

当前位置限位与 [官方 LeRobot RS follower 配置](https://github.com/Seeed-Projects/lerobot-robot-seeed-b601/blob/main/lerobot_robot_seeed_b601/config_seeed_b601_rs_follower.py#L64) 一致，单位为度，相对于从臂软件标定零位；最终还受 MIT 原始角度编码范围约束。

| 关节 | 官方 RS 遥操限位 |
| --- | --- |
| J1 | −145～145 |
| J2 | 0～170 |
| J3 | 0～200 |
| J4 | −80～90 |
| J5 | −90～90 |
| J6 | −130～130 |
| 夹爪电机 | 0～270 |

实体夹爪采用**连续位置跟随＋闭合端预压＋总输出力矩指令限幅**，实现在 `rebot_gripper.py`。主手距离闭合参考 `1°` 以内统一映射为完全闭合，其余手柄行程连续映射到从臂张开端：`openness = clip((handle_delta_deg - leader_close_tolerance_deg) / (45 - leader_close_tolerance_deg), 0, 1)`。默认电机范围仍为 `0..270°`，主手参考角差 `45°` 时完全张开；用 `--gripper-open-deg 180` 时完全张开对应电机 `180°`。主手闭合参考来自 `stararm.py calibrate`，从手闭合参考来自 `rebot.py calibrate` 的第七路角度，容差与预压分别配置，不修改标定文件。角度与实物行程关系仍需核对，可用 `--no-gripper` 保留原有夹爪状态。即使关闭夹爪控制，身份检查仍要求七个绑定电机在线。

每帧直接使用最新开合目标和位置、速度反馈，在上位机计算 `tau = clip(Kp * (q_effective - q_measured) - Kd * v_measured, -torque_limit_nm, +torque_limit_nm)`。内部虚拟目标 `q_effective = q_target - radians(close_preload_deg) * weight`；默认在最后 `5°` 从手电机行程内平滑加入闭合预压，完全闭合时达到 `1°` 电机角偏置。令 `window = min(radians(close_preload_window_deg), open_position - closed_position)`、`s = clip(1 - (q_target - closed_position) / window, 0, 1)`，则 `weight = s*s*(3 - 2*s)`。该权重只依赖当前开合目标，没有时间延迟或历史累积；张开超过预压窗口后为普通 PD。真实闭合位置与共享目标仍保留原值，内部虚拟目标允许低于闭合参考，不会被目标零位裁剪掉。

位置预压与阻尼先合成，再统一限幅；夹爪 MIT 报文的电机侧 `Kp=Kd=0`，全部控制力矩通过前馈字段下发。报文编码也限制力矩码范围，避免负力矩向下取整后超过设置值。固定摩擦前馈已移除，未叠加原来的 `0.3 Nm` 补偿。夹爪没有积分、接触/夹持状态切换、力矩历史窗口、目标重置或时间滤波；受阻时保留操作者目标，阻碍消失后继续跟随。

参数位于 `rebot_gripper.yaml`：默认 **`torque_limit_nm: 1.0`、`kp: 12.0`、`kd: 0.5`、`leader_close_tolerance_deg: 1.0`、`close_preload_deg: 1.0`、`close_preload_window_deg: 5.0`**，两侧使用相同参数。`--gripper-torque-nm` 覆盖包含预压与阻尼的正反两方向总力矩指令上限。增益和预压基于从手电机角度，不随 `--gripper-open-deg` 缩放。主手容差与从手预压可分别设为零来关闭；容差必须小于 `45°`，预压窗口必须为正。自定义 YAML 必填 `torque_limit_nm`、`kp`、`kd`，其余三项可省略并采用默认值。旧字段 `ki`、`friction_torque_nm`、`position_deadband_rad` 以及旧接触状态配置不再使用，包含它们的配置会报错，需删除后再运行。

主手 `1°` 容差与从手 `1°` 预压为本机待实测的起点；默认 `Kp=12 Nm/rad` 时，在真实闭合位静止会请求约 `−0.209 Nm` 闭合预压力矩。预压不足以克服实际静摩擦时仍可能留下开口；已达到总力矩限幅时，再增加偏置不能突破该上限。限幅约束正常遥操作下发的总力矩请求，不是经过标定的指尖力，也不保证实测力矩没有动态误差。本次未驱动实物。


启动时先用电机侧 PD 保持真机当前位置。完成逐轴使能和跨进程启动屏障后，先批量重发当前位置保持并收齐新反馈，再开始夹爪上位机控制，避免早先使能轴的反馈过期。启动及运行期间仅对 J1–J6 的实际反馈检查关节限位，容差为 `0.1 rad`（约 `5.73°`）；夹爪实际反馈不参与限位判断，但所有受控电机反馈仍须完整、有限且新鲜。目标位置按上述范围裁剪，包含夹爪及 `--gripper-open-deg` 设置。

六轴机械臂仍直接下发映射后的位置目标，由电机侧 MIT PD 执行；夹爪的位置目标用于上位机 PD 与闭合预压。二者均不做目标速度限制、插值或限速接入；主从初始姿态不同时，第一帧就使用对应的主臂目标。MIT 目标速度统一设为零，作为阻尼参考，不代表实际运动速度为零。程序不提供 `--speed`、`--gripper-speed` 参数，也没有加速度、jerk 限制或环境、自碰撞及桌面避障。

```bash
python rebot_teleop.py --side right --gripper-open-deg 180 --gripper-torque-nm 0.3
python rebot_teleop.py --side both --gripper-settings /path/to/rebot_gripper.yaml
python rebot_teleop.py --config /path/to/rebot.local.yaml --leader-config /path/to/stararm.local.yaml
```

原生六轴增益保留 `kp=[50,150,150,50,50,50]`、`kd=[3,10,10,5,4,4]`，复用归位/采样的 `MOTION_KP/MOTION_KD`，保持零前馈力矩。夹爪正常跟随使用上位机 PD、闭合预压和总力矩指令限幅；启动与退出使用当前位置的原生 PD 保持，增益取夹爪配置的 `kp/kd`，前馈为零。遥操不加载动力学模型或重力辨识配置，不提供 `--calibration` 参数。反馈过期、控制间隔和温度阈值仍从 `rebot_hand_guiding.yaml` 读取，可用 `--settings` 更换；遥操频率固定为下述 200/500 Hz，不使用该文件的 `rate_hz`。其中 `gravity_scale` 和 `torque_limit_nm` 不参与遥操控制，**不限制电机侧 PD 总输出或夹持力**。

遥操使用独立进程和只保留最新一帧的共享内存，双臂共五个工作进程加一个监控主进程；单臂为三个工作进程加主进程：

| 进程 | 目标频率 | 工作 |
| --- | --- | --- |
| 每侧 Star Arm 采集 | 按串口实际速度连续采集 | 独占该侧串口，发布完整七轴角度和读取开始的单调时钟时间戳 |
| 目标更新 | 500 Hz（2 ms） | 读取两侧最新采集快照，方向映射、位置裁剪后一次性发布完整主从目标对 |
| 每侧 reBot 控制 | 200 Hz（5 ms） | 独占该侧 CAN，取最新目标发送整批六/七轴原生 MIT 指令，再收集反馈 |
| 监控主进程 | 约 100 Hz | 协调启动阶段、处理按键/信号、检查子进程退出及维持心跳 |

500 Hz 是共享缓存上的目标处理频率，不表示串口每秒获取 500 组新数据。目标重发保留原始采集时间戳用于统计，不通过重复发布伪造采样时间。通信容错参考 [YAM 电机驱动](https://github.com/i2rt-robotics/i2rt/blob/0b475453c1412e00bd567dbe0a8a11975a40b114/i2rt/motor_drivers/dm_driver.py)及其[通信重试](https://github.com/i2rt-robotics/i2rt/blob/0b475453c1412e00bd567dbe0a8a11975a40b114/i2rt/motor_drivers/can_interface.py)：

- 主臂一次角度读取不完整时报告失败并重新读取，保留最后一帧完整七轴角度；恢复后继续跟随，连续 15 次失败才停止。Star Arm 使用自己的串口协议与 SDK 读取等待，不能将 CAN 的 10 ms 直接当作主臂读取时限；SDK 单次缺包读取可能耗时约 0.8 秒。
- 从臂先整批发送六/七轴 MIT 指令，每轮等待反馈 10 ms，缺失轴最多尝试 15 次；只重发缺失轴，已收到的有效反馈保留。CAN 收发异常也进入重试，电机上报故障仍立即停止。默认单次事务与截止时间规则继续用于其他控制入口，遥操作显式启用这组重试参数。
- 从臂持续执行最近一次有效目标，主臂采集或目标更新暂缓时继续发送缓存目标。移除遥操作中目标年龄 0.08 秒和控制间隔 0.10 秒的硬退出，不再用主臂采集时间缩短 CAN 收包期限；共享拖动配置里的这两个阈值仍用于拖动等原有入口。循环变慢和数据年龄保留在时序统计中。

连续读取失败耗尽尝试次数、电机故障或工作进程异常退出时，停止所有选中侧并尝试位置保持。主进程停止维持心跳超过 0.5 秒时，工作进程仍会自行退出并尝试保持；一侧进程意外死亡由主进程检测并通知其余进程停止。进程仍存活但不更新目标时，从臂按缓存语义维持最后目标，直至新目标到达或显式停止。

正常控制不再逐周期额外读取七路 `0x7019`，而是使用 MIT 回复中的位置做姿态校验，并更新退出保持的备用姿态；夹爪控制复用上一批成功指令的反馈位置、速度，不增加 CAN 查询，力矩仍作为遥测并检查有限性，同时校验反馈完整且处于运行状态。每批指令要求在重试耗尽前收到全部受控轴的有效反馈；不因重试使上一批反馈超过 0.08 秒就提前退出。启动和退出清理仍读取机械位置参数。这样减少每轴每周期的查询/回复通信。左右 CAN 进程独立运行，互不等待，两个从臂不保证硬件同步。

循环使用单调时钟安排下一周期；发生超时跳过已错过的周期，不集中补发旧目标。共享数据加锁读取完整快照，锁等待有 0.01 秒上限。退出时分别打印主臂采集、目标更新、每侧 CAN 的实测平均 Hz、最大循环间隔、最大工作耗时、最大主臂数据年龄和跳过周期数。200/500 Hz 是请求值，不能代替实机测量，也不是硬实时保证。

空格、Esc、Q、Ctrl-C 或处理到的 SIGTERM/SIGHUP 通知各进程停止。信号处理函数只记录本进程的停止请求，常规控制流程再更新跨进程停止事件；避免信号打断事件的锁操作时重入 `Event.set()`，导致主进程和子进程一起卡住、串口锁无法释放。每个 CAN 进程在关闭自己的 socket 前，尝试对已经启用的电机安装零目标速度的位置保持，退出不自动失能。沿用现有控制约定，将选中电机 CAN 看门狗设为零并回读确认，退出不恢复旧值、不写 EEPROM；主臂保持自由卸力。主进程默认最多等待清理 5 秒；仍未退出的进程会被强制结束并报错，不能声称该进程已确认当前位置保持，上一条电机侧位置指令可能继续生效。CAN 故障、断电或直接对电机进程执行 SIGKILL 也不能保证清理成功，以终端逐电机确认/错误为准。需要归位并释放从臂时，先准备参考姿态处的支撑，再运行 `python rebot_home.py --side right`（双臂用 `--side both`）；该命令会先运动归位，再失能。

退出时，J1–J6 与夹爪均切回实测当前位置的原生 PD 保持，前馈力矩清零；位置查询失败时使用最后一次已验证的位置。电机仍保持使能，主机侧位置环、闭合预压与总力矩限幅已经停止，**启动保持与退出后的原生保持不受上述主机力矩上限约束**。停止后不会继续追赶操作者的闭合目标，也不承诺继续保持物体夹持力。

离线验证实际启动 `spawn` 子进程，硬件由可跨进程创建的模拟串口/CAN 替代。覆盖硬件连接分属独立 PID、正常跟随目标、既有 MIT 增益、零速度/零前馈、运行期间无额外机械位置查询、单臂不控制夹爪、跨进程 UID/模式启动屏障、部分使能失败、CAN 第 15 次回复恢复及持续丢包退出、主臂连续缺帧后恢复及 15 次失败退出、慢采集时持续执行缓存目标并保留真实数据年龄、控制间隔超过 0.10 秒后继续运行、重试中手动停止、CAN 进程突然退出时另一侧保持、主进程 SIGTERM、停止事件持锁期间的重复信号、长等待中的信号退出、监控心跳中断，以及共享锁超时和过期周期跳过。夹爪另验证主手闭合容差与全开端点、缩短行程、闭合末段预压连续性、闭合位置持续预压、无力矩累积、窗口外无固定摩擦助力、预压与阻尼合成后的总力矩限幅、MIT 编码量化边界、禁止电机侧 PD 绕过限幅、受阻后即时反向响应、模型静摩擦下的闭合、主手容差到从手闭合预压的跨进程链路、启动反馈刷新、无效反馈停机及退出原生位置保持。另保留官方限位及归位模块内七轴失能的回归。本次未驱动实物，200/500 Hz 的实际时序、MIT 反馈位置、方向和夹爪范围仍需实机验证。

### Mink 末端位姿控制（EE Pose）

`rebot_ee.py` 是独立的单臂末端控制入口，支持 `--side right|left`。默认只运行运动学仿真，不读取绑定、不打开 CAN；加 `--hardware` 才连接并使能实体 J1–J6，夹爪保持原有状态。交互界面统一使用 Three.js Web viewer；MuJoCo 仅作为模型、正向运动学与碰撞计算后端，Mink 继续负责 IK。已移除原生 MuJoCo viewer，不使用 Viser。当前没有接入 Star Arm 主臂，也不同时控制双臂。

控制链路为：**基座坐标系下的 TCP 目标 → Mink / quadprog 在一次调用中迭代求出关节解 → 加回 `reference_rad` → 现有 `MotorBus.command_position` 直接下发 → MIT 电机侧 PD**。IK 参数与 YAM 官方 `Kinematics.ik` 对齐：数值步长 0.01、最多 200 次迭代、位置/朝向误差阈值均为 `1e-4`、任务 `lm_damping=1`、QP `damping=1e-4`。这些迭代连续执行，数值步长不代表实际运动时间；求出的关节目标直接发送，不按控制周期插值，不添加关节速度、加速度或 jerk 限制。`--joint-speed` 和 `--max-tracking-error` 已移除。沿用遥操的六轴 `MOTION_KP/MOTION_KD`，`v_des=0`；硬件模式默认启用重力前馈，`tau_ff = g(q_measured) × gravity_scale`。目标求解/显示循环请求 50 Hz，独立 CAN 线程请求 250 Hz；双方通过最新指令和反馈快照通信，错过周期跳过，不补发。

重力补偿使用现有 `GravityModel` / Pinocchio 和 `models/rebot_rs_dynamics.urdf`，输入为 `原始实测电机角度 − reference_rad`，不使用 IK 目标角度。启用前的保持、首批指令、CAN 循环、页面 `Hold` 和退出位置保持均携带补偿。`rebot_hand_guiding.yaml` 中的 `gravity_scale`（默认 `[1,0.98,0.98,1,1,1]`）和 `torque_limit_nm` 在 EE 中分别用于逐轴重力缩放及前馈检查；计算结果非有限或超限会报错并进入已有清理流程，不静默裁剪。该限值只约束重力前馈，**不限制电机侧 PD 总力矩**。使用原始 CAD 动力学模型，假定底座水平、机械臂竖直安装及原配空载末端，EE 当前不加载重力辨识配置；补偿可减小重力导致的稳态误差，模型偏差和摩擦仍可能留下残差。

```bash
conda activate rebot_control
python -m pip install 'mink>=1.4.0,<2' 'quadprog>=0.1.13' 'websockets>=14,<17'
# 使用 uv 管理依赖时也可运行 uv sync --extra ee

# 默认运动学仿真：自动打开本机 Web viewer，直接拖动三维操作柄
python rebot_ee.py

# 只输出浏览器地址
python rebot_ee.py --no-browser --port 8080

# 仿真：TCP 沿基座 X 正方向移动 1 cm，保持启动朝向，运行 2 秒
python rebot_ee.py --headless --offset 0.01 0 0 --duration 2

# 仿真：指定绝对位置与朝向；位置单位 m，角度单位 deg
python rebot_ee.py --pose 0.30 0.07 0.65 0 -30 20

# 实体右臂：从实测 TCP 沿基座 X 正方向移动 1 cm，然后退出并保持
python rebot_ee.py --hardware --side right --headless --offset 0.01 0 0 --duration 2
```

Web viewer 默认绑定 `127.0.0.1:8080`，端口占用时向后选择可用端口，终端打印实际地址；可用 `--port` 指定起始端口。前端使用本地打包的 Three.js 0.180.0、标准 `TransformControls` 和 `OrbitControls`，不依赖 CDN、Node.js 或前端构建步骤。模型网格启动时加载一次，后续通过同端口 WebSocket 更新各连杆姿态；浏览器以自身帧率渲染并立即更新正在拖动的目标，约 50 Hz 向后端发布最新位姿。状态携带目标序号，避免旧的反馈把本地目标拉回。

页面完成模型加载并连接后才打开 CAN。等待期间可在终端按 Ctrl-C 退出；`--no-browser` 仅打印访问地址。初始化完成后即可直接拖动或输入末端目标，无需点击开始按钮或确认弹窗。未指定启动目标时默认保持当前位置；真机使用 PD 加重力前馈。

界面按 [Apple 设计原则](https://developer.apple.com/design/human-interface-guidelines/design-principles) 聚焦当前任务：全屏机械臂视图、一个状态提示和底部操作工具。不再提供关节滑块、PD 调参、零力矩模式、坐标系切换或步进设置，对应 Web 命令与旧模式分支也已删除。电机使用固定的原有 `MOTION_KP/MOTION_KD`，期望速度为零；硬件前馈力矩由实测姿态的重力模型持续更新。

| 操作 | 方式 |
| --- | --- |
| 平移 | `Move` / `T`，拖动轴箭头或平面方块 |
| 旋转 | `Rotate` / `R`，拖动圆环 |
| 重置相机 | `Reset view` / `F` |
| 精细移动 | 按住 Shift：平移 1 mm、旋转 1° |
| 撤回本次拖动 | 拖动时按 Esc；不会结束控制 |
| 调整相机 | 空白处左键旋转、右键平移、滚轮缩放 |

目标统一使用基座坐标系。`Set pose` 按需展开数值输入，位置单位 mm，固定轴 XYZ 欧拉角单位 deg；点 `Apply pose` 提交并收起面板。前后端统一使用 `Rz(yaw) Ry(pitch) Rx(roll)`。末端误差显示在此面板中，碰撞拦截和断线才在主视图显示额外提示。编辑输入框或聚焦按钮时不会触发工具快捷键。

`Hold` 放弃当前目标，以实测当前位置重新建立末端目标。`Stop & hold` 结束程序并进入原有 MIT 保持清理。显式 `--pose` / `--offset` 在初始化完成后立即执行。关闭或刷新页面允许重连，保留当前控制状态及最后指令；退出请用 `Stop & hold` 或终端 Ctrl-C。Web 模式不读取终端空格/Esc/Q，headless 模式保留这些停止键。退出时打印 `EE stop reason`，区分页面按钮、终端键和信号；不可达目标不会自动退出。

双臂镜像和 Star Arm 预览沿用同一全屏布局，只显示预览状态、`Reset view` 和 `Close viewer`，不显示末端控制工具。

`--headless` 不启动 viewer，必须同时给出固定目标和 `--duration`。固定目标在期限结束时以末端误差不超过 2 mm、朝向误差不超过 2° 判定到达；到达返回 0，中止返回 130，未到达或出错返回 2。这是基于关节反馈和模型的判定，不是外部测量精度保证。

`--pose x y z roll pitch yaw` 使用单臂底座坐标系，旋转为固定轴 XYZ 欧拉角（`R = Rz(yaw) Ry(pitch) Rx(roll)`），不是双臂场景的世界坐标。`--offset dx dy dz` 相对本次启动实测位置、沿基座轴平移并保持启动朝向。仿真初始关节角默认为 `[0,0,0,0,0,0]°`，对应模型参考零姿态，可用 `--start-joints` 改变；实体只接受实测启动姿态，不接受该参数，也不会自动运动到仿真零姿态。

默认 EE/TCP 现定义在夹爪安装接口中心，即 MJCF 的固定坐标系 `gripper_mount` 原点：相对 `link6` 平移 `[0,0,0.009] m`，朝向与 `link6` 一致。9 mm 对应 CAD 安装盘前表面，模型夹爪后端也位于此处；这是模型定义，尚未用实物重新标定。Z 轴沿安装面法线朝夹爪，X/Y 轴沿用安装盘的 `link6` 固定方向，不随指尖、夹爪开口或夹爪网格改变。IK、Web 操作柄、数值位姿和末端误差均使用此参考点。`--tcp-offset x y z` 默认为零，现在表示在 `gripper_mount` 坐标系中的附加位置偏移，单位 m，朝向保持不变；混合训练时应保持零偏移并统一各平台的安装坐标轴、位姿表达和预测时间间隔。

这是参考系语义变更：旧版默认使用 `gripper_end` 原点，其相对 `link6` 的平移为 `[0,0,0.16621] m`，朝向也不同。旧的绝对 `--pose`、历史 EE 状态/目标以及非零 `--tcp-offset` 不能直接当成新的安装接口位姿使用，需按完整刚体变换转换。更换 fingertip 无需重新定义默认 EE；指尖接触位置和工具外形差异仍需由视觉或工具条件学习。

IK 在内存中固定夹爪的三个显示用滑动关节，只求解 J1–J6；不修改原始 MJCF。关节范围使用前文官方遥操限位与 MIT 可编码范围的交集，含 J3 的 200° 上限。模型角度为 `原始电机角度 − reference_rad`，不再次应用 Star Arm 的方向映射。真机启动前预热求解器、核验七个 UID、检查六轴模式和姿态，随后沿用原生 MIT 使能及看门狗设置。每批指令使用 MIT 回复刷新反馈及备用保持姿态，不逐周期查询机械位置。

EE 路径参考 [YAM Kinematics](https://github.com/i2rt-robotics/i2rt/blob/main/i2rt/robots/kinematics.py)。每次求解以最新实测关节角为初值；采用 Mink 默认的越界处理，不在 IK 入口或每次积分后增加严格角度拒绝，也不设置 `safety_break=True`。输入仍必须有限、形状正确。不可达目标在 200 次迭代后返回最后近似解，Three.js Web 和 headless 模式会继续使用。发送前按 reBot 关节/编码位置边界裁剪，不添加速度、跟踪误差或 IK 耗时停机阈值。

独立 CAN 线程持续重发最后一条被接受的目标，并刷新实测关节角；IK 变慢或显示停顿不会使 CAN 等待求解。原有反馈时效、故障及有限时间的 CAN 事务检查保留在通信层；IK 不再使用上一轮求解前的反馈时间作为发送期限，也不使用遥操的 `max_sample_gap_s` 限制目标循环。目标与 CAN 的实测频率分别在退出时打印。Web 回调只发布目标和开始、保持、停止请求，不操作 CAN 和求解器。线程先停止并汇合，再由原有 `MotorBus` 安装退出保持；正常退出不失能、不归位、不恢复看门狗，不保证 SIGKILL、断电或通信故障时保持成功。

目标自碰撞检查与 YAM 一样，使用独立 MuJoCo 数据检查将要发送的姿态，忽略平面和相邻刚性连杆，穿透深度超过 1 mm 时阻止更新指令并保留上一条已接受目标。内存模型开启现有网格的接触标志，固定夹爪零件按所属刚性连杆判断相邻关系，不修改源 MJCF。网格接触使用 MuJoCo 凸包近似，尚未用实物标定；没有整段路径、桌面或外部环境避障。运动学仿真只验证模型与目标求解，不模拟 PD 跟踪、负载或实物接触。

保留的 reBot 适配包括 RS 原生扩展帧协议、模型/TCP、软件零位、关节及编码范围、默认 PD 增益、UID/模式核验、通信故障处理和退出位置保持；不复制 YAM 另一款电机的增益数值、CAN 事务时序或扩大的硬件关节边界。当前仍只控制六轴，不改变夹爪状态，未加入 YAM 的夹爪/示教手柄功能。摩擦前馈为零。

离线测试使用真实 Mink 与虚拟 CAN，覆盖完整位姿直接发送、边界附近实测 IK 初值、目标自碰撞拦截及显示隔离、不可达目标、TCP 偏移、软件零位、MIT 增益、零速度、实测姿态重力前馈及异常检查、UID/模式检查、缺失反馈、慢 IK 时 CAN 继续刷新、停止期间不发布新目标及退出保持。Web 协议测试另覆盖开始控制的确认门控、固定 MIT 增益、拒绝旧模式与调参命令、完整位姿与目标序号、错误消息隔离、静态资源和路径边界、断线后保持以及终端转义序列不误停；移动/旋转、数值输入、确认弹窗和刷新恢复另在浏览器验证。运行：

```bash
python -m unittest discover -s tests -p 'test_rebot_ee*.py' -v
```

### 手动引导与位置保持（Hand Guiding）

`rebot_hand_guiding.py` 实现手动引导（Hand Guiding）：手动拖动时提供重力补偿与阻尼，静止后保持位置；当前不记录或回放轨迹。支持 `--side right|left|both`，默认仅选右臂。它与只读的 `rebot_mirror.py` 独立运行，本身不启动 viewer。参考仓库的重力补偿及速度锁止示例提供了设计依据，但本脚本使用独立的通信和控制实现，不调用示例的自动归零流程。

重力模型来自 `models/rebot_rs_dynamics.urdf`，保留参考仓库的惯性参数，由 Pinocchio 计算六轴 `g(q)`；夹爪移动分支固定在闭合位置，质量仍计入整臂。模型坐标为 `电机原始角度 − reference_rad`，正方向与姿态镜像一致。**使用前需核对实物参考姿态与模型零姿态、逐轴正方向；当前两臂零姿态已由用户确认一致，但动力学参数尚未在本机实物上辨识。** 假定底座水平、机械臂竖直安装、使用原配末端且未抓取负载；变更末端质量、安装方向或零位后需重新核对模型。

运行脚本即使能选中臂的 J1–J6 并进入拖动控制，没有 `--enable` 开关或只读分支。先在空载、有人托稳的条件下测试单臂，再扩展到双臂。电机必须事先处于 MIT 模式，主控制脚本不会为了切换模式而先自动失能：

```bash
python rebot_hand_guiding.py --side right
python rebot_hand_guiding.py --side left
python rebot_hand_guiding.py --side both
```

只需要查询角度时，使用 `python rebot.py read right --watch`。

每次只运行其中一条。默认持续运行；在运行程序的终端中按 **空格、Esc、Q 或 Ctrl-C** 触发软急停。程序结束拖动控制，先将当前姿态、重力前馈和正的 `kp/kd` 写入电机侧 MIT 位置保持，再关闭 CAN 连接，**保持电机使能，不自动归位、不失能**。`--duration` 到时、普通报错退出以及处理到的 SIGTERM/SIGHUP 也使用同一保持清理路径；一侧保持失败仍会尝试另一侧。这里的软急停是停止拖动并保持位置，不是安全认证的硬件急停。

**主脚本 `rebot_hand_guiding.py` 和共享通信模块 `rebot_mit.py` 均没有主动失能路径，也不导入归位模块。** 拖动控制不改写电机 ID/硬件原点，不发送参数持久化指令，不改变第七路夹爪的控制状态。运行前退出其他实体控制程序，避免同时写入同一电机。

软急停后，如需归位并卸力，只需运行一条命令，将 J1–J6 缓慢移动到保存的 `reference_rad`；全部所选臂到位后，自动失能各臂的七个电机（含夹爪）。归位与失能实现统一位于 `rebot_home.py` 中。运行前准备好参考姿态处的支撑，失能后电机不再提供支撑，夹爪也会卸力：

```bash
python rebot_home.py --side right
python rebot_home.py --side both --speed 0.1
```

归位采用关节空间最小 jerk 插值，同时发送解析的目标位置与速度；位置/速度 PD 在电机侧 MIT 环执行，主机只计算重力前馈，默认计划峰值速度 0.3 rad/s，可用 `--speed` 调整；它不提供避障，需事先确认回到参考姿态的路径畅通。计划轨迹结束后，全部所选臂六轴的位置误差需连续 0.5 秒不超过 0.05 rad（约 2.9°），才进入失能步骤。归位运动不改变夹爪目标；成功后的失能会释放夹爪。按空格、Esc、Q、Ctrl-C 或收到 SIGTERM/SIGHUP 中止归位时，保持当时姿态并跳过失能；报错同样不会继续失能。中止返回状态码 130，归位或失能出错返回 2，全部完成返回 0。归位脚本与拖动脚本需先后运行，不能同时控制同一机械臂。

失能由归位模块内部的 `disable()` 实现，不修改电机运行模式或其他参数。归位成功后，先结束运动连接的位置保持清理，再新建连接核验 UID 并逐电机确认失能，避免清理阶段在失能后再次发送位置指令。任一电机失能发送或确认失败时，仍尝试其余电机并最终报错。

启动时逐轴等待运行状态 `mode=2`，成功后打印 `enable confirmed (mode=2)`；电机若已在运行，则带重力补偿接管，打印 `running hold adopted (mode=2)`，不重复使能。确认超时会报告期望模式和最后观察到的模式。退出时的 `native position hold confirmed; enabled` 表示该轴已确认位置保持。

控制过程：

- `STARTUP`：以启动时的当前位置作为保持目标，前 2 秒保留位置保持和完整模型重力补偿，随后进入普通 `HOLD`；不把零姿态当作运动目标，不重新从零力矩渐入。这样再次运行时不会撤掉上次退出留下的支撑力。
- `DRAG`：受推后释放位置保持，仅输出重力补偿和速度阻尼；目标随当前位置更新。
- `HOLD`：滤波后的六轴最大速度低于 0.03 rad/s 并持续 0.5 秒后，记录当前位置，保持刚度在 0.3 秒内渐入。任一轴速度达到 0.12 rad/s，或偏离保持目标达到 0.035 rad（约 2°）时，再次进入 `DRAG`。这根据运动推断是否有人推动，没有手部接触传感器。

拖动和归位不设速度自动停机检查；归位不设跟踪误差停机或到位超时。归位若跟不上目标，会继续发送指向参考姿态的原生 MIT 位置/速度指令，直到满足到位判定或用户手动停止。空格、Esc、Q、Ctrl-C 停止控制并保持使能。静止速度、释放速度和位置偏差参数用于切换拖动/保持状态，不触发程序退出。

拖动脚本的总力矩为 `g(q) × gravity_scale + Kp × (保持角度 − 当前角度) − Kd × 估计速度`，在主机侧计算并对**总和**逐轴限幅，通过 MIT 的前馈力矩字段发送；电机侧 `kp`、`kd` 均为 0，避免叠加未计入限幅的位置力矩。速度由位置和实际采样时间差分、低通得到，不依赖 RS 固件的速度反馈。默认最大力矩为 `[3, 15, 10, 3, 2, 1]` N·m；若计算所需的重力补偿本身已超过限值，则停止，不把补偿不足当作正常漂浮。

设置见 `rebot_hand_guiding.yaml`。共享 CAN 通信层仅记录温度，不因温度读数退出；`max_temperature_c` 仅为兼容旧配置保留，不再生效，电机上报的故障仍会使程序退出。`hold_kp` 决定保持软硬，`damping` 决定拖动阻力，`gravity_scale` 修正各轴模型偏差；数值是实验初值，不代表本机已调好。可用 `--settings /path/to/settings.yaml` 加载另一组参数；`--gravity-scale 0.5` 可额外减半补偿用于有人托住时的初步试验，此时不会充分抵消自重。不要通过单纯提高刚度掩盖零位、方向或重力模型错误。

通信与退出约束：

- 启动前核验所选机械臂的全部七个 UID，只控制六个机械臂关节；逐项检查已处于 MIT 模式，预置保持参数后才使能。不自动发送停止/复位或模式切换指令。
- 默认请求 50 Hz；样本间隔超过 0.10 秒、读取开始距发送力矩超过 0.08 秒、超出模型限位或收到故障报告时结束拖动控制，执行保持清理，不发送失能。
- 为满足“主程序退出不导致超时失能”的软件约定，主动控制前将六轴 CAN 看门狗设为 `0x7028=0` 并逐项回读确认，退出后也不恢复旧的非零超时值。保持改由电机侧位置环继续执行，而不是保留最后一条纯力矩指令。此更改不写入 EEPROM。
- 退出时优先读取最新姿态；读取失败则尝试使用最后验证过的姿态。保持使用配置中的正 `hold_kp`、`damping` 及限幅后的重力前馈。电机侧位置环会另产生位置/阻尼力矩，因此主机侧总力矩限幅的描述仅适用于正常拖动控制，不能将其视为退出保持时的电机总力矩保证。
- SIGKILL 等不能执行 Python 清理的终止方式不属于“退出前转保持”的保证范围；软件不会主动失能，但也不能承诺这类终止后的姿态保持。保持确认失败会明确打印错误，不以失能作为回退动作。

MIT 编码按 [RS06 手册](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS06/RS06User%20Manual251112.pdf) 和 [RS00 手册](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS00/RS00User%20Manual251112.pdf) 的范围实现：RS06 力矩 ±36 N·m、速度 ±50 rad/s；RS00 力矩 ±14 N·m、速度 ±33 rad/s。当前安装的 `robstride-dynamics` 某些编码表与此不同，因此此实验脚本不使用其 MIT 指令编码函数；已有只读扫描和镜像不受影响。

验证状态：离线已检查重力与势能梯度一致性、拖动/保持切换、总力矩限幅，以及模拟 CAN 中的空格/Ctrl-C 急停、到时和异常退出保留使能、单/双臂归位成功后七轴失能、归位中止或报错跳过失能、失能失败报告及归位模块内的七轴失能；另有静态检查防止拖动及共享通信模块调用失能入口。本机双臂只读查询和预计力矩计算此前已通过。用户提供的实机日志已显示启动、拖动、位置保持及手动失能的反馈确认，也暴露过速度和归位跟踪误差退出；删除这些停机检查及合并归位与失能后的版本目前仅通过离线回归，尚未实机复测。

### 静态重力参数辨识

```bash
python rebot_identify.py --side right
```

运行后直接完成预定位、自动采样、拟合和独立验证。只保留两个参数：

| 参数 | 用途 | 默认值 |
| --- | --- | --- |
| `--side` | `right` 或 `left` | `right` |
| `--speed` | 峰值关节速度，rad/s | `0.2` |

**固定采样流程：**先展开 J3 到至少 1.1 rad，再抬 J2 到 0.7 rad、调整腕部，进入 `J2=0.7、J3=1.1 rad` 的抬肘姿态。然后依次让 J2、J3、J4、J5 在固定范围内往返，其他关节保持抬肘姿态，J1 和 J6 不扫描。预定位和返回抬肘的过程不采样。

| 扫描关节 | 目标范围（模型坐标，rad） |
| --- | --- |
| J2 | 0.45–1.45 |
| J3 | 0.65–1.90 |
| J4 | −0.60–0.60 |
| J5 | −0.80–0.80 |

每个关节取 6 个固定角度，从小到大采一遍、再从大到小采一遍；每点稳定 0.7 秒后采样 1 秒。转向处多走 0.08 rad，确保两次从相反方向接近。第一轮共 24 对，16 对训练、8 对留出验证；拟合通过后，用每关节另外 3 个角度进行第二轮独立验证，共 12 对。两次测量始终按整对划分，一方向缺失或稳定窗口不合格时整对剔除。

**拟合只使用当前扫描关节的力矩，其他关节的保持力矩不参与拟合。** 分别在正反两次实测姿态计算回归矩阵，再将矩阵和力矩估计平均，减小方向相关静摩擦的影响。报告记录正反方向差异和平均残差。只辨识 J2–J6 五个降维刚体组的质量比例，固定 CAD 质心，J1 不可观测的比例固定为 1；固定件及闭合夹爪计入所属组。这不是完整惯量、质心、摩擦或末端负载辨识。

运动继续使用已验证顺滑的原生 MIT 位置、速度、正 `kp/kd` 和重力前馈：`kp=[50,150,150,50,50,50]`、`kd=[3,10,10,5,4,4]`。六轴整批发送，解析最小 jerk 轨迹从上一目标连续接续。没有跟踪误差停机或到位超时停机；反馈过期、电机故障仍结束控制并尝试保持。力矩根据实际量化指令、跟踪误差及位置差分速度估计，依赖电机增益标定，固件力矩只用于诊断。

**姿态参考本机 `reBotArm_control_py/tools/gravity_calibration/pd_sweep_id.py` 和对应无接触标定说明。** 采用固定抬肘区域，不做通用碰撞规划。程序检查关节范围和模型重力预算，不能识别实物支撑或环境碰撞；闭合空夹爪、连杆和桌面应保持净空，不能有人手、线缆或其他外力支撑。静态配对不能保证消除所有摩擦和外力。

程序读取 `rebot.local.yaml`、`rebot_hand_guiding.yaml`，输出到 `calibration_runs/<side>/<UTC时间戳>/`。没有使能开关、分步命令或预览模式；围绕当前位置随机取点的规划、`--span`、D-optimal/Halton 选择和 CAD 碰撞规划模块已删除。启动位置只用于连接到抬肘姿态，J1 保持启动角度。

拟合至少需要 12 对训练和 4 对留出数据；独立验证至少 8 对，并检查回归矩阵的秩与条件数。质量比例触及 0.5–2.0 边界、求解失败或验证不合格时不输出最终校准。验证要求被扫描各轴 RMSE 不超过 0.25 N·m、相对原模型恶化不超过 0.02 N·m，整体 RMSE 至少下降 10% 或已不超过 0.05 N·m。J1、J6 不扫描，报告中相应指标为 `null`。

输出包括两轮计划、`session.yaml`、`data.jsonl`、`independent.jsonl` 和 `report.json`。第一轮候选在 `candidate/`，第二轮通过后才在根部生成最终 `calibration.yaml` 和 `calibrated.urdf`，导出后重载核对预测。新日志版本 3、配置版本 2；旧随机计划、未配对日志和旧版配置不再加载。历史实测日志保留追溯，不用于新流程。

空格、Esc、Q、Ctrl-C、SIGTERM/SIGHUP 终止采样并尝试保持使能，中断后不继续后续运动。拟合期间也保持，不自动归位或应用校准。不修改硬件零位、ID、EEPROM 或夹爪状态。成功后，将最终 `calibration.yaml` 通过 `--calibration` 传给 `rebot_hand_guiding.py` 或 `rebot_home.py`；校准绑定单臂 UID、软件零位及原始模型哈希。

### 多路 Webcam 预览

运行 `webcam_viewer.py`，GStreamer 自动发现摄像头、查询支持的采集模式，并为每路设备打开独立预览窗口：

```bash
python webcam_viewer.py                                    # 自动识别设备并选择采集模式
python webcam_viewer.py --cameras 0 2                       # 指定摄像头编号
python webcam_viewer.py --cameras /dev/video0 /dev/video2
```

程序面向 Linux 桌面环境，使用系统 GStreamer 和 PyGObject；Ubuntu 可安装：

```bash
sudo apt install python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-x
```

Conda 等 Python 环境若缺少系统 GStreamer 绑定，程序会自动切换到 `/usr/bin/python3` 执行；也可直接使用 `/usr/bin/python3 webcam_viewer.py`。预览不依赖 OpenCV、Pillow 或 Tkinter。

采集模式使用 Cheese 的自动选择规则：从设备公布的模式中，优先选宽度 ≥640 像素且帧率 ≥15 FPS 的最高分辨率；没有符合条件的模式时选择最高分辨率。同一分辨率优先选择最高帧率。程序不维护已保存的分辨率偏好，不提供手动设置分辨率、帧率、格式或缓冲大小的命令行参数，也不写死某个摄像头的采集分辨率。本机两路 LRCP USB3.0 自动选到 MJPEG 2592×1944 / 23 FPS，该模式已通过双路取帧验证，可保留完整鱼眼视野。

GStreamer 负责采集、解码和显示；预览画面等比例缩小到约 640×480 后显示，窗口可调整大小，缩放不会改变采集视野。窗口标题和启动日志显示设备与选定模式，实际收到第一帧后打印 `first frame received`。按 `Q`、`Esc`、关闭任意预览窗口或按 `Ctrl-C` 退出并释放设备。某一路失败时报告错误，其他设备继续预览；启动后 10 秒仍无首帧时报告超时，所有设备失败时程序退出。多路画面不保证硬件同步。

### Star Arm 遥操作

Star Arm 扫描和左右身份绑定，只使用官方舵机 SDK。

Linux 下如果打开 `/dev/ttyUSB0` 等串口时提示 `Permission denied`，将当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout wubingqian
```

其中 `wubingqian` 替换为实际用户名；`-aG` 会保留原有用户组并追加串口访问组。执行后需注销系统并重新登录，再重新打开 IDE 或终端运行程序。通过 `chmod` 临时修改的设备权限可能在拔插后恢复，加入用户组后通常无需重复修改。

```bash
conda activate rebot_control
python stararm.py scan
python stararm.py bind left /dev/ttyUSB0
python stararm.py bind right /dev/ttyUSB1
python stararm.py calibrate left
python stararm.py calibrate right
python stararm_teleop.py
```

绑定前核对实体左右臂；配置保存到 `stararm.local.yaml`，优先使用 USB 连接路径。
换插口或互换线缆后需重新核对；修改已有绑定加 `--replace`。
扫描和绑定只查询在线 ID、保存本地配置。

先将实物（包括手柄）摆到约定的默认零姿态并保持静止，再执行 `python stararm.py calibrate left`。命令立即采样并保存七个软件参考角到 `stararm.local.yaml`，不打开窗口、不等待 Enter、不逐轴检查方向，也不修改舵机硬件原点。右臂把 `left` 换为 `right`，重新标定加 `--replace`。

如果只需修正主手夹爪闭合零位，先正常退出实体遥操作及其他占用主手串口的程序，将两只主手夹爪捏到闭合端并保持稳定，再执行：

```bash
python stararm.py calibrate left --gripper-only --replace
python stararm.py calibrate right --gripper-only --replace
```

该选项需要已有七路软件标定，仅查询主手夹爪舵机 ID 6，收集 20 个有效样本并保存闭合参考中位数，正常采样约 0.4 秒，保留 J1–J6 和另一侧标定。无效反馈会重新查询，最多尝试 30 次；有效样本不足或采样波动超过 0.2° 时拒绝保存，不使用上次缓存的角度。机械臂六轴无需摆回零姿态；不连接从手 CAN 总线，不发送运动或硬件归零指令。完成后重新启动实体遥操作，加载新的闭合参考。

独立脚本 `python stararm_teleop.py` 默认显示两只 Star Arm 的实时姿态；使用 `--target rebot` 切换为遥操 Three.js 中的两只虚拟 reBot Arm RS，不需要左右臂参数。程序自动读取已有绑定和软件标定，每臂读取七路角度；不连接实体 RS 或 CAN 总线。只更新模型关节位置，不进行动力学积分，当前用于姿态跟随预览。点击页面 `Close viewer` 或在终端按 Ctrl-C 退出。

`--target` 与 `--mode` 独立组合：

```bash
python stararm_teleop.py --target stararm                 # Star Arm 双臂显示；默认
python stararm_teleop.py --target rebot                   # 遥操 Three.js 虚拟 RS 双臂，布局参考 ABC-130K
python stararm_teleop.py --target rebot --mode hover      # 同上，启用实物 Star Arm 悬停
python stararm_teleop.py --no-browser --port 8080         # 手动打开终端打印的本机地址
```

`stararm` 显示沿用原来的 ROS/URDF 关节方向与手柄反向联动。`rebot` 的六轴采用 RS 参考 follower 的固定映射：直接用 SDK 多圈角度减去对应 Star Arm 的软件参考角，保留连续角度（不折回 ±180°），再依次乘以 `[+1, +1, -1, -1, -1, +1]`，转为弧度并裁剪到 RS 模型关节范围。手柄采用参考 follower 的 `×6` 电机角映射，再将 0–270° 线性换算为模型 0–50 mm 的单侧滑动行程；两侧夹爪同步张开。夹爪目前是可视化比例映射，尚未验证实体丝杆的角度—位移关系。

限位在连续角度映射之后执行：到达模型限位后保持在该端，反向回到范围内时继续跟随。旧版本先将角度折回 ±180°，会导致越过 180° 时突然跳向另一端限位；现已去除遥操路径上的这一折回。校准采样和悬停判定仍使用最短角差。

双臂底座布局参考 [ABC-130K 官方 MuJoCo 场景](https://github.com/amazon-far/abc/blob/main/abc_sim/models/yam_bimanual_empty.xml)：双臂底座分别位于 `(0.2525, ±0.31, 0.75)` m，朝向一致，底座相距 620 mm。保留 RS 原始尺寸与关节零位，不缩放模型。场景显示双臂与白色桌板；桌板尺寸为 595 × 1300 mm，桌面高度为 750 mm。

标定命令只读反馈并保存本地配置。遥操使用字符串参数 `--mode` 选择模式：

```bash
python stararm_teleop.py --mode observe  # 自由拖动，释放锁力，不自动悬停；默认模式
python stararm_teleop.py --mode hover    # 自动悬停
```

`observe` 和 `hover` 都在打开 Three.js 页面前直接发送广播卸力指令，不等待 Enter；运行前需托住两只实物臂。`observe` 模式退出不会自动恢复锁力。

双臂悬停规则：先托住两只实物臂，再执行：

```bash
python stararm_teleop.py --mode hover
```

进入悬停：左右臂同时静止约 1 秒，且两个手柄均闭合，才向两臂各自的六个关节发送当前位置保持指令。静止检测仍采用最近 25 帧均值；每臂六轴相邻滤波帧的角度变化绝对值之和均不超过 0.1°，共用一个连续静止计时。手柄动作不参与静止检测，所以两臂已经静止时，闭合两只手柄即可进入悬停。不再要求启动后先移动 2°。

退出悬停：任意一个手柄张开，立即向两臂发送广播卸力指令，不需要手柄往返动作。以标定默认零姿态的手柄读数为闭合基准：偏离不超过 5° 判为闭合，偏离达到 10° 判为张开；中间区间保留前一状态，避免抖动反复切换。手柄 ID 6 始终不锁定。重新进入时仍需两臂同时静止且两手柄闭合。

当前位置保持仍使用原版指令参数：100 ms、加减速各 50 ms、功率字段 4095。Three.js 页面显示实时角度，不增加静止检测滤波的延迟。此规则不再使用电流触发、电流基线或 `--hover-current-ma` 参数。

只有两臂反馈完整时才更新悬停逻辑；缺失反馈会清除静止计时并冻结模型，终端报告失败侧、缺失舵机 ID 和重试次数。短暂缺帧会重试，恢复完整反馈后继续运行；至少连续 3 次读取失败，且距上次完整反馈已超过 2 秒，才停止并执行退出卸力。SDK 单次缺帧读取本身可能耗时约 0.8 秒，不再因一次这样的超时直接退出。此版本未接入按钮板控制。

这属于位置保持，不是连续重力补偿。`hover` 模式点击 `Close viewer`、终端 Ctrl-C 或报错退出时，会在关闭串口前分别发送广播卸力指令；一侧卸力失败仍会尝试另一侧，并在终端报告失败。退出前需托住两只实物臂；物理断线时无法保证卸力指令送达。不重设舵机原点或多圈计数，不修改标定。

模型保存在 `models/stararm.xml` 和 `models/meshes/`，来自 [Star Arm 102-HD 官方 URDF](https://github.com/servodevelop/Star-Arm-102/tree/298fd045d20c1b0419fa9521e90f8f99cb2b1b14/hardware/102-hd/urdf/stararm102-ld-hd-description)。原模型把 J5 导出成固定关节，本项目按同系列 FL/ROS 模型恢复为局部 -Z 转轴。此模型用于运动姿态回放，未作为 HD 动力学模型验证。

RS 模型保存在 `models/rebot_rs.xml` 和 `models/rebot_rs_meshes/`，由参考仓库 `reBot-DevArm/Rebot_Arm_description/RS/mjcf/rs_arm.xml` 整理，保留机械臂网格、原始关节变换和腕部相机外参元数据；原生窗口的灯光、显示配置及独立碰撞网格、驱动和动力学约束已去除；夹爪的三个滑动关节由脚本同步赋值。模型和网格均位于当前仓库，运行不依赖参考仓库路径。
