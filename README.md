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

这些命令只发送设备 ID 查询和位置参数 `0x7019` 读取，不发送使能、卸力、运动、模式切换、硬件零位或参数写入；退出仅关闭 CAN socket。它们不释放已有锁力，也不提供重力补偿。缺失反馈、UID 不匹配、无效角度或收到故障报告时停止当前操作；不会用零值补齐。双臂按实物核对后分别执行，不要求同时连接。

此软件参考标定与官方示例向电机发送的零位设置不同。尚未完成出厂/硬件零位设置的机械臂，需要按 [RS 快速入门](https://wiki.seeedstudio.com/cn/rebot_b601_rs_getting_started/) 完成对应步骤；本脚本的本地偏置不能替代该步骤。参考角也不会自动应用到现有 `stararm_teleop.py`：该脚本的 Rebot 目标仍由 StarArm 控制 MuJoCo 模型，不控制实体 RS。

### 实体 reBot 双臂姿态同步到 MuJoCo

完成两臂的 `bind` 和 `calibrate` 后，运行新脚本：

```bash
conda activate rebot_control
python rebot_mirror.py
```

程序读取 `rebot.local.yaml`，按 USB 路径解析两块 CAN 适配器，核验两臂七个电机的 UID，再打开 MuJoCo 双臂场景。手动拖动实体 Rebot Arm 左右臂，仿真中的对应机械臂跟随显示。沿用现有 RS 模型、桌面和双臂布局。若 CAN 尚未启动，先运行 `python rebot.py scan`。

实体臂应先处于可手动拖动状态，并在拖动时托稳。此脚本只查询角度，启动、运行和退出均不发送使能、卸力、运动、模式切换或硬件零位指令，也不提供重力补偿。如果实物已锁力，脚本不会自动解除锁力。关闭 MuJoCo 窗口或 Ctrl-C 退出并关闭两路 CAN 连接，不改写本地标定。

六轴按 `当前电机角度（弧度）− reference_rad` 直接赋给对应模型关节，不应用 StarArm 的方向映射或角度倍率，不折回 ±π。标定时的实体参考姿态需对应模型零姿态，否则显示会有偏差；超出模型关节范围时裁剪并在终端提示。这里只更新关节位置并计算运动学，不进行动力学积分。

夹爪用第七路电机相对于闭合参考角的变化显示开合，默认将 0–270° 映射为模型单侧 0–50 mm 行程，不使用 StarArm 手柄的 `×6` 倍率。该角度—行程关系仅用于比例预览，尚未实测标定，可按需要调整：

```bash
python rebot_mirror.py --fps 30 --gripper-open-deg 270
python rebot_mirror.py --config /path/to/rebot.local.yaml
```

默认请求 30 Hz，每次顺序读取左、右臂各七路角度，不保证硬件同步；实际刷新率取决于 CAN 反馈速度。仅在两臂反馈均完整时更新姿态，任一侧读取失败则冻结两只模型并报告失败侧，短暂缺帧会重试；连续失败至少三次且距上次完整反馈已超过两秒时退出。收到电机故障报告则立即退出，不等待重试。

### 手动引导与位置保持（Hand Guiding）

`rebot_hand_guiding.py` 实现手动引导（Hand Guiding）：手动拖动时提供重力补偿与阻尼，静止后保持位置；当前不记录或回放轨迹。支持 `--side right|left|both`，默认仅选右臂。它与只读的 `rebot_mirror.py` 独立运行，本身不打开 MuJoCo 窗口。参考仓库的重力补偿及速度锁止示例提供了设计依据，但本脚本使用独立的通信和控制实现，不调用示例的自动归零流程。

重力模型来自 `models/rebot_rs_dynamics.urdf`，保留参考仓库的惯性参数，由 Pinocchio 计算六轴 `g(q)`；夹爪移动分支固定在闭合位置，质量仍计入整臂。模型坐标为 `电机原始角度 − reference_rad`，正方向与姿态镜像一致。**使用前需核对实物参考姿态与模型零姿态、逐轴正方向；当前两臂零姿态已由用户确认一致，但动力学参数尚未在本机实物上辨识。** 假定底座水平、机械臂竖直安装、使用原配末端且未抓取负载；变更末端质量、安装方向或零位后需重新核对模型。

默认仅做只读检查，查询 UID 和角度，计算并打印预计力矩，不使能、不卸力、不切换模式、不设置看门狗，也不发送力矩：

```bash
python rebot_hand_guiding.py --side both --duration 3
```

核对无误后，先在空载、有人托稳的条件下测试单臂，再扩展到双臂。加 `--enable` 才会使能选中臂的 J1–J6 并下发力矩。电机必须事先处于 MIT 模式，主控制脚本不会为了切换模式而先自动失能：

```bash
python rebot_hand_guiding.py --side right --enable
python rebot_hand_guiding.py --side left --enable
python rebot_hand_guiding.py --side both --enable
```

每次只运行其中一条。默认持续运行；在运行程序的终端中按 **空格、Esc、Q 或 Ctrl-C** 触发软急停。程序结束拖动控制，先将当前姿态、重力前馈和正的 `kp/kd` 写入电机侧 MIT 位置保持，再关闭 CAN 连接，**保持电机使能，不自动归位、不失能**。`--duration` 到时、普通报错退出以及处理到的 SIGTERM/SIGHUP 也使用同一保持清理路径；一侧保持失败仍会尝试另一侧。这里的软急停是停止拖动并保持位置，不是安全认证的硬件急停。

**主脚本 `rebot_hand_guiding.py`、`rebot_home.py` 和共享通信模块 `rebot_mit.py` 均没有主动失能路径，也不导入失能脚本。** 不改写电机 ID/硬件原点，不发送参数持久化指令，不改变第七路夹爪的控制状态。运行前退出其他实体控制程序，避免同时写入同一电机。

软急停后，如需归位，手动运行独立脚本，将 J1–J6 缓慢移动到保存的 `reference_rad`；归位完成后同样保持使能，不接着失能：

```bash
python rebot_home.py --side right
python rebot_home.py --side both --speed 0.1
```

归位采用关节空间最小 jerk 插值，默认计划峰值速度 0.1 rad/s，最大允许 0.2 rad/s；它不提供避障，需事先确认回到参考姿态的路径畅通。按空格、Esc、Q 或 Ctrl-C 中止归位时，保持当时姿态。归位脚本与拖动脚本需先后运行，不能同时控制同一机械臂。

**只有明确手动运行下面的独立脚本才会主动失能**，J1–J6 的失能指令仅位于此文件中；运行前先放稳或支撑机械臂：

```bash
python rebot_disable.py --side right
python rebot_disable.py --side both
```

若启动控制时报告当前不是 MIT 模式，可在已支撑的条件下手动运行 `python rebot_disable.py --side right --prepare-mit`：先确认失能，再设置并回读 MIT 模式。这一步不会由主程序自动调用。

启动时逐轴等待运行状态 `mode=2`，成功后打印 `enable confirmed (mode=2)`；电机若已在运行，则带重力补偿接管，打印 `running hold adopted (mode=2)`，不重复使能。确认超时会报告期望模式和最后观察到的模式。退出时的 `native position hold confirmed; enabled` 表示该轴已确认位置保持。

控制过程：

- `STARTUP`：以启动时的当前位置作为保持目标，前 2 秒保留位置保持和完整模型重力补偿，随后进入普通 `HOLD`；不把零姿态当作运动目标，不重新从零力矩渐入。这样再次运行时不会撤掉上次退出留下的支撑力。
- `DRAG`：受推后释放位置保持，仅输出重力补偿和速度阻尼；目标随当前位置更新。
- `HOLD`：滤波后的六轴最大速度低于 0.03 rad/s 并持续 0.5 秒后，记录当前位置，保持刚度在 0.3 秒内渐入。任一轴速度达到 0.12 rad/s，或偏离保持目标达到 0.035 rad（约 2°）时，再次进入 `DRAG`。这根据运动推断是否有人推动，没有手部接触传感器。

拖动和归位不设速度自动停机检查；归位不设跟踪误差停机或到位超时。归位若跟不上目标，会继续朝参考姿态施加限幅后的控制力矩，直到满足到位判定或用户手动停止。空格、Esc、Q、Ctrl-C 停止控制并保持使能。静止速度、释放速度和位置偏差参数用于切换拖动/保持状态，不触发程序退出。

总力矩为 `g(q) × gravity_scale + Kp × (保持角度 − 当前角度) − Kd × 估计速度`，在主机侧计算并对**总和**逐轴限幅，通过 MIT 的前馈力矩字段发送；电机侧 `kp`、`kd` 均为 0，避免叠加未计入限幅的位置力矩。速度由位置和实际采样时间差分、低通得到，不依赖 RS 固件的速度反馈。默认最大力矩为 `[3, 15, 10, 3, 2, 1]` N·m；若计算所需的重力补偿本身已超过限值，则停止，不把补偿不足当作正常漂浮。

设置见 `rebot_hand_guiding.yaml`。`hold_kp` 决定保持软硬，`damping` 决定拖动阻力，`gravity_scale` 修正各轴模型偏差；数值是实验初值，不代表本机已调好。可用 `--settings /path/to/settings.yaml` 加载另一组参数；`--gravity-scale 0.5` 可额外减半补偿用于有人托住时的初步试验，此时不会充分抵消自重。不要通过单纯提高刚度掩盖零位、方向或重力模型错误。

通信与退出约束：

- 启动前核验所选机械臂的全部七个 UID，只控制六个机械臂关节；逐项检查已处于 MIT 模式，预置保持参数后才使能。不自动发送停止/复位或模式切换指令。
- 默认请求 50 Hz；样本间隔超过 0.10 秒、读取开始距发送力矩超过 0.08 秒、超出模型限位、温度达到 65°C 或收到故障报告时结束拖动控制，执行保持清理，不发送失能。
- 为满足“主程序退出不导致超时失能”的软件约定，主动控制前将六轴 CAN 看门狗设为 `0x7028=0` 并逐项回读确认，退出后也不恢复旧的非零超时值。保持改由电机侧位置环继续执行，而不是保留最后一条纯力矩指令。此更改不写入 EEPROM。
- 退出时优先读取最新姿态；读取失败则尝试使用最后验证过的姿态。保持使用配置中的正 `hold_kp`、`damping` 及限幅后的重力前馈。电机侧位置环会另产生位置/阻尼力矩，因此主机侧总力矩限幅的描述仅适用于正常拖动控制，不能将其视为退出保持时的电机总力矩保证。
- SIGKILL 等不能执行 Python 清理的终止方式不属于“退出前转保持”的保证范围；软件不会主动失能，但也不能承诺这类终止后的姿态保持。保持确认失败会明确打印错误，不以失能作为回退动作。

MIT 编码按 [RS06 手册](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS06/RS06User%20Manual251112.pdf) 和 [RS00 手册](https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS00/RS00User%20Manual251112.pdf) 的范围实现：RS06 力矩 ±36 N·m、速度 ±50 rad/s；RS00 力矩 ±14 N·m、速度 ±33 rad/s。当前安装的 `robstride-dynamics` 某些编码表与此不同，因此此实验脚本不使用其 MIT 指令编码函数；已有只读扫描和镜像不受影响。

验证状态：离线已检查重力与势能梯度一致性、拖动/保持切换、总力矩限幅，以及模拟 CAN 中的空格/Ctrl-C 急停、到时和异常退出保留使能、手动归位后保持、独立手动失能；另有静态检查防止控制模块调用失能入口。本机双臂只读查询和预计力矩计算此前已通过。用户提供的实机日志已显示启动、拖动、位置保持及手动失能的反馈确认，也暴露过速度和归位跟踪误差退出；删除这些停机检查后的版本目前仅通过离线回归，尚未实机复测。

### 多路 Webcam 预览

安装依赖后运行 `webcam_viewer.py`，每个摄像头使用独立的 Tkinter 窗口显示。OpenCV 仅负责采集，Pillow 负责画面转换和缩放，不使用 OpenCV 窗口：

```bash
python -m pip install opencv-python-headless numpy Pillow
python webcam_viewer.py                                    # 自动识别设备，1920×1080 / 30 FPS
python webcam_viewer.py --cameras 0 2 4                     # 自定义摄像头编号
python webcam_viewer.py --cameras /dev/video0 /dev/video2
python webcam_viewer.py --cameras 0 1 --width 1920 --height 1080 --fps 30
python webcam_viewer.py --backend v4l2 --width 1920 --height 1080 --fps 30
```

需要桌面图形环境和 Tkinter，可运行 `python -m tkinter` 检查。若缺少 Tkinter，Conda 环境安装 `conda install tk`；Ubuntu 系统 Python 安装 `sudo apt install python3-tk`。已有 `opencv-python` 的环境只需补装 `Pillow`，也能运行；新环境使用上面的 headless 版本，同一环境只保留一种 OpenCV 包。

画面保持宽高比，每路显示设备标识，窗口可独立移动和缩放；打开或读取失败的设备显示黑底及无信号提示，其余设备继续预览。按 `Q`、`Esc`、关闭任意摄像头窗口或 `Ctrl-C` 退出并释放所有摄像头。分辨率和帧率是请求值，实际采集参数取决于设备支持；多路画面不保证硬件同步。Linux 默认查询设备能力并跳过元数据节点；其他系统默认摄像头编号为 `0`、`1`。也可通过 `--cameras` 显式指定设备。

每路摄像头在独立线程中持续读取，只保留最新帧；显示窗口不会等待其他摄像头采集。默认使用已在本机两路摄像头同时验证的 1920×1080 / 30 FPS 模式，Linux 使用 V4L2，其他系统自动选择后端；保留后端缓冲大小和设备格式。用 `--backend auto|v4l2|ffmpeg` 选择后端；启动日志会打印实际后端。`--width` 和 `--height` 修改采集分辨率；预览区域初始约为 640×480，画面随窗口尺寸按比例缩放并居中。仅在需要时用 `--fps`、`--fourcc MJPG`、`--buffer-size` 请求修改；不支持的参数会报告实际返回值。启动日志中的设备参数不表示已经有画面，收到第一帧后会明确打印 `first frame received`。窗口文字使用 Tk 系统字体，不再需要 OpenCV Qt 字体目录修复。

本机 LRCP USB3.0（`0bda:3535`）公布的 MJPG 模式中，640×480 仅对应 60 FPS，1920×1080 对应 30 FPS。单路实测 V4L2 在 640×480/60 FPS 下，默认 4 帧缓冲和 1 帧缓冲均出现取帧超时；1920×1080/30 FPS、默认缓冲已通过双路同时取帧验证，因此作为默认配置。尚未确定 640×480 模式失败是摄像头固件、驱动还是 OpenCV 的具体兼容性问题。

窗口中的 FPS 是实际读取速率，`Last read` 是最近一次读取完成至今的时间，不代表从曝光到显示的总延迟；超过 1 秒没有新帧时显示无信号。`select() timeout` 表示设备没有及时返回帧，不能通过修改字体修复。先退出预览进程，分别用 `python webcam_viewer.py --cameras /dev/video5` 和对应另一设备的路径测试；如果单路默认设置也超时，重新插拔摄像头并重新识别节点。若仅双路失败，再检查共用 USB Hub、带宽及供电。设备读取异常阻塞时，退出最多等待每路线程 2 秒并报告仍阻塞的设备。

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

独立脚本 `python stararm_teleop.py` 默认显示两只 Star Arm 的实时姿态；使用 `--target rebot` 切换为遥操 MuJoCo 中的两只 reBot Arm RS，不需要左右臂参数。程序自动读取已有绑定和软件标定，每臂读取七路角度；不连接实体 RS 或 CAN 总线。只更新模型关节位置，不进行动力学积分，当前用于姿态跟随预览。关闭窗口或 Ctrl-C 退出。

`--target` 与 `--mode` 独立组合：

```bash
python stararm_teleop.py --target stararm                 # Star Arm 双臂显示；默认
python stararm_teleop.py --target rebot                   # 遥操 MuJoCo RS 双臂，布局参考 ABC-130K
python stararm_teleop.py --target rebot --mode hover      # 同上，启用实物 Star Arm 悬停
```

`stararm` 显示沿用原来的 ROS/URDF 关节方向与手柄反向联动。`rebot` 的六轴采用 RS 参考 follower 的固定映射：直接用 SDK 多圈角度减去对应 Star Arm 的软件参考角，保留连续角度（不折回 ±180°），再依次乘以 `[+1, +1, -1, -1, -1, +1]`，转为弧度并裁剪到 RS 模型关节范围。手柄采用参考 follower 的 `×6` 电机角映射，再将 0–270° 线性换算为模型 0–50 mm 的单侧滑动行程；两侧夹爪同步张开。夹爪目前是可视化比例映射，尚未验证实体丝杆的角度—位移关系。

限位在连续角度映射之后执行：到达模型限位后保持在该端，反向回到范围内时继续跟随。旧版本先将角度折回 ±180°，会导致越过 180° 时突然跳向另一端限位；现已去除遥操路径上的这一折回。校准采样和悬停判定仍使用最短角差。

双臂底座布局参考 [ABC-130K 官方 MuJoCo 场景](https://github.com/amazon-far/abc/blob/main/abc_sim/models/yam_bimanual_empty.xml)：双臂底座分别位于 `(0.2525, ±0.31, 0.75)` m，朝向一致，底座相距 620 mm。保留 RS 原始尺寸与关节零位，不缩放模型。场景显示双臂与白色桌板；桌板尺寸为 595 × 1300 mm，桌面高度为 750 mm。

标定命令只读反馈并保存本地配置。遥操使用字符串参数 `--mode` 选择模式：

```bash
python stararm_teleop.py --mode observe  # 自由拖动，释放锁力，不自动悬停；默认模式
python stararm_teleop.py --mode hover    # 自动悬停
```

`observe` 和 `hover` 都在打开 MuJoCo 窗口前直接发送广播卸力指令，不等待 Enter；运行前需托住两只实物臂。`observe` 模式退出不会自动恢复锁力。

双臂悬停规则：先托住两只实物臂，再执行：

```bash
python stararm_teleop.py --mode hover
```

进入悬停：左右臂同时静止约 1 秒，且两个手柄均闭合，才向两臂各自的六个关节发送当前位置保持指令。静止检测仍采用最近 25 帧均值；每臂六轴相邻滤波帧的角度变化绝对值之和均不超过 0.1°，共用一个连续静止计时。手柄动作不参与静止检测，所以两臂已经静止时，闭合两只手柄即可进入悬停。不再要求启动后先移动 2°。

退出悬停：任意一个手柄张开，立即向两臂发送广播卸力指令，不需要手柄往返动作。以标定默认零姿态的手柄读数为闭合基准：偏离不超过 5° 判为闭合，偏离达到 10° 判为张开；中间区间保留前一状态，避免抖动反复切换。手柄 ID 6 始终不锁定。重新进入时仍需两臂同时静止且两手柄闭合。

当前位置保持仍使用原版指令参数：100 ms、加减速各 50 ms、功率字段 4095。MuJoCo 使用实时角度，不增加静止检测滤波的延迟。此规则不再使用电流触发、电流基线或 `--hover-current-ma` 参数。

只有两臂反馈完整时才更新悬停逻辑；缺失反馈会清除静止计时并冻结模型，终端报告失败侧、缺失舵机 ID 和重试次数。短暂缺帧会重试，恢复完整反馈后继续运行；至少连续 3 次读取失败，且距上次完整反馈已超过 2 秒，才停止并执行退出卸力。SDK 单次缺帧读取本身可能耗时约 0.8 秒，不再因一次这样的超时直接退出。此版本未接入按钮板控制。

这属于位置保持，不是连续重力补偿。`hover` 模式关闭窗口、Ctrl-C 或报错退出时，会在关闭串口前分别发送广播卸力指令；一侧卸力失败仍会尝试另一侧，并在终端报告失败。退出前需托住两只实物臂；物理断线时无法保证卸力指令送达。不重设舵机原点或多圈计数，不修改标定。

模型保存在 `models/stararm.xml` 和 `models/meshes/`，来自 [Star Arm 102-HD 官方 URDF](https://github.com/servodevelop/Star-Arm-102/tree/298fd045d20c1b0419fa9521e90f8f99cb2b1b14/hardware/102-hd/urdf/stararm102-ld-hd-description)。原模型把 J5 导出成固定关节，本项目按同系列 FL/ROS 模型恢复为局部 -Z 转轴。此模型用于运动姿态回放，未作为 HD 动力学模型验证。

RS 模型保存在 `models/rebot_rs.xml` 和 `models/rebot_rs_meshes/`，由参考仓库 `reBot-DevArm/Rebot_Arm_description/RS/mjcf/rs_arm.xml` 整理，仅保留机械臂可视网格与原始关节变换，去除相机、碰撞网格、驱动和动力学约束；夹爪的三个滑动关节由脚本同步赋值。模型和网格均位于当前仓库，运行不依赖参考仓库路径。来源说明见 `models/LICENSE.md`。
