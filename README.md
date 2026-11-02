## rebot arm rs版的设置，控制，遥操作

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
