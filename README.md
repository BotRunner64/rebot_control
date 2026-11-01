## rebot arm rs版的设置，控制，遥操作

Star Arm 扫描和左右身份绑定，只使用官方舵机 SDK。

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
