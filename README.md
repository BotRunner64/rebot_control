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

独立脚本 `python stararm_teleop.py` 启动双臂遥操，不需要左右臂参数。程序自动读取已绑定、标定的左右实物臂，在同一个 MuJoCo 窗口中生成两个 Star Arm 模型并分别跟随，每臂读取七路角度，关节方向沿用 ROS 驱动到 URDF 的固定映射，手柄两片反向联动。只更新模型关节位置，不运行会改变姿态的动力学积分。缺失反馈不会当作零角度使用；持续失联会停止。关闭窗口或 Ctrl-C 退出。

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
