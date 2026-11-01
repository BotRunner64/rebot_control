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

标定命令只读反馈并保存本地配置。遥操脚本默认只读；如果需要软件解锁以便手动拖动，可给 `stararm_teleop.py` 加 `--unlock`，程序会先提示托住两只实物臂并按 Enter，再同时释放锁力。退出不会自动恢复锁力，解锁期间需支撑机械臂。

模型保存在 `models/stararm.xml` 和 `models/meshes/`，来自 [Star Arm 102-HD 官方 URDF](https://github.com/servodevelop/Star-Arm-102/tree/298fd045d20c1b0419fa9521e90f8f99cb2b1b14/hardware/102-hd/urdf/stararm102-ld-hd-description)。原模型把 J5 导出成固定关节，本项目按同系列 FL/ROS 模型恢复为局部 -Z 转轴。此模型用于运动姿态回放，未作为 HD 动力学模型验证。
