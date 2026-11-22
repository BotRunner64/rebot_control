# 硬件绑定与标定

目标：告诉程序“哪块 CAN 适配器、哪七个电机是左臂”，以及“参考姿态下每个电机的角度是多少”。主手同理。

## 从臂 reBot Arm RS

每臂一块独立的 USB-CAN 适配器（两臂电机 ID 相同，不能并到一条总线）。每臂 7 个电机：ID 1–6 为 J1–J6，ID 7 为夹爪；主机 ID `0xFD`，1 Mbps。

```bash
rebot arm scan                 # 自动把 CAN 口配置为 1 Mbps 并扫描 ID 1–7
rebot arm bind left can0       # 以实物为准核对左右，接口编号不代表左右
rebot arm bind right can1
```

`scan` 需要时会用 `sudo ip link` 配置接口（只有这三条网络命令提权）。重启或重新插拔适配器后再跑一次 `scan` 即可。

绑定记录适配器的 USB 路径和 7 个电机的 UID。之后接口编号变了也能找到；**换 USB 口、换适配器或换机械臂需要重新绑定**（加 `--replace`，旧标定同时清除）。

把机械臂摆到参考姿态（与主手参考姿态对应，夹爪闭合），托稳不动：

```bash
rebot arm calibrate left
rebot arm calibrate right
rebot arm read left --watch    # 查看原始角度和相对参考的角度（度）
```

- 标定读取 10 组角度，任一电机抖动超过 2° 会拒绝保存。重新标定加 `--replace`。
- 只修正夹爪闭合位置：`rebot arm calibrate right --gripper-only --replace`（夹爪完全闭合并静止）。
- 这是**软件零位**，只写本机配置，不改电机硬件零位。出厂零位未设置的机械臂需先按 [RS 快速入门](https://wiki.seeedstudio.com/cn/rebot_b601_rs_getting_started/) 完成。

这些命令只读：不使能、不卸力、不运动、不写电机参数。

## 主手 Star Arm

```bash
rebot leader scan                       # 扫描 CH340/CH341 串口上的舵机 ID 0–7
rebot leader bind left /dev/ttyUSB0
rebot leader bind right /dev/ttyUSB1
rebot leader calibrate left             # 摆到参考姿态，手柄闭合
rebot leader calibrate right
```

- 绑定保存 `/dev/serial/by-path/...` 稳定路径；换 USB 口需重新绑定。
- 只修正手柄闭合位置：`rebot leader calibrate left --gripper-only --replace`。
- 预览主手姿态：`rebot leader view`；用主手驱动虚拟 reBot：`rebot leader view --target rebot`。
  `--mode hover` 时双臂静止且手柄闭合会锁住主手，张开任一手柄解锁。

## 检查标定是否正确

```bash
rebot mirror
```

浏览器里的双臂应与实物姿态一致。只读，不改变电机状态；实体臂需处于可拖动状态。

## 配置文件

| 文件 | 内容 | 谁写 |
| --- | --- | --- |
| `config/arm.local.yaml` | 从臂绑定与 `reference_rad` | `rebot arm` |
| `config/leader.local.yaml` | 主手绑定与 `reference_deg` | `rebot leader` |
| `config/control.yaml` | 控制参数与安全阈值 | 手工编辑 |
| `config/gravity.left.local.yaml`、`config/gravity.right.local.yaml` | 对应臂的重力补偿标定 | 从 `rebot identify` 的结果目录复制 `calibration.yaml` |
| `config/gripper.yaml` | 夹爪增益、力矩上限、闭合预压 | 手工编辑 |

`*.local.yaml` 不进 Git。所有命令都可用 `--config` 指定其他绑定文件。

## 常见故障

**`scan` 显示 `IDs []`**

1. 收到 0 帧：检查机械臂 48V 电源、CAN 线、终端电阻。USB-CAN 接上不代表电机有电。
2. 有流量但没有匹配回复：检查电机 ID 和协议配置。
3. 接口是 UP、没有错误计数，但就是收不到回复：退出其他控制程序，**拔插电脑端的 USB-CAN**（不要带电插拔电机线束），再 `rebot arm scan`。本机遇到过这种情况，重新拔插即恢复；原因未确定。

**`Bound CAN adapter is missing or ambiguous`**：适配器换了 USB 口或没插好。插回原口，或重新 `bind --replace`。

**`motor UIDs or online IDs changed`**：线接到了另一只臂，或有电机掉线。先 `rebot arm scan` 看在线 ID。

**`motors must already be in MIT mode`**：程序不会自动切换电机模式或失能。用厂商工具把电机设为 MIT 模式。

**`Follower pose is outside the joint limits`**：当前姿态超出限位，多半是软件零位不对或姿态离谱。报错里有每个关节的角度，核对后调整姿态或重新标定。
