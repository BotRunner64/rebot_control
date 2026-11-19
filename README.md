# rebot_control

reBot Arm RS（B601-RS）双臂的控制、Star Arm 主手遥操作、数据采集与远端策略控制。

所有功能通过一个命令 `rebot <子命令>` 使用。

## 安装

```bash
conda create -n rebot_control python=3.11 && conda activate rebot_control
pip install -e '.[ee,policy,data]'          # 在仓库根目录执行
```

- `ee`：末端位姿控制与策略控制所需的 IK 依赖（mink、quadprog）。
- `policy`：远端策略通讯（msgpack）。同时安装协议包：`pip install -e ./policy_link`。
- 相机采集使用系统自带的 `/usr/bin/python3` 和 GStreamer，需要的系统包见 [数据流程](docs/data.md#系统依赖)。
- `data` 安装采集和切分导出所用的 LOOM-data 契约实现。

## 第一次使用

每只机械臂只需做一次，结果保存在本机的 `config/*.local.yaml`（不进 Git）。详见 [硬件绑定与标定](docs/setup.md)。

```bash
# 从臂 reBot：扫描 CAN、绑定左右、在参考姿态下标定软件零位
rebot arm scan
rebot arm bind left can0
rebot arm bind right can1
rebot arm calibrate left
rebot arm calibrate right

# 主手 Star Arm：同样三步
rebot leader scan
rebot leader bind left /dev/ttyUSB0
rebot leader bind right /dev/ttyUSB1
rebot leader calibrate left
rebot leader calibrate right
```

## 命令一览

| 命令 | 作用 | 会驱动电机 |
| --- | --- | --- |
| `rebot arm scan\|bind\|calibrate\|read` | 从臂扫描、绑定、软件零位标定、读角度 | 否 |
| `rebot leader scan\|bind\|calibrate\|view` | 主手扫描、绑定、标定、3D 预览 | 否（`view --mode hover` 会锁主手） |
| `rebot mirror` | 把实体双臂姿态同步到浏览器 3D 视图 | 否 |
| `rebot cameras` | 浏览器预览相机 | 否 |
| `rebot guide` | 拖动示教：重力补偿 + 阻尼，静止后保持 | 是 |
| `rebot home` | 回到参考姿态，然后**失能**全部电机 | 是 |
| `rebot teleop` | Star Arm 遥操作实体双臂和夹爪 | 是 |
| `rebot collect` | 遥操作并录制数据 | 是 |
| `rebot ee` | 末端位姿控制（默认仿真，`--hardware` 上实机） | 仅 `--hardware` |
| `rebot policy` | 连接远端策略服务做双臂控制 | 是 |
| `rebot identify` | 自动重力参数辨识 | 是 |
| `rebot process` | 回放、切分、标注并导出 LOOM LeRobot v3.0 数据集 | 否 |

每个命令都支持 `--help`。

## 常用流程

```bash
# 遥操作（双臂 + 夹爪）
rebot teleop

# 采集 → 切分标注 → 导出训练数据
rebot collect --dataset datasets/cup --camera-viewer
rebot process --dataset datasets/cup --output datasets/cup_loom

# 用完后归位并卸力
rebot home
```

## 安全约定

这些约定对所有会驱动电机的命令都成立：

- **停止**：终端按 `空格`、`Esc`、`Q` 或 `Ctrl-C`。停止后电机**保持使能并锁在当前位置**，不会掉下来，也不会自动归位。
- **卸力只有一个入口**：`rebot home`。它先回到参考姿态，到位后才失能。运行前准备好支撑。
- 启动时先核验电机身份（UID）和当前姿态，任何一项不符都不会使能。
- 目标与实测姿态相差超过 45°（`config/control.yaml` 的 `max_tracking_error_deg`）会立即停止并保持。
- 没有避障，没有速度限制。首次运行新功能时空载、有人托稳、留出空间。
- 这是软件急停，不是安全认证的硬件急停。断电、`kill -9`、CAN 断线时不保证保持成功。

## 文档

| 文档 | 内容 |
| --- | --- |
| [硬件绑定与标定](docs/setup.md) | CAN/串口接线、绑定、软件零位、常见故障 |
| [控制功能](docs/control.md) | 拖动、归位、遥操作、末端控制、策略控制、重力辨识 |
| [数据流程](docs/data.md) | 采集、离线切分标注、导出 LeRobot |
| [数据格式](docs/data-format.md) | 录制文件结构、坐标系、时间戳定义 |
| [策略链路协议](docs/policy-link.md) | 控制端与策略服务之间的消息格式 |
| [代码架构](docs/architecture.md) | 目录结构、分层、如何加新功能 |

## 目录

```text
rebot/          Python 包（见 docs/architecture.md）
config/         control.yaml、gripper.yaml；*.local.yaml 为本机绑定
models/         控制与显示用的 MJCF / URDF / 网格
web/            浏览器页面（3D 视图、相机预览、数据处理）
policy_link/    策略链路协议包，策略服务端单独安装
tests/          离线测试（虚拟 CAN / 串口，不连接实机）
runs/           运行日志与辨识结果（不进 Git）
datasets/       数据盘软链接（不进 Git）
```

硬件 CAD、打印件在同级的 `../rebot_hardware/`。

## 测试

```bash
python -m unittest discover -s tests                                   # 全部离线测试
PYTHONPATH=. /usr/bin/python3 -m unittest discover -s tests -p 'test_cameras.py'   # 需要 GStreamer 的相机测试
```
