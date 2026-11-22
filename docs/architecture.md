# 代码架构

## 分层

依赖只能从上往下：

```text
apps/       每个子命令一个文件：解析参数、搭好对象、跑主循环
  │
runtime/    一只臂的会话（启动/指令/退出保持）、伺服线程、双臂 Robot、进程间共享
viewer/     浏览器 3D 视图            data/    录制、切分、导出
policy/     策略客户端                media/   相机采集与视频（系统 Python）
  │
model/      纯数学：运动学、重力、轨迹、夹爪、限位、主从映射。不碰硬件
devices/    硬件协议：RobStride CAN、SocketCAN、Star Arm 串口
bindings.py settings.py paths.py     本机绑定、控制参数、路径
```

```text
rebot/
├── cli.py              `rebot` 命令：分发到 apps，统一把错误变成一行提示和退出码
├── paths.py            仓库内所有路径的唯一来源
├── bindings.py         config/*.local.yaml 的读写与校验 → ArmBinding / LeaderBinding
├── settings.py         config/control.yaml → ControlSettings
├── devices/
│   ├── socketcan.py    CAN 接口发现与配置
│   ├── robstride.py    MotorBus：一只臂的 CAN 传输（查询 + MIT 控制 + 退出保持）
│   └── stararm.py      主手串口：读角度、卸力
├── model/
│   ├── limits.py       关节限位、跟踪误差检查
│   ├── kinematics.py   CartesianIK（Mink + MuJoCo）：FK、IK、自碰撞
│   ├── gravity.py      GravityModel（Pinocchio）、GravitySupport
│   ├── gravity_fit.py  重力辨识的离线部分：扫掠计划、数据筛选、拟合
│   ├── trajectory.py   最小 jerk 轨迹、TrajectoryTracker、运行日志
│   ├── guiding.py      拖动示教控制律
│   ├── gripper.py      夹爪上位机 PD + 力矩限幅
│   └── leader_map.py   主手角度 → 从臂关节目标
├── runtime/
│   ├── arm.py          Arm：一只从臂的完整会话（下文）
│   ├── worker.py       ArmWorker：后台线程按固定频率伺服最新目标
│   ├── robot.py        Robot：双臂末端级接口 state() / command()
│   ├── console.py      停止：StopSignal、按键、信号
│   ├── ipc.py          多进程共享的“最新一帧”与心跳
│   └── timing.py       定频循环与时序统计
├── viewer/             server.py（HTTP+WebSocket）、scene.py、ee.py、dual.py
├── data/               recording.py、debug.py、process.py、previews.py、reset_detection.py、synchronous.py、contract.py
├── media/              recorder.py、capture.py、mjpeg.py、record_preview.py、video_tools.py、policy_capture.py
├── policy/             client.py（动作时间线、异步推理）、cameras.py
└── apps/               arm_setup、leader_setup、leader_view、mirror、guide、home、teleop、
                        collect、ee、policy、identify、process、cameras
```

## 核心对象：`Arm`

所有驱动电机的命令都通过 `runtime/arm.py` 的 `Arm`。它把“安全地接管一只臂”这件事只实现一次：

```python
with Arm(binding, settings, gravity=model, ...) as arm:   # 打开 CAN、核验 UID、读姿态、装好退出保持
    arm.prepare()                                         # 确认 MIT 模式、关看门狗
    arm.enable()                                          # 重新读姿态，在当前位置使能
    ...                                                   # 下面三种指令之一
# 退出 with：电机切到当前位置保持，保持使能。Arm 永远不会失能电机。
```

| 指令 | 用途 | 使用者 |
| --- | --- | --- |
| `sample()` + `command_torque(tau)` | 纯力矩 | guide |
| `sample()` + `command_trajectory(record)` | 位置 + 速度前馈，跟轨迹 | home、identify |
| `servo(q)` | 跟最新位置目标，用指令回复当反馈 | teleop、ee、policy |

接口上的关节角都是**相对软件零位**的弧度；原始电机角只在 `Arm` 内部出现。

各命令的区别只是构造参数：

| 命令 | 重力前馈 | 夹爪 | 反馈策略 | 谁调用 `servo` |
| --- | --- | --- | --- | --- |
| guide | 有 | — | — | — |
| home / identify | 有 | — | — | — |
| ee | 有 | — | `strict`：缺一次回复就停 | `ArmWorker` 线程，250 Hz |
| policy | 有 | 有 | `strict` | `ArmWorker` 线程，250 Hz（经 `Robot`） |
| teleop / collect | 有 | 可选，collect 需要夹爪 | `tolerant`：每轴重试 15 次 | 每臂一个进程，200 Hz |

`servo()` 一个周期：检查目标与上次反馈的误差 → 算重力前馈和夹爪力矩 → 发一批 MIT 指令并收齐回复 → 校验姿态、更新退出保持 → 再检查一次误差。

## 两种并发结构

**线程（ee、policy）**：IK 慢且不定时，所以 `ArmWorker` 线程独占 CAN 按 250 Hz 重发最新目标，规划循环只读写快照，两边互不等待。

**进程（teleop、collect）**：主手串口一次读取可能阻塞很久，所以每个设备一个 `spawn` 进程，之间只共享“最新一帧”（`ipc.SharedFrame`），不排队。主进程 `Supervisor` 分阶段放行：全部就绪 → 准备 → 全部使能 → 运行；任何一个进程出事，全部停止并保持。

## 约定

- **路径**只从 `rebot.paths` 取，不用 `Path(__file__)` 拼。
- **命令**是 `apps/<name>.py` 里的 `main(argv=None, prog=None)`：只解析参数、调用函数、返回退出码。出错直接抛异常，由 `cli.py` 统一打印（退出码 2；`Ctrl-C` 为 130）。
- **`media/` 在系统 Python 下运行**（GStreamer 绑定只在那里）。它只能 import 标准库、`gi` 和 `rebot.paths`，并通过 `paths.system_python("rebot.media.xxx", ...)` 以子进程启动。
- **只有 `apps/home.py` 可以发失能指令**，有测试看守（`tests/test_guide_home.py`）。
- 可注入：硬件相关函数都接受 `bus_factory=` / `stop=`，测试用 `tests/fakes.py` 的 `VirtualMotors` 代替 CAN。
- 代码和注释用英文，文档用中文。

## 加一个新命令

1. 写 `rebot/apps/foo.py`，提供 `main(argv=None, prog=None)`。
2. 在 `rebot/cli.py` 的 `COMMANDS` 里加一行。
3. 要驱动电机就用 `Arm`（参考 `apps/guide.py`，约 100 行）；控制律放 `model/`，保持不依赖硬件，便于单测。
4. 在 `tests/` 里用 `fakes.VirtualMotors` 写生命周期测试：启动顺序、停止后是否保持、出错是否不使能。

## 测试

全部离线，不连接实机：虚拟 CAN 固件模型、虚拟串口、GStreamer 测试画面；遥操作测试会真的拉起 `spawn` 进程。

```bash
python -m unittest discover -s tests
python -m unittest discover -s tests -p 'test_teleop.py' -v
PYTHONPATH=. /usr/bin/python3 -m unittest discover -s tests -p 'test_cameras.py'
```
