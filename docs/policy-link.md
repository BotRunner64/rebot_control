# 策略链路协议（rebot-policy-link/1）

`rebot policy`（控制端）与策略服务之间的消息格式。它与具体策略无关：**观测是各路数据流最近的几行，动作是策略预测的 state 流未来几行**，位姿、开度、单位和坐标的定义见[数据格式](data-format.md)。模型输入名、图像尺寸、动作步数和频率由策略仓库自己的适配层处理，不出现在控制端。

实现是 `policy_link/rebot_policy_link.py`：一个只依赖 `msgpack` 和 `websockets` 的单文件包，含客户端、服务端外壳、校验函数和 mock 策略。

```bash
pip install -e ./policy_link                 # 控制端（本仓库环境）

uv build policy_link --wheel --out-dir /path/to/wheels          # 策略仓库：装 wheel
uv pip install --python /path/to/policy-env/bin/python --no-deps /path/to/wheels/rebot_policy_link-0.1.0-py3-none-any.whl
```

## 传输

WebSocket 二进制帧，内容为普通 MessagePack（不使用 NumPy 扩展，任何语言都能实现）。连接建立后服务端先发元数据，之后每收到一条观测回一条动作；控制端同一时刻最多一个请求在途。服务端发来的文本帧表示出错，内容为错误信息，随后关闭连接。

## 元数据（服务端 → 控制端）

```text
{"protocol": "rebot-policy-link/1",
 "state": ["left_eef", "right_eef", "left_gripper", "right_gripper"],
 "rgb":   ["top", "left_wrist", "right_wrist"],
 "mock":  false, "rtc_prefix_steps": 3}
```

`state` 和 `rgb` 是该策略在每条观测中需要的流。`rebot policy` 的 `--camera NAME=DEVICE` 必须恰好提供 `rgb` 列出的相机。`mock` 为真的服务会被 `rebot policy` 拒绝。实机策略必须声明正整数 `rtc_prefix_steps`，指定每次重规划承诺的动作步数。

## 观测（控制端 → 服务端）

```text
{"timestamp_ns": 1234567890123,
 "instruction": "pick up the cup",
 "state": {
   "left_eef":      {"timestamp_ns": [...], "pose": [[x, y, z, R00, R10, R20, R01, R11, R21, R02, R12, R22]]},
   "right_eef":     {"timestamp_ns": [...], "pose": [[...]]},
   "left_gripper":  {"timestamp_ns": [...], "openness": [0.5]},
   "right_gripper": {"timestamp_ns": [...], "openness": [0.5]}},
 "rgb": {
   "top":        {"timestamp_ns": [...], "jpeg": [<bytes>]},
   "left_wrist": {...}, "right_wrist": {...}}}
```

- 每路流是一张带采集时间的小表：EEF 为 `timestamp_ns` + `pose`（12 个 float64，米，旋转按列展平，各臂自己的基座系），夹爪为 `timestamp_ns` + `openness`（0 闭合，1 张开）。相机流用 `jpeg` 列携带单帧字节，供策略直接解码。
- 每路流的行按时间非递减排列，最后一行最新。格式允许多行以便将来传历史；当前控制端每路只发最新一行，策略端取最后一行。
- 时间戳是控制端主机单调时钟的纳秒，各流保留自己的采集时刻，不对齐、不插值。顶层 `timestamp_ns` 是本次观测的参考时刻（两臂反馈时刻中较早的一个）。
- 图像是采集子进程输出的 JPEG 原样转发，目前最长边不超过 640 像素、质量 85，控制端不解码也不缩放。

RTC 重规划请求增加可选字段 `action_prefix`，结构与下方动作回复相同。每路包含恰好 `rtc_prefix_steps` 个绝对目标，时间戳为当前观测后第 1 至第 d 个 30 Hz 控制时刻。首块省略该字段。

## 动作（服务端 → 控制端）

```text
{"state": {
   "left_eef":      {"timestamp_ns": [t1, t2, ...], "pose": [[...], [...], ...]},
   "right_eef":     {...},
   "left_gripper":  {"timestamp_ns": [t1, t2, ...], "openness": [...]},
   "right_gripper": {...}}}
```

动作与观测的 state 流同构，含义是“这些流在未来时刻应处于的状态”，时间戳使用控制端时钟（由观测的参考时刻推算），必须严格递增且晚于参考时刻。行数和间隔由策略决定。当前 pi05 适配层返回 50 行，第 k 行为参考时刻后 (k+1)/30 秒。

控制端要求回复恰好包含上面四路流，并在本地做与策略无关的检查：数值有限、旋转为正常旋转矩阵、夹爪裁剪到 `[0,1]`、动作块未过期且覆盖到下一次重规划之后。执行时每路流取“时间戳晚于当前时刻的第一行”作为目标，不插值；因此目标间隔明显大于 1/30 秒的策略会产生阶跃目标，应在策略端自行加密。

## 接入一个新策略

在策略仓库里写一个 `infer(observation) -> reply` 函数并交给外壳：

```python
import rebot_policy_link as link

def infer(observation):
    ...  # 取各流最后一行、解码 JPEG、推理
    times = link.future_timestamps(observation["timestamp_ns"], count=50, hz=30)
    return {"state": {"left_eef": {"timestamp_ns": times, "pose": poses}, ...}}

link.serve(infer, link.metadata(rgb=["top", "left_wrist", "right_wrist"]), "127.0.0.1", 8000)
```

外壳负责握手、观测与回复的结构校验、跨连接串行推理和错误回传。
