# 遥操作数据采集与 reBot 源契约

本实现以用户提供的 `LOOM-data/docs/contract/export-contract.md` 为目标格式。采用采集和处理分离流程时，一次遥操作运行保存一个连续原始 session，物理上沿用 episode 目录格式，退出并校验成功后发布到原始数据集。最终 episode 拆分、instruction 与相机语义命名通过独立的 [离线处理工具](processing.md) 完成。不连接下游项目、不修改其契约文件。

## 使用

先完成 README 中的双侧 Star Arm、reBot 绑定和软件零位标定。在运行遥操作的 Python 环境中安装采集额外依赖：

```bash
conda activate rebot_control
python -m pip install 'pyarrow>=17'
```

也可以用项目声明的 `recording` extra 安装依赖。相机子进程使用 `/usr/bin/python3`，需要系统提供 `python3-gi`、GStreamer introspection，以及 `v4l2src`、`decodebin`、`videoconvert`、`x264enc`、`mp4mux`、`qtdemux`、`avdec_h264` 插件；MJPEG 相机还需要对应解码插件。Ubuntu 的相关包通常为 `gir1.2-gstreamer-1.0`、`gstreamer1.0-plugins-base`、`gstreamer1.0-plugins-good`、`gstreamer1.0-plugins-ugly`、`gstreamer1.0-libav`。

```bash
python rebot_collect.py \
  --dataset datasets/cup \
  --instruction 'pick up the cup' \
  --camera top=/dev/video0 \
  --camera left_wrist=/dev/video2 \
  --camera right_wrist=/dev/video4
```

`rebot_collect.py` 是独立采集入口，一条命令同时运行遥操作和采集，无需另开遥操作进程。它复用已有控制和写盘模块，固定使用双臂和双夹爪。只有 `--dataset` 必填；不传 `--camera` 时自动发现相机，使用 `camera_0`、`camera_1` 等占位名；不传 `--instruction` 时写入 `__pending__`，录制后再批量补充。也可用 `--camera 0 --camera 2` 只选设备、暂不命名。输出结构保持不变，占位数据应在训练前补全标注；自动编号按本次设备枚举顺序分配。

需要边采集边看画面时，加一个开关：

```bash
python rebot_collect.py --dataset datasets/cup --camera-viewer
```

自动在浏览器打开 Camera viewer，终端也会打印本机地址（从 8080 起寻找空闲端口）。预览复用本次采集的相机帧，不会再次打开设备。相机线程只向一个有界槽位交出帧引用；像素复制和限时 socket 发送由后台线程完成，缩放、JPEG 编码和网页服务在独立低优先级进程中完成。预览最长边 640 像素、最多 10 FPS，满队列或慢客户端只丢预览帧，录制管线、原生采样率和时间戳保持原样。预览失败不会纳入采集就绪屏障或停止判断。

关闭网页或点击网页里的 **Close viewer** 不会结束采集；网页中的 Q/Esc 也只控制预览。停止采集仍使用终端的 Space、Esc、Q 或 Ctrl-C。采集结束后预览进程随之关闭。不加 `--camera-viewer` 时，不启动这些预览线程或进程。预览所需的 `appsrc`、`appsink`、`videoscale`、`jpegenc` 使用现有系统 GStreamer 插件。

`--camera` 至少指定一路，设备也可以写编号，如 `top=0`。同一物理相机在整个数据集中必须保持相同 ID；推荐使用稳定的 `/dev/v4l/by-id/...` 设备路径。ID 只允许小写 ASCII 字母、数字和下划线，ID 和设备路径均不可重复。采集模式要求双臂和双夹爪，不支持 `--side left/right` 或 `--no-gripper`。

默认生成 `episode_<UUID>`；可以用 `--episode-id cup_0001` 指定未使用过的名称。再次运行相同命令会追加一个 episode，不覆盖已有数据。支持原有 `--config`、`--leader-config`、`--settings` 和夹爪控制参数。

相机沿用 `webcam_viewer.py` 的自动采集模式选择，编码前等比例缩小到最长边不超过 960 像素，不裁剪视野，较小画面不放大；输出宽高对齐到偶数。例如 2592×1944 保存为 960×720，保持 4:3。相机收到首帧、state 写盘进程就绪后才进入电机使能阶段；完成双侧使能屏障后，开始记录并放行原有遥操作控制循环。启动准备阶段的相机帧不写入视频，帧率和逐帧时间戳保持原生采样。

按 Space、Esc、Q 或 Ctrl-C 结束，机械臂按现有逻辑保持当前位置；程序随后收齐 state、封装视频、解码检查帧数、统一时间原点并发布 episode。看到 `Saved episode ...` 才表示保存成功。停止后不要立即强制杀死进程，视频核验和 Parquet 收尾仍需要时间。一次轨迹只有一条任务指令，自动覆盖所有采样流的完整时间范围。

原有 `rebot_teleop.py --dataset ...` 入口保持兼容。仅遥操作继续使用 `rebot_teleop.py`，不传 `--dataset` 时也不要求安装 PyArrow。

## 输出

```text
datasets/cup/
├── episodes.jsonl
└── episodes/<episode_id>/
    ├── state/
    │   ├── left_eef.parquet
    │   ├── right_eef.parquet
    │   ├── left_gripper.parquet
    │   └── right_gripper.parquet
    └── rgb/
        ├── top.mp4
        ├── top.parquet
        ├── left_wrist.mp4
        ├── left_wrist.parquet
        ├── right_wrist.mp4
        └── right_wrist.parquet
```

每张 EEF 表只有 `timestamp_ns: int64` 和不可空的 `pose: fixed_size_list<float64, 12>`；每张夹爪表只有 `timestamp_ns: int64` 和 `openness: float32`。视频为无音轨 H.264 MP4，相机索引只有 `frame_index: int64`、`timestamp_ns: int64`，帧号从零开始，与按显示顺序解码的视频帧逐帧对应。无 action、关节状态、速度或其他传感器字段，无 episode 级 JSON。

H.264 使用 `x264enc pass=qual quantizer=23 bitrate=50000`，以 CRF 23 在画质和存储之间折中，码率上限为每路 50 Mbps，保留原有 `ultrafast` 和零延迟设置。50 Mbps 是上限，实际码率随画面变化；等比例缩小和 CRF 23 会比此前全分辨率 CRF 18 显著节省空间，具体占用应实测。旧实现未指定码率，默认 2 Mbps 对约 500 万像素的画面会造成明显块状压缩失真；旧视频已丢失的细节无法恢复。此设置不消除相机本身的高增益噪声。

## 源状态与采样时刻

state 取自从臂已经收到并通过校验的 MIT 实际反馈，不取主臂角度或控制目标。六轴模型关节角为 `feedback_position - reference_rad`，直接使用原有 MJCF 的关节方向，不应用 Star Arm 的方向映射，也不插值或裁剪六轴实际位置。

每个成功的七轴反馈批次产生一个 EEF 样本和一个夹爪样本。EEF 是该批次六个实测关节的正运动学结果，其时间戳取前六轴中最晚的主机接收时刻；夹爪取第七轴自身的主机接收时刻。两者分别写独立时间序列，左右臂独立运行。这个采样定义是源的原生完整反馈批次，实际频率取决于 CAN 回复和重试；标称 200 Hz 控制循环不是采样率保证。

CAN 反馈不提供可用的硬件采样时刻，因而当前 state 时间戳是 Python `time.monotonic_ns()` 的主机接收时间，不代表电机内部采样时刻或硬件同步。六轴反馈也不是硬件同时锁存。这一来源限制无法由导出格式消除。

相机保留 GStreamer buffer 的原生 PTS，经 TIME segment 转成 running time，再加 pipeline base time，桥接到同一主机单调时钟。桥接在启动时读取 GStreamer clock 与 `time.monotonic_ns()`，不使用首帧到达时刻或编码完成时刻作为相机时间原点。转换关系见 [GStreamer 时间同步说明](https://gstreamer.freedesktop.org/documentation/additional/design/synchronisation.html)。相机必须提供有效且非递减的时间戳；没有时间戳时报错，不按 FPS 补造。USB/驱动时间戳本身不等同于经过标定的曝光硬件同步。

不对齐行号、不补帧、不重采样。收尾时取四路 state 和全部相机首样本中的最小值，逐表用 int64 纳秒减去该共同原点，保留所有原生间隔；不会对各流分别归零。单条 instruction 的 `start_ns=0`，`end_ns` 为所有流归零后的最大时间戳，必须大于零。视频容器时序仅用于播放，实际时间以相机 Parquet 为准。

## 安装接口与 EEF 轴向

两臂固定在外部世界中，以各自模型的 `base_link` 为时不变参考系。采集期间不得移动基座；不存储左右臂之间的外参。

安装基准点沿用 `models/rebot_rs.xml` 中的 `gripper_mount`：相对 `link6` 平移 `[0,0,0.009] m`，对应模型安装盘前表面中心。它不随夹爪开合、指尖位置或工作端 TCP 改变。这是 CAD 模型声明，尚未通过实物重新标定。

用于数据的 EEF 轴向固定声明如下：

| 数据 EEF 轴 | 在 `gripper_mount` 中的方向 | 含义 |
|---|---|---|
| +X | +Z | 从安装面指向夹爪工作端的接近方向 |
| +Z | +X | 本模型声明的掌背方向 |
| +Y | −Y | 由 `+Z × +X` 确定 |

从数据 EEF 到原生安装接口的固定变换为：

```text
T_mount_from_eef =
[[ 0,  0,  1,  0],
 [ 0, -1,  0,  0],
 [ 1,  0,  0,  0],
 [ 0,  0,  0,  1]]

T_base_from_eef = T_base_from_mount @ T_mount_from_eef
```

导出 `pose = [x, y, z, R00, R10, R20, R01, R11, R21, R02, R12, R22]`，平移单位米，旋转按列展平，定义为 `p_base = R @ p_eef + t`。这个数据坐标变换只用于采集，不改变现有末端控制入口的 `gripper_mount` 轴向。实物正式采集前需核对零位、安装面和掌背方向与模型一致；更换安装方式时必须更新此源声明及固定变换。

## 实际夹爪 openness

闭合参考来自各侧 `rebot.local.yaml` 的第七轴 `reference_rad`，张开角跨度默认是物理完全张开的 270°：

```text
openness = clip((measured_motor_7_rad - reference_rad[6])
                / radians(record_gripper_open_deg), 0, 1)
```

有限越界值裁剪到 `[0,1]`，非有限反馈报错。实际位置受阻时保持对应实测开合程度，不会把闭合指令当作已经闭合。

`--record-gripper-open-deg` 指定两侧共同的物理全开跨度，与控制目标行程 `--gripper-open-deg` 分离。例如控制仅允许打开 180°、硬件全开仍为 270° 时，达到控制上限应记录约 `0.667`，不能重新标定成 `1`。如果实测硬件全开为其他角度，使用经核对的跨度；当前实现要求左右夹爪使用相同跨度，角度到开合程度采用线性比例，尚未测量指尖间距的非线性关系。

## 写盘、故障与验证

电机进程只把完整实际反馈放入最多 4096 条的队列，不等待 FK、Parquet 或视频编码。独立 state 进程按批写 Parquet；每个相机由独立系统 Python/GStreamer 进程采集和编码，录制分支不使用 leaky queue、videorate 或人为抽帧。可选预览独立抽帧且允许丢帧。设备/驱动在进入应用前丢失的帧无法恢复。

采集队列满、写盘错误、相机出错或无帧超时、采集子进程异常、硬件故障均使本次运行失败，保留 `.pending/<episode_id>/` 下的暂存文件，不向 `episodes.jsonl` 发布失败 episode。可选 viewer 的退出或故障不触发此流程。退出仍执行现有双臂位置保持流程。临时二进制时间戳和日志仅存在于暂存目录，成功发布前移除。

发布前要求四路 state 和所有相机均非空、流内时间戳非递减、轨迹时长为正，每路 MP4 均完整解码且帧数与索引一致。先完成 episode 文件，再更名到 `episodes/` 并以临时文件替换 `episodes.jsonl`；普通写入异常会回退目录。整个采集期间持有数据集文件锁，拒绝并发写同一数据集、重复 episode ID 和已有元数据/目录不一致。目录更名与元数据替换是两个文件系统操作，断电或 SIGKILL 落在两者之间仍可能留下孤立目录；再次采集会明确拒绝不一致的数据集，需要人工检查后恢复，不会静默覆盖。

离线检查命令：

```bash
python -m unittest discover -s tests -p 'test_rebot_recording.py' -v
python -m unittest discover -s tests -p 'test_rebot_teleop.py'
/usr/bin/python3 -m unittest discover -s tests -p 'test_webcam_viewer.py'
```

预览相关测试覆盖开关默认关闭、三路画面与录制同时工作、关闭或杀死 viewer 后继续保存、viewer 启动失败继续采集，以及 socket 客户端完全不读数据时原生帧间隔和完整视频仍正常。

采集测试使用真实 spawn 进程、虚拟 CAN/串口、GStreamer 测试画面及实际 H.264/Parquet 文件。它检查 FK 与 MuJoCo 一致、坐标轴及旋转列顺序、实际反馈与目标区分、非均匀/不同长度时间序列、统一原点、跨写入批次、指令范围、元数据追加、夹爪裁剪、队列溢出、数据集锁、写盘进程故障和电机故障不发布且保持，以及终端向所有子进程发送 Ctrl-C 后仍能收齐并发布完整 episode。测试不会连接实体机械臂或摄像头；实际设备的持续采集性能、时钟延迟和硬件几何还需实机核验。
