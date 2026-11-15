# 数据流程

三步，各自独立：

```text
rebot collect   连续录制一整段遥操作        →  datasets/cup            （原始，只追加）
rebot process   回放、切分 episode、写任务描述 →  datasets/cup_processed  （新目录）
rebot export    30 FPS 对齐，转 LeRobot v3    →  datasets/cup_lerobot    （新目录）
```

采集时不用管 episode 边界和任务描述，录完再标。本机的 `datasets/` 是指向数据盘 `/mnt/data/rebot_data` 的软链接。

文件结构、坐标系和时间戳的精确定义见 [数据格式](data-format.md)。

## 系统依赖

相机采集和视频处理在系统 Python（`/usr/bin/python3`）下跑 GStreamer，Conda 环境里不需要装。Ubuntu：

```bash
sudo apt install python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

先用 `rebot cameras` 确认相机能出画面。

## 1. 采集 `rebot collect`

一条命令同时跑遥操作和录制。固定双臂 + 双夹爪，至少一路相机。

```bash
rebot collect --dataset datasets/cup                    # 自动发现相机，之后再标注
rebot collect --dataset datasets/cup --camera-viewer    # 同时在浏览器看画面
rebot collect --dataset datasets/cup \
  --instruction 'pick up the cup' \
  --camera top=/dev/video0 --camera left_wrist=/dev/video2 --camera right_wrist=/dev/video4
```

- 按 `空格` / `Esc` / `Q` / `Ctrl-C` 结束。之后程序还要封装和校验视频，**看到 `Saved episode ...` 才算保存成功**，别急着杀进程。
- 再次运行会往同一数据集追加一段，不覆盖。
- 不传 `--camera` 时相机命名为 `camera_0`、`camera_1`…（按本次枚举顺序）；不传 `--instruction` 时写入 `__pending__`。两者都可以在 `rebot process` 里补。
- 想让相机名稳定对应物理设备，用 `--camera top=/dev/v4l/by-id/...`。同一数据集里同名相机应是同一个视角。
- `--camera-viewer` 的预览是尽力而为的（640 px、10 FPS，会丢帧），关掉它不影响录制。
- `--record-gripper-open-deg`：夹爪物理全开对应的电机角度（默认 270），用于把记录的开合度归一化到 0–1。它和控制行程 `--gripper-open-deg` 是两回事。
- 其余遥操作参数与 `rebot teleop` 相同。

**录了什么**：从臂**实测**的末端位姿和夹爪开合度（不是主手角度，也不是控制目标），加每路相机的 H.264 视频和逐帧时间戳。视频等比例缩到最长边 960 px。

**失败时**：相机出错、写盘出错、电机故障等都会让这次录制失败，数据留在 `<dataset>/.pending/` 供排查，不会混进正式数据。机械臂照常停止并保持。

## 2. 切分与标注 `rebot process`

本地网页，不连接机械臂和相机。原始数据只读。

```bash
rebot process --dataset datasets/cup --output datasets/cup_processed
```

打开终端打印的 `http://127.0.0.1:8090`（`--port` 可改）：

1. 顶部填 **Instruction**；需要时直接点画面角落改相机名。
2. 回放检查时间轴。蓝色是保留的片段，阴影是排除的部分（回位、调整等）。
   - 选中一段，拖它的边缘只修剪这一段：往里拖，空出来的部分变成排除区；往外拖，到相邻片段为止。
   - 两段紧挨着时，按住 `Shift` 拖边缘是移动两段之间的切点（两段一起变）。
   - 选中一段点 **Exclude** 整段排除；选中阴影点 **Include** 把它变成一段。
3. 点 **Export**。

快捷键：`空格` 播放，`←/→` 逐帧，双击或 `S` 切分，`Ctrl/⌘+Z` 撤销，`Ctrl/⌘+滚轮` 缩放时间轴。编辑自动保存。

**自动初切**：第一次打开时自动切一次。规则是找“双臂都回到起始姿态附近（10 cm、20° 以内）并静止 0.8 秒以上”的时刻，把它当作一次复位，复位之间的运动算一个 episode。这只是初稿——任务本身在起始位置附近停顿也会被误切——请对着视频确认。之后刷新不会重新自动切。

**草稿**：保存在输出目录旁的 `cup_processed.plan.json`（`--plan` 可指定）。有草稿后可以不开网页直接导出：

```bash
rebot process --dataset datasets/cup --output datasets/cup_processed_v2 \
  --plan datasets/cup_processed.plan.json --export
```

规则：

- 输出目录必须不存在。改了标注就换一个新的输出路径。
- 导出前所有片段都要有非空、非 `__pending__` 的 instruction。
- 片段是半开区间 `[start, end)`；各路数据按自己的时间戳筛选，不补帧、不插值。视频按帧精确裁切并重新编码。
- 回放用的低分辨率预览缓存在草稿旁的 `.rebot-preview/`，可以随时删。

## 3. 导出 LeRobot v3

第一次使用先装转换依赖（只装数据集相关的部分）：

```bash
uv pip install --torch-backend cpu -e '.[lerobot]'
uv pip install --no-deps lerobot==0.4.4
```

```bash
rebot export --dataset datasets/cup_processed --output datasets/cup_lerobot --repo-id local/cup
```

- `--repo-id` 只是数据集名字，不会上传。输出目录必须不存在。
- `--fps`（默认 30）；`--max-delta-ms`（默认 50）是对齐时允许的最大时间差，超过会报 `Synchronization gap` 并指出是哪路数据、哪个时刻——通常说明那路数据断采了。

**对齐方式**：每个 episode 取所有数据流都有数据的时间段，按 30 Hz 建时间轴，每个时刻每路取**时间最近的样本**（不插值）。

**state 和 action**（都是 26 维 float32）：

```text
[左臂 x y z, 左臂旋转矩阵按列展平 9 个, 左夹爪开合度,
 右臂 x y z, 右臂旋转矩阵按列展平 9 个, 右夹爪开合度]
```

`observation.state[k]` 是第 k 个时刻的实测状态，`action[k]` 是**第 k+1 个时刻的实测状态**。也就是说 action 是“下一步实际到达的状态”，不是当时发给机械臂的指令（采集时没有记录指令）。

读取示例：

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(
    repo_id="local/cup", root="/mnt/data/rebot_data/cup_lerobot", video_backend="pyav",
    delta_timestamps={"action": [i / 30 for i in range(50)]},
)
sample = dataset[0]
sample["observation.state"].shape   # (26,)
sample["action"].shape              # (50, 26)
```

导出会用官方读取器重读校验、完整解码所有视频，通过后才发布；失败不留半成品。转换代码依赖 `lerobot==0.4.4` 的内部编码接口，升级 LeRobot 前先跑 `tests/test_lerobot.py`。
