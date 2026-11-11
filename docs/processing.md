# 连续采集与离线处理

采集只负责连续保存传感器数据。episode 边界、任务 instruction 和相机语义名称在离线阶段确定。`rebot_process.py` 同时提供本地网页和命令行导出，处理过程不连接机械臂或摄像头。

## 1. 连续录制

```bash
conda activate rebot_control
python rebot_collect.py --dataset datasets/raw_cup
```

整段操作持续录制，操作中不切 episode、不标任务。不传相机参数时自动发现并使用 `camera_0` 等采集编号，instruction 默认 `__pending__`。结束后等待 `Saved episode ...`，再开始处理。为了兼容现有采集实现，每次连续 session 仍保存为原始数据集中的一个 `episodes/<id>`；这只是存储单位，最终训练 episode 由后处理决定。也可以直接处理此前已经录好的数据。

需要稳定对应设备时使用 `--camera camera_0=/dev/v4l/by-id/...`。自动编号仅表示本次枚举顺序，重新插拔设备可能改变顺序；不同 session 的同名采集编号应对应同一个物理视角。如果视角改变，请采集到另一个原始数据集，分别处理。

## 2. 打开本地处理界面

在原有采集环境中运行，需要已有的 PyArrow、NumPy、OpenCV，以及系统 Python 的 GStreamer H.264 依赖。不需要 Node.js 或前端构建。

```bash
python rebot_process.py \
  --dataset datasets/raw_cup \
  --output datasets/cup_processed
```

打开终端打印的 `http://127.0.0.1:8090`。可用 `--port 8091` 修改端口。仅监听本机回环地址，所有资源和数据都由本地服务提供。

打开录制后自动拆分一次，随后只需：

1. 在顶部填写统一的 **Instruction**；需要时直接编辑画面角落的相机名称。
2. 回放检查时间轴，拖动片段或阴影边缘调整 reset 排除范围。
3. 点击 **Export**。

蓝色片段保留、阴影排除。选中范围后，唯一的上下文按钮用于 **Exclude / Include**。编辑自动保存；刷新恢复手动结果，不会再次自动拆分。已生成过 reset 建议的旧草稿也不会重新覆盖。

没有 Auto split、参考姿态设置、速度选择、回放导航、手动保存或 Fine adjustments 面板。单个录制时也不显示录制选择器。需要时使用 `Space` 播放、左右方向键逐帧、双击或 `S` 切分、`Ctrl/⌘ Z` 撤销；长时间轴支持 `Ctrl/⌘ + 滚轮` 缩放。快捷操作说明仅在对应控件悬停时出现。

自动拆分完成标记随草稿保存，撤销初稿或手动修改后重新打开，都不会重新执行拆分。检测结果无法建立有效 reset 时保留整段供手动调整。网络等临时故障不会记为完成，可以刷新重试。

## Reset 初步拆分规则

分析使用双臂各自的实际 EEF 反馈，不用视频画面推断。仅用于检测的 20 Hz 时间网格采用最近已有反馈，反馈超过 250 ms 未更新视为无效。导出仍使用全部原始样本，不改变采样率或时间戳。

默认寻找开始 30 秒内首个持续至少 0.8 秒的稳定姿态作为参考。之后要求双臂都距各自参考位置不超过 3 cm、姿态差不超过 20°，并且线速度不高于 1.5 cm/s、角速度不高于 8°/s，持续至少 0.8 秒，才作为一次回位静止候选。不会仅因任务中途在别的位置暂停就拆分；只有一侧回位也不会认作双臂 reset。

速度用 0.25 秒窗口估算。默认从检测到的停顿向前排除约 1 秒，作为手动回位动作的初始估计；停顿结束前留约 0.15 秒余量，以保留缓慢离开初始位置的任务动作。剩余区间至少 0.75 秒、包含实际运动且所有必需流均有样本，才建议为 episode。只有静止或反馈缺失的区间不会伪造任务 episode。

**静止不等于精确的 reset 动作起点。** 该启发式不能保证识别完整的手动回位，任务本身若长时间停在初始位置附近也可能产生误判。阴影边界是初稿，需要通过视频确认并拖动修正。算法不控制机械臂，原始数据始终保留。自动检测的原始建议保存到草稿及导出来源记录中，实际导出边界以手动调整后的 `episodes` 为准。

预览是按源 Parquet 时间戳定位的 JPEG 画面，最多约 10 次/秒刷新，慢机器可能更低；这只影响回放流畅度。逐帧定位及导出使用完整的原始帧索引，不按预览刷新率抽帧。相机尚未产生首帧时会显示提示。

## 3. 草稿与批处理

默认草稿保存在输出目录旁的 `cup_processed.plan.json`。可用 `--plan /path/to/plan.json` 指定位置，草稿必须位于原始数据集以外。JSON 中 `dataset` 是原始数据集绝对路径，`session` 是原有 `episodes.jsonl` 中的 ID，时间为相对连续录制原点的整数纳秒。

```json
{
  "version": 1,
  "dataset": "/absolute/path/to/datasets/raw_cup",
  "recording_instructions": {"episode_replace_with_source_id": "Pick up the cup and place it on the tray"},
  "camera_names": {"camera_0": "top"},
  "episodes": [
    {
      "id": "cup_0001",
      "session": "episode_replace_with_source_id",
      "start_ns": 2000000000,
      "end_ns": 8000000000
    }
  ]
}
```

已有草稿可以直接批处理，无需启动网页：

```bash
python rebot_process.py \
  --dataset datasets/raw_cup \
  --output datasets/cup_processed_v2 \
  --plan datasets/cup_processed.plan.json \
  --export
```

每次导出目标必须是不存在的新目录；修改标注后使用新的输出路径，避免覆盖已有成果。空 instruction 和 `__pending__` 可以暂存在草稿，但导出前必须补齐。

## 时间、视频和发布语义

- 选段使用半开区间 `[start_ns, end_ns)`；四路 state 和每路相机分别按自己的时间戳筛选，不补帧、不插值、不重采样。每段必须包含全部必需流的至少一个样本，且总采样时长为正。
- 所有输出表统一减去所选片段的 `start_ns`，保留流之间的偏移和各自原始间隔。首个样本可能大于零，不会将每路分别归零。
- 相机按筛选结果对应的原始显示帧编号解码裁切，重新编码 H.264；帧号从零开始，Parquet 保留选中帧的实际时间。重新编码会引入有损压缩，视频容器按选中帧的时间间隔播放，样本时间仍以 Parquet 为准。
- 每路输出视频重新完整解码核对帧数。为确保帧精确性，当前实现从源视频开头解码到片段终点，长 session 后段的多次导出可能较慢。
- 输出沿用既有 `episodes.jsonl`、`state/*.parquet`、`rgb/*.mp4` 和 `rgb/*.parquet` 契约；instruction 从零覆盖到片段内所有流的最后一个样本时间。数据集根目录额外保存 `processing.json`，记录源路径、选段和名称映射。
- 全部片段先写入输出目录旁的临时目录，校验成功后整体更名发布。异常清理未发布内容，原始数据保持不变；强制杀死进程可能留下隐藏临时目录。使用输出侧文件锁，拒绝多个处理进程同时导出到同一目标。

## 界面原则与验证

界面参考 [Apple Human Interface Guidelines — Design principles](https://developer.apple.com/design/human-interface-guidelines/design-principles)：围绕回放、切分与任务描述组织主界面；保持熟悉的视频控制和清晰的状态反馈；提供可恢复编辑、独立草稿、显式导出；支持键盘、可访问标签及窄屏布局。界面代码和文案为英文，符合本仓库约定。

```bash
python -m unittest discover -s tests -p 'test_rebot_process.py' -v
python -m unittest discover -s tests -p 'test_rebot_reset_detection.py' -v
```

测试使用真实 H.264 与 Parquet，检查统一录制 instruction、多个片段、异步采样、选段边界、相机重命名、帧内容与帧数、状态值及 schema 保留、原始文件哈希不变、错误输入、草稿恢复、已有输出保护与导出失败不发布。


## 产品取舍

自动步骤直接执行一次，界面只留下统一任务描述、视频、时间轴和导出。相机名称直接在画面上修改。操作细节放到悬停提示与快捷键中，常规状态用短文字反馈，不再占用主界面解释算法或展示设置。
