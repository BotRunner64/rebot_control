# 数据格式

`rebot collect` 和 `rebot policy --record` 保存可回放的连续录制。`rebot process` 把选中的片段写成 LOOM-data 的标准 LeRobot v3.0 数据集。操作步骤见[数据流程](data.md)，完整字段定义见 [LOOM-data 导出契约](https://github.com/BotRunner64/LOOM-data/blob/36dd849dd1eb9ce87dfb59ad656c00147270ff66/docs/contract/export-contract.md)。

## 连续录制

每次成功的采集生成一个 `recordings/<id>/` 目录，包含以下文件。

| 文件 | 内容 |
| --- | --- |
| `recording.json` | 同步帧数、采集时使用的相机名、任务描述 |
| `frames.parquet` | 每帧一行，含 `observation.state` 和 `task` |
| `<camera>.mp4` | 对应相机的同步视频 |
| `debug/` | 仅策略录制时保存的原频率诊断数据 |

## 时间与图像

从臂每收齐一批反馈生成一个 EEF 样本和一个夹爪样本。EEF 取前六轴最晚的接收时刻，夹爪取第七轴的接收时刻。遥操作伺服标称 200 Hz，策略控制标称 250 Hz。相机按设备实际帧率采集，GStreamer PTS 和反馈接收时刻换算到同一主机单调时钟。硬件之间没有触发同步。

录制结束时取所有流的共同时间区间，从最晚的首样本起以 30 Hz 取帧。位置与开度线性插值，旋转用 SLERP，每路 RGB 取最近的原始帧。跨越超过 100 ms 的 state 间隙、最近 RGB 距帧时刻超过 50 ms，或共同区间不足两帧都会使本次同步失败。失败的采样留在 `.pending/<id>/`。成功录制通过目录改名一次发布。

第 k 帧对应 k/30 秒，每路视频为 H.264、yuv420p、恒定 30 fps、无音轨。按解码图像等比缩放到宽 640，高度按 4:3、16:9、16:10、16:13 分别取 480、360、400、520。其他宽高比会报错。

## LeRobot v3.0

每个布局使用 `top`、`left_wrist`、`right_wrist` 的非空子集，每集每路一个视频。帧表位于 `data/`，其中 `timestamp = frame_index / 30`。每个 episode 的末帧也保留。指令由 `task_index` 关联 `meta/tasks.parquet`，片段 ID 存入 `meta/episodes/` 的 `episode_id` 列。

`observation.state` 是 26 维 float32，依次为左 EEF（12）、左夹爪（1）、右 EEF（12）、右夹爪（1）。数据集只提供实测状态，动作由下游构造。

## 末端位姿 `pose`

```text
pose = [x, y, z, R00, R10, R20, R01, R11, R21, R02, R12, R22]
```

- 平移单位米；旋转矩阵**按列展平**；含义是 `p_base = R @ p_eef + t`。
- 参考系是**各臂自己的基座** `base_link`。两臂各算各的，数据里不存两臂之间的外参。采集期间不要移动基座。

### 末端坐标系

原点在夹爪安装面中心（模型里的 `gripper_mount`，相对 `link6` 平移 `[0, 0, 0.009] m`）。它不随夹爪开合或更换指尖而变。

| EEF 轴 | 在 `link6` 中的方向 | 含义 |
| --- | --- | --- |
| +X | +Z | 接近方向：从安装面指向夹爪工作端 |
| +Z | +X | 掌背方向 |
| +Y | −Y | 由 +Z × +X 确定 |

```text
T_link6_from_eef = [[0,  0, 1, 0    ],
                    [0, -1, 0, 0    ],
                    [1,  0, 0, 0.009],
                    [0,  0, 0, 1    ]]
```

采集、`rebot ee`、`rebot policy`、3D 操作柄都用这同一个定义。它来自 CAD 模型，未经实物标定。

## 夹爪 `openness`

```text
openness = clip((电机 7 实测角 − 闭合零位) / radians(record_gripper_open_deg), 0, 1)
```

0 = 闭合，1 = 张开。取实测位置：夹住物体合不拢时，记录的就是合不拢的开度。线性比例，没有换算成指尖间距。
