# 数据格式

`rebot collect` 和 `rebot process` 的输出格式相同，遵循 LOOM 导出契约。这里是数据的精确定义；怎么用见 [数据流程](data.md)。

## 目录结构

```text
datasets/cup/
├── episodes.jsonl                      每行一个 episode：id 与任务描述
└── episodes/<episode_id>/
    ├── state/
    │   ├── left_eef.parquet            timestamp_ns: int64, pose: 12 × float64
    │   ├── right_eef.parquet
    │   ├── left_gripper.parquet        timestamp_ns: int64, openness: float32
    │   └── right_gripper.parquet
    └── rgb/
        ├── <camera>.mp4                H.264，无音轨
        └── <camera>.parquet            frame_index: int64, timestamp_ns: int64
```

- 没有 action、关节角、速度等其他字段。
- `frame_index` 从 0 开始，对应视频按显示顺序解码的第几帧。**帧的时间以 Parquet 为准**，不要用视频容器的时间或标称帧率。
- `rebot process` 的输出在根目录多一个 `processing.json`，记录来源和切分信息。

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

## 数据来源与时间戳

**state 来自从臂的实测反馈**，不是主手角度或控制目标。每收齐一批 7 个电机的反馈产生一个 EEF 样本（6 轴正运动学）和一个夹爪样本。实际频率取决于 CAN，标称 200 Hz 不是保证。

**时间戳都是同一台主机的单调时钟**（纳秒）：

- state：主机**收到**反馈的时刻（EEF 取前 6 轴里最晚的一个）。CAN 反馈没有硬件时间戳，所以这不是电机内部的采样时刻。
- 相机：GStreamer buffer 的 PTS 换算到主机单调时钟，不是首帧到达或编码完成的时刻。相机不提供时间戳会报错，不会按帧率编造。

**各路数据保持各自的原始采样**：不对齐、不补帧、不重采样，行数也各不相同。发布时所有流减去同一个时间原点（所有流首样本中最早的那个），所以流与流之间的时间差保留。对齐到统一时间轴是 `rebot export` 的工作。

左右臂、各相机之间都没有硬件同步。

## 视频

采集时等比例缩小到最长边 ≤ 960 px（不裁剪、不放大），宽高取偶数。x264 CRF 23，码率上限 50 Mbps，保持相机原生帧率。启动准备阶段的帧不写入。

## 发布保证

- 一段录制只有全部校验通过才会出现在 `episodes.jsonl`：所有流非空、时间戳不递减、每路视频能完整解码且帧数与索引一致。
- 失败的录制留在 `.pending/<episode_id>/`。
- 录制期间对数据集加文件锁，拒绝并发写入和重复的 episode id。
- 断电或 `kill -9` 恰好发生在“目录改名”和“更新 `episodes.jsonl`”之间时，会留下一个未登记的目录；下次采集会明确报错并拒绝写入，需要人工检查，不会静默覆盖。
