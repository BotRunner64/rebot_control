# 转换为 LeRobot v3

`rebot_to_lerobot.py` 将 `rebot_process.py` 导出的已切分、已标注数据转换成固定 **30 FPS** 的 LeRobot v3.0 数据集。源数据只读，不连接机械臂或相机，不上传 Hugging Face。

## 安装与运行

直接使用现有的 `rebot_control` conda 环境。LeRobot 已加入项目依赖；本机已安装 `lerobot==0.4.4`、CPU 版 `torch==2.7.1`、`torchvision==0.22.1` 和转换所需依赖，可以直接运行：

```bash
conda activate rebot_control
cd /home/wubingqian/projects/my_projects/rebot_projects/rebot_control

python rebot_to_lerobot.py \
  --dataset datasets/cup_processed \
  --output datasets/cup_lerobot_v3 \
  --repo-id local/cup
```

在其他机器的 `rebot_control` 环境补装转换依赖时执行：

```bash
conda activate rebot_control
cd /home/wubingqian/projects/my_projects/rebot_projects/rebot_control
uv pip install --python "$CONDA_PREFIX/bin/python" --torch-backend cpu -r requirements-lerobot.txt
uv pip install --python "$CONDA_PREFIX/bin/python" --no-deps lerobot==0.4.4
```

这里有意只安装 LeRobot 的数据集相关依赖，`--no-deps` 避免安装与离线转换无关的遥操、硬件输入和策略依赖。写入和读取接口固定使用 `lerobot==0.4.4`。没有 `uv` 时，也可在激活的 conda 环境中先用 pip 安装 CPU 版 `torch==2.7.1`、`torchvision==0.22.1`，再依次执行 `python -m pip install -r requirements-lerobot.txt` 与 `python -m pip install --no-deps lerobot==0.4.4`。

`--repo-id` 只是数据集身份，不会创建远程仓库或上传数据。输出必须是不存在的新目录，并且与源目录互不嵌套。`datasets` 软链接指向数据盘，以上输入、输出都会使用 `/mnt/data/rebot_data`。

运行时默认显示总 episode 进度、当前 episode 的逐帧进度、处理速度和预计剩余时间。同步、视频编码和保存等没有中间计数的阶段显示阶段名称与已耗时；最终视频校验也有逐帧进度条。只有编码和校验完成后才会显示保存成功。需要关闭进度条时可设置环境变量 `TQDM_DISABLE=1`。

可选参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--fps` | `30` | 全部输出流使用的整数频率 |
| `--max-delta-ms` | `50` | 目标时刻与最近原始样本之间允许的最大绝对时间差 |

如果出现 `Synchronization gap`，错误会给出 episode、数据流、目标时刻和偏差。先检查源流是否断采、相机实际频率是否过低，再决定是否调整阈值。此阈值限制每路相对目标时间的偏差，不保证任意两路实际采样时间差小于同一阈值。默认 50 ms 是导出上限，不是硬件同步精度承诺。

## 同步和动作定义

每个 episode 独立同步：

1. 读取四路状态和所有相机的原始 `timestamp_ns`，取首样本时间的最大值作为共同起点，末样本时间的最小值作为终点。
2. 从共同起点按 30 Hz 建立时间轴，使用整数纳秒运算，避免累计舍入误差。不同流不会各自归零。
3. 对每个时刻独立选择各流时间戳最近的样本。等距时选较早时刻，同一时刻有重复样本时选第一行。允许在局部重复或跳过样本，不插值、不平滑。最近邻可能选择略晚于目标时刻的样本，属于离线对齐。
4. 相机完全依据 Parquet 索引选择显示顺序帧号，不依据视频标称 FPS 或容器 PTS 推算原始采样时刻。原始视频顺序解码，每路只保留当前 RGB 帧。
5. 观察状态使用第 `k` 个同步时刻的实测值，`action[k]` 使用第 `k+1` 个同步时刻的实测值。最后一个时刻只提供动作目标，不作为观察行输出。因此 N 个同步时刻产生 N−1 行，不跨 episode，也不生成尾部伪造动作。

现有源数据没有记录控制命令，因此 `action` 是**未来实测状态构成的监督目标**，不是原来发送给机械臂的命令。

状态与动作都是 float32 的 26 维向量，顺序如下：

```text
[left_x, left_y, left_z,
 left_R00, left_R10, left_R20, left_R01, left_R11, left_R21, left_R02, left_R12, left_R22,
 left_gripper_openness,
 right_x, right_y, right_z,
 right_R00, right_R10, right_R20, right_R01, right_R11, right_R21, right_R02, right_R12, right_R22,
 right_gripper_openness]
```

保持位置单位米、旋转矩阵按列展平、各臂自己的基座坐标系，以及夹爪 `0=闭合，1=张开`。源位姿 float64 转为训练常用的 float32，除精度转换外不修改数值。模型原生 EEF、采集和末端控制统一使用[采集源契约](recording.md)中的坐标定义，无额外换轴。

相机名、分辨率和视野保持原样；视频根据选中的帧重新编码为 H.264，统一 30 FPS，使用固定版本 LeRobot 的编码设置，会引入有损压缩。同步图像直接送入各路并行编码线程，不生成中间 PNG 文件。每路队列最多缓存 4 帧，编码跟不上时生产端等待，不丢帧。该适配使用固定版本 `lerobot==0.4.4` 的内部编码接口；升级 LeRobot 时需要重新核对接口并运行测试。输出仍由官方写入器保存并经过完整校验。

## 输出与核验

标准 v3 内容由官方 `LeRobotDataset.create/add_frame/save_episode/finalize` 写出，包含 `meta/info.json`、`meta/stats.json`、`meta/tasks.parquet`、`meta/episodes/`、`data/` 和 `videos/`。多个 episode 可以共用同一个 Parquet 或 MP4 分片，边界由官方元数据保存。

输出根目录仅包含标准的 `data/`、`meta/` 和 `videos/`，不额外生成转换记录或同步索引文件。

发布前会调用官方读取器，以 `delta_timestamps` 方式重读每个 episode 的首尾样本，检查动作窗口、形状和 episode 边界；再完整解码全部输出视频，检查帧数及 30 FPS 时间间隔。源视频也会完整解码以核对索引数量，尾部损坏或额外帧不会静默忽略。

所有内容先写到输出目录旁的隐藏临时目录，验证通过后整体更名。使用输出文件锁防止同一目标并发转换；普通异常会清理临时目录，不发布半成品，源数据不变。强制杀死进程可能留下隐藏临时目录。

## 读取数据

转换结果的本地读取示例：

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(
    repo_id="local/cup",
    root="/mnt/data/rebot_data/cup_lerobot_v3",
    video_backend="pyav",
    delta_timestamps={"action": [i / 30 for i in range(50)]},
)
sample = dataset[0]
print(sample["observation.state"].shape)  # (26,)
print(sample["action"].shape)             # (50, 26)
print(sample["task"])
```

## 测试

```bash
conda activate rebot_control
python -m unittest discover -s tests -p 'test_rebot_to_lerobot.py' -v
```

测试使用真实 H.264 视频、异步状态和相机时间戳，经命令行导出后用官方读取器逐帧核对状态、下一步动作、图像内容、instruction 和跨 episode 的动作块 padding，同时覆盖缺采、视频索引不匹配、失败清理、源文件哈希不变、输出保护和并发锁。编码测试还使用延迟启动的真实 H.264 消费线程和仅一帧的队列，检查背压时不丢帧、不写 PNG，以及编码线程失败时不会挂起或发布半成品。
