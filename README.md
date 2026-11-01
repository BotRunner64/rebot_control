## rebot arm rs版的设置，控制，遥操作

Star Arm 扫描和左右身份绑定，只使用官方舵机 SDK。

```bash
conda activate rebot_control
python stararm.py scan
python stararm.py bind left /dev/ttyUSB0
python stararm.py bind right /dev/ttyUSB1
```

绑定前核对实体左右臂；配置保存到 `stararm.local.yaml`，优先使用 USB 连接路径。
换插口或互换线缆后需重新核对；修改已有绑定加 `--replace`。
只查询在线 ID 和保存本地配置，不修改舵机参数或发送运动指令。
