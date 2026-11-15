除文档外所有代码保持英文。文档写给人看：先说怎么用，再说关键行为，不写实现流水账。

结构与约定见 `docs/architecture.md`。要点：

- 分层依赖只能向下：`apps → runtime / viewer / data / policy → model / devices`。
- 驱动电机一律通过 `rebot.runtime.arm.Arm`；只有 `rebot/apps/home.py` 可以发失能指令。
- 路径只从 `rebot.paths` 取；命令入口是 `rebot/apps/<name>.py` 的 `main(argv, prog)`，在 `rebot/cli.py` 注册。
- `rebot/media/` 在系统 Python 下运行，只能 import 标准库、`gi` 和 `rebot.paths`。
- 改控制代码后运行 `python -m unittest discover -s tests`。

硬件 CAD、打印文件、硬件预览图片、硬件说明页及其打包和渲染脚本统一保存在同级 `../rebot_hardware/`，不要放进本控制仓库。
本仓库 `models/` 中供控制、仿真和运行时可视化使用的模型保留在此。
