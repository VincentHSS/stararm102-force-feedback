# LeRobot 官方遥操作自动反馈增强

**旧版存档：B601-DM 当前推荐 [独立 Python 遥操作](../b601/)，不再经过 LeRobot。** 此目录保留用于维护和卸载已有钩子，旧版说明见 [历史教程](../../docs/b601_lerobot_legacy.md)。新旧入口不要同时占用串口/CAN。

在能正常运行官方遥操作的同一 Python 环境里执行：

```bash
python -m pip install ./integrations/lerobot_auto_feedback
python -m stararm_auto_feedback.install
```

然后继续运行原来的 `lerobot-teleoperate` 命令。插件没有手动扭矩输入、独立遥操作循环或串口驱动。

[旧版安装、逻辑、自动参数和卸载说明](../../docs/b601_lerobot_legacy.md)。尚未实机联调。
