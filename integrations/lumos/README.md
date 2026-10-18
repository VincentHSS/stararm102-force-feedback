# Lumos 自动反馈优化版

本目录是 `stararm102-force-feedback` 的第三种方案：StarArm-102 Leader → Lumos 单臂。与本仓库的自家 StarArm 从臂、B601-DM 独立 Python 版分开运行。

优化代码只保存在本目录，原 `VincentHSS/stararm102-lumos-teleop` 仓库保留原有版本。

在本目录安装与运行：

```bash
bash scripts/install_ubuntu.sh
python stararm102_lumos_teleop.py --calibrate --can can0 --leader-port /dev/ttyUSB0 --gripper
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper --execute
```

执行模式默认加入自动负载阻尼、堵转保持、稳定恢复及 HD 按钮仲裁。无按钮加 `--button-disable`，无夹爪从全部命令中删除 `--gripper`。自动反馈不要求运行时输入扭矩。

- [`lumos_auto_feedback.py`](lumos_auto_feedback.py)：自动反馈核心，含中文注释。
- [`stararm102_lumos_teleop.py`](stararm102_lumos_teleop.py)：校准、预览与遥操作入口。
- [完整安装、校准、运行和参数教程](../../docs/lumos.md)。
- [三种方案对照](../../README.md)。

已有校准文件可沿用；通过 `--sdk-root` 可复用其他目录中的官方 SDK，不需要改动原 Lumos 项目。

```bash
python -m unittest discover -s tests -v
```

34 项模拟测试；尚未实机验证。
