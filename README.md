# StarArm 102 Force Feedback

StarArm 102 主从遥操作控制器，支持 StarArm 从臂电流反馈，以及 Seeed B601-DM 从臂力矩反馈，提供主臂阻尼、堵转检测与位置保持。

StarArm 自家从臂保留原单文件控制程序；Seeed 601 使用官方 LeRobot 遥操作的自动反馈增强。

## 功能

- 主臂角度经均值滤波后发送到从臂，支持配置多个从臂。
- 按从臂关节电流映射主臂阻尼，默认 30 Hz 采样、按功率死区更新。
- 根据角度误差、同向跟随比例和负载判断堵转，默认锁定全部主臂关节。
- 手动锁定与自动锁定统一管理，阻尼更新跳过已锁定关节。
- 锁力重发独立于从臂采样；自动解锁要求误差持续回落。
- 终端输入排队，主线程统一处理控制状态与串口操作。

电流反馈是负载的近似量，不是经过标定的末端力或力矩测量。

## Seeed B601-DM：增强官方 LeRobot 遥操作

在已经能运行官方遥操作的同一环境中安装：

```bash
python -m pip install ./integrations/lerobot_auto_feedback
python -m stararm_auto_feedback.install
```

然后继续使用原来的 `lerobot-teleoperate` 命令。自动加入 601 负载反馈阻尼与堵转时的 Leader 保持；不增加手动扭矩输入、不另开串口、不替换官方映射、驱动或遥操作循环。

[601 完整安装与逻辑说明](docs/b601_dm.md)。原独立 601 控制入口已移除。官方标定与设备配置继续沿用；实际阻尼和锁力尚未上机验证。

## StarArm 从臂快速开始

以下操作和手动控制命令仅对应 StarArm 自家从臂的 `stararm102_ro.py`。Seeed 601 的官方 LeRobot 集成没有这套手动控制台。

需要 Python 3.10+、主从机械臂、串口连接与设备供电。验证情况见 [测试说明](docs/testing.md)。

```bash
python -m venv .venv
# Linux
source .venv/bin/activate
# Windows PowerShell 改用：.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python -m serial.tools.list_ports
```

编辑 `stararm102_ro.py` 顶部的串口配置：

```python
SERVO_BAUDRATE = 1000000
LEADER_PORT_NAME = "/dev/ttyUSB0"
FOLLOWER_PORT_NAME_Arr = ["/dev/ttyUSB1"]
```

Windows 对应填写 `COM3`、`COM4` 等实际端口，然后启动：

```bash
python stararm102_ro.py --leader_type 102LD
# 带按键板的 HD 型号
python stararm102_ro.py --leader_type 102HD
```

默认启用电流反馈和堵转检测。首次上机应在支撑好机械臂、工作区清空、可及时断电的条件下低速验证；启动会卸力并重置圈数，程序内的 `n` 和 `q` 也会使主臂卸力。

## 常用操作

| 终端输入 | 功能 |
|---|---|
| `s` / `fb` / `st` | 总体状态 / 电流反馈 / 堵转状态 |
| `fb on` / `fb off` | 开关电流反馈；关闭时保留已有锁定 |
| `st lock` / `st unlock` | 锁定全部主臂关节 / 解除已有锁定 |
| `l` | 锁定 J1–J6，关闭电流反馈 |
| `500` | 全部主臂关节设为 500 mW 阻尼，关闭电流反馈 |
| `3=800 4=900` | 设置舵机 ID 3、4 的阻尼，对应 J4、J5 |
| `n` | 主臂卸力，同时关闭电流反馈和堵转检测 |
| `h` / `q` | 帮助 / 退出 |

**协议中的功率 `0` 表示按功率保护阈值执行，不代表关闭。** 反馈映射下限至少为 1 mW；`n` 会卸力且不会停止从臂跟随。

## 文档

| 文档 | 内容 |
|---|---|
| [601 版本教程与逻辑](docs/b601_dm.md) | 官方 LeRobot 接入、自动反馈逻辑、参数与卸载 |
| [安装与上机教程](docs/tutorial.md) | 接线、环境、端口、启动、基础验证和标定 |
| [代码逻辑](docs/architecture.md) | 主循环、数据流、负载映射、堵转状态和锁力维护 |
| [命令与参数](docs/reference.md) | 终端命令、启动参数和默认值 |
| [故障排查](docs/troubleshooting.md) | 未锁定、锁力不足、误触发、串口和反馈问题 |
| [测试说明](docs/testing.md) | 无硬件回归、CI 和实机验证范围 |
| [变更记录](CHANGELOG.md) | 锁定可靠性及控制台整理 |

## 项目结构

```text
stararm102_ro.py          StarArm 从臂控制与 Leader 核心
integrations/lerobot_auto_feedback/
                         官方 LeRobot 自动反馈增强与参数示例
requirements.txt         串口与舵机 SDK 依赖
requirements-dev.txt     开发工具
README.md
CHANGELOG.md
docs/                    中文教程与逻辑说明
tests/test_controller.py 无硬件回归测试
.github/workflows/ci.yml  自动检查
```

## 开发验证

```bash
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python -m py_compile stararm102_ro.py
python stararm102_ro.py --help
```

测试使用模拟串口与控制器，不连接机械臂。本项目调用第三方 `fashionstar-uart-sdk`，不随仓库分发其源码；第三方组件遵循各自许可。当前版本没有完成实机锁力和整机安全验证，控制频率取决于串口、设备数量和监控查询耗时。
