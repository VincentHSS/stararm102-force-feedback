# StarArm 102 Force Feedback

以 StarArm-102 为 Leader，为三种不同从臂提供负载阻尼、堵转检测、保持和恢复释放。三种方案的代码、启动方式与教程分开维护，按实际从臂选择。

## 先选你的方案

| 方案 | 主臂 → 从臂 | 负载来源 | 运行方式 | 代码入口 | 教程 |
| --- | --- | --- | --- | --- | --- |
| **一：自家 StarArm-102** | StarArm-102 → StarArm-102，可配置多个从臂 | 舵机电流 mA | 直接运行脚本，保留原有手动控制台 | [`stararm102_ro.py`](stararm102_ro.py) | [StarArm 教程](docs/tutorial.md) |
| **二：Seeed B601-DM** | StarArm-102 → Seeed B601-DM | DM 电机估计力矩 Nm | 独立 Python，直接 UART/CAN；无需 LeRobot | [`integrations/b601/`](integrations/b601/) | [B601-DM 教程](docs/b601_dm.md) |
| **三：Lumos 优化版** | StarArm-102 → Lumos 单臂及可选夹爪 | SDK 关节/夹爪力矩 Nm | 本仓库内独立的 Python 遥操作版本 | [`integrations/lumos/`](integrations/lumos/) | [Lumos 教程](docs/lumos.md) |

三种方案都提供自动负载阻尼和堵转保持，但驱动、映射和配置不能混用。**B601-DM 与 Lumos 不需要运行时手动输入 LD 扭矩。**

Lumos 优化版只在本力反馈仓库中维护。[原 Lumos 仓库](https://github.com/VincentHSS/stararm102-lumos-teleop)保留原有版本；需要自动反馈功能，请使用这里的 `integrations/lumos/`，不要把优化文件覆盖到原项目中。

## 方案一：自家 StarArm-102 主从臂

对应根目录 `stararm102_ro.py`。主臂角度经滤波后发送至自家从臂，用从臂电流调节主臂阻尼；结合角差、同向跟随比例和负载确认堵转。手动锁定与自动锁定统一管理，保持重发不会被阻尼更新覆盖。

需要 Python 3.10+。在仓库根目录安装依赖：

```bash
python -m pip install -r requirements.txt
python -m serial.tools.list_ports
```

编辑脚本顶部端口配置：

```python
SERVO_BAUDRATE = 1000000
LEADER_PORT_NAME = "/dev/ttyUSB0"
FOLLOWER_PORT_NAME_Arr = ["/dev/ttyUSB1"]
```

Windows 使用实际 COM 端口。启动时按 Leader 型号选择：

```bash
python stararm102_ro.py --leader_type 102LD
# 带按键板的 HD 型号
python stararm102_ro.py --leader_type 102HD
```

默认启用电流反馈和堵转检测。启动会卸力并重置圈数；`n` 和 `q` 也会使主臂卸力。

**以下手动命令仅适用于方案一，不适用于 B601-DM 或 Lumos：**

| 终端输入 | 功能 |
| --- | --- |
| `s` / `fb` / `st` | 总体状态 / 电流反馈 / 堵转状态 |
| `fb on` / `fb off` | 开关电流反馈；关闭时保留已有锁定 |
| `st lock` / `st unlock` | 锁定全部主臂关节 / 解除已有锁定 |
| `l` | 锁定 J1–J6，关闭电流反馈 |
| `500` | 全部主臂关节设为 500 mW 阻尼，关闭电流反馈 |
| `3=800 4=900` | 设置舵机 ID 3、4 的阻尼，对应 J4、J5 |
| `n` | 主臂卸力，同时关闭电流反馈和堵转检测；从臂继续跟随 |
| `h` / `q` | 帮助 / 退出 |

[完整教程](docs/tutorial.md) · [代码逻辑](docs/architecture.md) · [命令与参数](docs/reference.md) · [故障排查](docs/troubleshooting.md)

## 方案二：Seeed B601-DM，独立 Python 遥操作

对应 `integrations/b601/`，直接读取 Leader UART，使用达妙串口桥或 SocketCAN 控制 B601。无需 LeRobot、MotorBridge 或那个零位修复 wheel：

```bash
cd integrations/b601
python -m pip install -r requirements.txt
python teleop.py --simulate
cp b601.example.json my_b601.json
# 编辑端口/CAN 配置，摆好两臂参考姿态后采集
python teleop.py --config my_b601.json --capture paired.json
python teleop.py --config paired.json
# 核对方向、限位与配对姿态，在 paired.json 设置 calibration_verified=true
python teleop.py --config paired.json --execute
```

包含自适应滤波、输入死区、按时间限速、加速度和单帧步长限制，以及负载底载采样。连续零位的新回包有效；保留校验、丢包、突跳检查。默认映射来自官方配置，但配对姿态单独采集，不重置电机零位。

堵转必须满足大角差、超出底载的持续负载、实际无进展及连续观察窗口；正常追赶不再仅因跟随比例低就锁住。短暂丢包跳过运动帧，持续超时才保持并锁存故障。自动阻尼、堵转恢复与 HD 仲裁均无需手动输入扭矩。

核心代码：[`controller.py`](integrations/b601/controller.py)；通信与零位处理：[`bus.py`](integrations/b601/bus.py)，均含关键中文注释。配对、预览、执行、参数与 CSV 诊断见 [B601-DM 教程](docs/b601_dm.md)。旧 LeRobot 插件仅保留为 [历史存档与卸载参考](docs/b601_lerobot_legacy.md)，不要与新入口同时运行。

## 方案三：Lumos 优化版，独立保存在本仓库

对应 `integrations/lumos/`，不依赖 LeRobot。保留原 Lumos 遥操作的校准、映射、滤波、限速与诊断，增加自动阻尼、堵转保持、恢复释放，以及 HD 按钮与自动保持的统一仲裁。松开 HD 不会覆盖仍有效的自动保持。

从仓库根目录进入 Lumos 优化版后安装环境：

```bash
cd integrations/lumos
bash scripts/install_ubuntu.sh
```

安装脚本使用当前激活的 Python 环境，下载官方 Lumos SDK。完整的 Ubuntu、CAN、端口及校准准备见 [Lumos 教程](docs/lumos.md)。首次使用先校准，再预览：

```bash
python stararm102_lumos_teleop.py --calibrate --can can0 --leader-port /dev/ttyUSB0 --gripper
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper
```

确认映射后执行，自动反馈默认开启：

```bash
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper --execute
```

无 HD 按钮时增加 `--button-disable`；不使用夹爪时从所有命令中删除 `--gripper`。预览和校准不启用自动反馈，HD 按钮保持原有行为。`--feedback-disable` 可关闭整套自动反馈以排查接口兼容问题。

带中文注释的核心：[`lumos_auto_feedback.py`](integrations/lumos/lumos_auto_feedback.py)。它需要与 [`stararm102_lumos_teleop.py`](integrations/lumos/stararm102_lumos_teleop.py) 放在同一目录，由后者调用。

已有 Lumos 校准文件可以继续使用；如果 SDK 安装在原 Lumos 项目内，用 `--sdk-root /实际路径/startouch_sdk` 指向它，无需修改原项目文件。

## 反馈含义与验证范围

电流/估计力矩反映的负载包含重力、摩擦等影响，不等于经过标定的末端接触力。Leader SDK 的功率单位为 mW，不能直接当作从臂 Nm 力矩；协议中的功率 `0` 有特殊含义，不代表关闭。

Leader 保持不是从臂急停；实际保持能力取决于固件、供电及通信。三个方案均做了无硬件模拟验证，尚未完成实机锁力和整机保护验证。具体反馈新鲜度与通信边界分别写在各方案教程中。

## 开发验证

在仓库根目录运行 StarArm 与历史 B601 插件的 52 项测试：

```bash
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

Lumos 优化版的 34 项测试单独运行：

```bash
cd integrations/lumos
python -m unittest discover -s tests -v
```

B601 独立 Python 版的 51 项测试另行运行：

```bash
# 从仓库根目录进入
cd integrations/b601
python -m unittest discover -s tests -v
```

共 137 项模拟测试；CI 在 Ubuntu / Windows、Python 3.10 / 3.12 上执行。详情见 [测试说明](docs/testing.md) 和 [变更记录](CHANGELOG.md)。第三方组件遵循各自许可。
