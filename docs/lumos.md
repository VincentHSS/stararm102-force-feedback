# 方案三：Lumos 优化版自动负载反馈

本方案的代码位于本力反馈仓库的 [`integrations/lumos/`](../integrations/lumos/)，与自家 StarArm 从臂和 B601-DM 方案分开运行。优化后的代码不放入原 `VincentHSS/stararm102-lumos-teleop` 仓库。返回 [三种方案对照](../README.md)。

在现有 StarArm → Lumos Python 遥操作循环中复用已经连接的 UART、CAN 和校准数据。主程序负责运动、滤波、限速及夹爪映射；`lumos_auto_feedback.py` 负责 Leader 阻尼、堵转保持及 HD 按钮仲裁，不单独打开端口或向 Lumos 发送运动指令。

## 安装、校准与启动

环境：Ubuntu 20.04 / 22.04 / 24.04，Python 3.10+（推荐 3.10），StarArm UART 和 Lumos CAN 连接。首次安装需要网络、`git` 及 `sudo`。

```bash
git clone https://github.com/VincentHSS/stararm102-force-feedback.git
cd stararm102-force-feedback/integrations/lumos
# 激活你准备使用的 Python/Conda 环境后安装
bash scripts/install_ubuntu.sh
```

脚本将官方 StarTouch SDK 安装到当前 Python 环境，源码保存在本优化版目录下的 `.dependencies/startouch_sdk`。SDK 配置需要其中的 `src/config/robot_kinematics.yaml`，请保留源码目录。

已有 SDK 可通过 `--sdk-root /实际路径/startouch_sdk` 复用，包括安装在原 Lumos 项目下的 SDK；不会向原项目写入优化代码。已经有可用 Python 环境和 SDK 时，无需再次运行安装脚本。

检查 CAN 和串口：

```bash
ip -details link show can0
ls -l /dev/ttyUSB*
```

CAN 尚未启用时配置为 1 Mbps；已启用时无需重复设置：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

确认工作区域畅通、急停可用，摆好两臂参考姿态后校准。连接 SDK 会使能 Lumos 电机，但校准不会下发运动目标：

```bash
python stararm102_lumos_teleop.py --calibrate --can can0 --leader-port /dev/ttyUSB0 --gripper
```

按提示记录参考位置、Leader 六关节活动范围及夹爪开合。默认校准路径是 `~/.config/stararm102_lumos/calibration.json`；已有校准文件可沿用，也可用 `--calibration-file /路径/calibration.json` 指定。

先预览检查映射：

```bash
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper
```

预览不发送 Lumos 目标，但仍连接 SDK，默认启用的 HD 按钮也会控制 Leader 保持/释放。确认后再使用下面的执行命令。

## 更新和执行

在本力反馈仓库目录更新代码；若本地有自行修改的映射、速度等参数，先保留这些修改再更新：

```bash
git pull --ff-only
```

然后进入 `integrations/lumos/`。两份 Python 文件必须放在一起：

```text
stararm102_lumos_teleop.py   # 原有遥操作入口
lumos_auto_feedback.py      # 带中文注释的自动反馈核心
```

使用原来的执行参数；自动反馈无需另开控制台或输入 LD 扭矩：

```bash
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper --execute
```

无 HD 按钮增加 `--button-disable`；无夹爪删除 `--gripper`，并与校准设置一致。预览和校准不启用自动反馈，HD 按钮维持原有行为。

启动提示缺少反馈接口时，更新项目中已安装的官方 SDK：

```bash
git -C .dependencies/startouch_sdk pull --ff-only
python -m pip install --upgrade ./.dependencies/startouch_sdk
```

需要排查旧 SDK 兼容性时，在原执行命令后加 `--feedback-disable`，恢复原有跟随和 HD 按钮控制。此选项关闭整套自动阻尼与自动堵转保持。

## 核心逻辑

1. 主循环读取 Leader；自动反馈启用时使用 FashionStar SDK 的 `realtime=True`，缺失反馈直接停止循环，避免把旧缓存当新输入。
2. 滤波、范围限制和限速仍按原来的代码生成下一帧目标。自动反馈比较的是**上一帧已发送的限速目标**与 Lumos 实际位置，不使用未限速的输入。
3. 用 `get_joint_positions()` 和 `get_joint_torques()` 读取六关节反馈。弧度换成角度，再除以跟随增益，得到 Leader 等效角差。关节原点和映射符号对角差绝对值、同轴跟随位移比均抵消。
4. 负载为 `clamp((abs(torque) - floor) / (reference - floor), 0, 1)`，线性映射到 Leader 阻尼功率。SDK 中 Leader 功率为 mW，从臂力矩为 Nm，二者并非物理等价。
5. 角差达到阈值后观察一个时间窗口。只有跟随不足、负载足够，并连续达到设定窗口数才确认堵转。目标运动时计算有符号跟随位移比；目标静止时检查角差是否收敛，允许正常追赶。
6. 任一关节触发时保持所有参与自动反馈的关节，启用夹爪时也保持 Leader 夹爪。每隔设定时间重发保持请求；所有触发关节均连续满足释放角差条件后恢复阻尼。
7. 反馈异常时保持 Leader，并暂停发送新的 Lumos 关节及夹爪目标。已有目标由 SDK 继续处理；这不是从臂急停。反馈恢复且所有角差连续达标后才继续发送。
8. 正常退出或异常退出时，先尝试逐个释放自动反馈接管的 Leader 关节，再清理 Lumos、关闭 UART。单个释放失败不会阻止其余关节清理。

保持模式为 `0x11`，阻尼为 `0x12`，退出释放为 `0x10`。保持通过 SDK 模式指令实现，不另行写入 Leader 角度目标；指令写入成功不等于已经验证实际锁住。

## HD 按钮优先级

| 条件 | Leader 六关节 | 启用的 Leader 夹爪 |
| --- | --- | --- |
| 反馈故障或任一自动堵转未解除 | 保持 | 保持 |
| 自动状态正常，HD 按钮按下 | 保持 | 按负载阻尼 |
| 自动状态正常，HD 按钮松开 | 按负载阻尼 | 按负载阻尼 |

HD 只更新按钮状态，自动反馈统一决定并下发模式。因此松开 HD 不会释放仍处于堵转或反馈故障状态的 Leader。

## 夹爪

启用 `--gripper` 时要求 SDK 支持 `get_gripper_state()`，读取 `position`、`effort_nm`、`enabled`、`error_code`、`feedback_valid` 和 `feedback_age_ms`。开度 0..1 乘以校准记录的 Leader 夹爪行程后参与角差判断。`--invert-gripper` 已在主循环应用，传入核心的是实际已发送开度。

首次尚未发送夹爪目标时，以当前开度作为观察参考；此后比较上一帧已发送开度。反馈年龄默认不超过 250 ms。旧 SDK 没有这些字段时会进入反馈故障保持，应更新 SDK 或明确关闭自动反馈排错。

## 参数

在 `lumos_auto_feedback.py` 的 `FeedbackConfig` 中调整默认值，或在主程序顶部设置 `AUTO_FEEDBACK_CONFIG = FeedbackConfig(...)`。不需要在运行中手动输入扭矩。重启程序后生效。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `torque_reference_nm` | `(5,5,5,1,1,1,1)` | J1..J6、夹爪的归一化参考力矩；不是额定值或硬件保护值 |
| `torque_floor_nm` | `(0.1,0.1,0.1,0.05,0.05,0.05,0.05)` | 反馈负载底限 |
| `power_min_mw` / `power_max_mw` | `1` / `1200` | Leader 阻尼功率范围；协议中 0 有特殊含义 |
| `power_deadband_mw` | `20` | 功率变化小于此值时抑制重复写入 |
| `sample_hz` | `30` | 反馈采样最高频率，受主循环实际频率限制 |
| `lock_deg` / `release_deg` | `20` / `8` | Leader 等效角差的堵转观察/释放阈值 |
| `window_s` | `0.4` | 跟随观察窗口 |
| `tracking_ratio` | `0.35` | 实际有符号位移与已发送目标位移的最低正常比值 |
| `tracking_min_deg` | `1.5` | 目标位移低于此值时改看角差收敛 |
| `convergence_deg` | `1` | 静止目标下一个窗口应缩小的角差 |
| `minimum_load_ratio` | `0.3` | 堵转确认需要的最低归一化负载 |
| `confirm_windows` | `1` | 连续异常窗口数 |
| `release_stable_s` | `0.3` | 释放或故障恢复的连续稳定时间 |
| `hold_mw` / `relock_s` | `6000` / `1` | 保持功率与重发间隔 |
| `gripper_max_age_ms` | `250` | 夹爪反馈最大允许年龄 |

参考力矩、功率和阈值是待实机调试的默认值。力矩含重力、摩擦等影响，未做重力补偿，也不是直接接触力测量。保持重发依赖主循环，进程停止、通信失败或循环长时间阻塞时无法独立执行。

六关节位置/力矩接口没有向这层 Python 暴露每轴反馈时间戳及有效标志，因此只能检查维度、有限数值和调用异常，不能保证检测到返回有限旧值的 CAN 中断。夹爪则有明确的新鲜度检查。软件保持不替代硬件急停和 SDK 原有保护。

## 验证与接口来源

```bash
python -m unittest discover -s tests -v
```

模拟测试覆盖负载归一化、映射、堵转窗口、反向运动、稳定释放、HD 仲裁、夹爪新鲜度、反馈异常时停止下发，以及退出清理。尚未在实际 Lumos 和 Leader 上验证保持力度、阈值或通信中断行为。

接口依据 [Lumos 官方 Python API](https://github.com/lumos-open/startouch_sdk/blob/main/README_API.md) 和 [SingleArm 源码](https://github.com/lumos-open/startouch_sdk/blob/main/interface_py/startouchclass.py)，本次核对提交 `040412542c0b6b6fd2bb6dd652c0de6c666631be`。Leader 使用 `fashionstar-uart-sdk 1.3.12` 的 `send_sync_servo_monitor(..., realtime=True)` 和 `stop_on_control_mode()`。
