# 安装与上机教程

## 1. 准备设备

准备 StarArm 102 主臂、从臂、数据连接和匹配的电源。默认每臂使用舵机 ID 0–6，对应显示的 J1–J7；HD 按键板默认 ID 7。

先支撑机械臂、确认两臂姿态和运动方向，清空活动范围。启动阶段会发送卸力和圈数重置指令；重置圈数不等于自动回零，也不会校准主从零位。软件堵转检测不能替代设备急停。

## 2. 安装环境

在仓库根目录执行：

```bash
python -m venv .venv
```

Linux：

```bash
source .venv/bin/activate
```

Windows PowerShell：

```powershell
.\.venv\Scripts\Activate.ps1
```

安装依赖并列出端口：

```bash
python -m pip install -r requirements.txt
python -m serial.tools.list_ports
```

依赖锁定为本地验证使用的 `pyserial==3.5`、`fashionstar-uart-sdk==1.3.12`。切换 SDK 版本后，需要重新检查监控回包与锁力指令行为。

Linux 遇到串口权限不足时，先检查端口权限与所属组。若系统使用 `dialout` 管理串口，可将用户加入该组并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

## 3. 配置端口

修改 `stararm102_ro.py` 顶部配置。Linux 示例：

```python
LEADER_PORT_NAME = "/dev/ttyUSB0"
FOLLOWER_PORT_NAME_Arr = ["/dev/ttyUSB1"]
```

Windows 示例：

```python
LEADER_PORT_NAME = "COM3"
FOLLOWER_PORT_NAME_Arr = ["COM4"]
```

多从臂示例：

```python
FOLLOWER_PORT_NAME_Arr = ["/dev/ttyUSB1", "/dev/ttyUSB2"]
```

所有从臂接收相同目标角度。堵转判断逐关节选择与目标偏差最大的从臂角度；电流取可用读数的最大绝对值。如果任一从臂缺少某关节角度，该关节本轮不参与判断和阻尼更新。

## 4. 先运行无硬件检查

```bash
python -m unittest discover -s tests -v
python stararm102_ro.py --help
```

这些命令不会打开设备串口。通过后再进行下一步。

## 5. 首次启动

可先关闭反馈与自动堵转，单独检查角度和主从方向：

```bash
python stararm102_ro.py --leader_type 102LD --no_feedback --no_stall
```

缓慢、小范围拖动主臂，确认从臂方向、姿态与末端行程。J7 目标会乘以 1.5，并限制到 0–90°；J1–J6 没有额外的软件角度限位，须与设备实际配置匹配。

配有按键板的 HD：

```bash
python stararm102_ro.py --leader_type 102HD
```

型号选择决定按键默认开关；仍可通过 `--button_enable`、`--button_disable` 和 `--button_id` 覆盖。

## 6. 检查手动锁定

保持工作区空旷、机械臂有支撑，在终端输入：

```text
st lock
st
st unlock
```

`st lock` 锁定全部七个主臂关节，不依赖从臂堵转条件。`st unlock` 解除锁定并恢复反馈阻尼、原手动阻尼或卸力状态。

`l` 只锁 J1–J6，且关闭电流反馈。不要通过持续增大外力来验证保持能力。未出现锁定状态应检查命令处理；已经显示锁定但保持力不足，应检查固件、供电和功率限制，见 [故障排查](troubleshooting.md)。

## 7. 开启反馈与堵转检测

默认启动已开启两项功能；如果按上述首次启动方式关闭过，可输入：

```text
fb on
st on
fb
st
```

参考电流是负载映射标尺，默认值为：

```text
J1   J2    J3    J4    J5    J6    J7
500  1500  3500  3000  1000  1000  1000  mA
```

这些是配置起点，不是额定电流或通用标定结果。应在设备允许的工作范围内，记录各关节空载及已知工作负载的电流，再设置参考值：

```text
fbmax 500,1500,3500,3000,1000,1000,1000
fbp 1 1200
fbq 30
```

电流越接近参考值，主臂对应阻尼越接近上限。`fbmax auto` 使用会衰减的峰值参考，适合观察，不能当作稳定的标定结果。

## 8. 理解自动锁定

默认逻辑：误差达到 20° → 观察 0.4 s → 判断从臂是否同向跟随或误差收敛 → 负载比例达到 30% → 确认堵转并锁定全部主臂关节。

自动锁定后，从臂对应目标维持在触发时的指令角度；不会主动停止从臂驱动。所有触发关节误差均连续 0.3 s 不超过 8° 后，解除锁定。

先用 `st` 查看原因，再改一个参数、观察一次结果。不要直接大幅提高保持功率或取消所有判据。调参规则见 [命令与参数](reference.md)。

## 9. 结束运行

`q` 或 Ctrl+C 退出主循环，清理路径会尝试使主臂卸力并关闭串口。先支撑好机械臂。程序退出没有为从臂下发专门的卸力或急停指令，仍需按设备流程停止和断电。

`n` 是“主臂卸力并关闭反馈与堵转检测”，不是退出，也不是整机急停。按 `n` 后恢复自动功能，需要分别执行 `fb on` 和 `st on`。
