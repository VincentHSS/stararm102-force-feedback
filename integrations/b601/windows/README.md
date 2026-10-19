# Windows 空载标定入口

这些文件是便携包 `app/` 中的启动源码。便携包包含 Python、pyserial、官方模型及本机配置；源码本身不包含设备配对、标定数据或运行时。

关节编号：J1 底座左右转，J2 肩部抬落大臂，J3 肘部屈伸小臂，J4 腕部俯仰，J5 腕部左右偏转，J6 末端旋转，J7 夹爪。

## 操作

先完成[模型、配对和空载标定准备](../../../docs/b601_extra_load.md)。在 `integrations/b601/` 下放置本机已验证的 `paired.json`、固定版本的 `ReBot_Arm_DM.urdf`，并安装 pyserial。源码入口适用于 Windows 和 Damiao 串口桥。

```powershell
python windows/calibration_guide.py --collect train
python windows/calibration_guide.py --collect check
python windows/launcher.py --fit
python windows/demo.py
```

第一次回车确认两臂姿态对应、夹爪空载、无外部接触、人员离开活动范围，然后检查设备并启动跟随。每组准备时间不限；按窗口给出的 J2/J3/J4 动作调整，到位后再次按回车，开始累计 10 秒稳定数据。移动或反馈不稳定时暂停计时，四组完成后才停机。准备阶段的数据保留在原始 CSV 中，拟合只使用确认采样的时间段；提前结束的引导采样会被拒绝用于拟合。

采样入口会根据操作者确认的当前对应姿态重记主臂软件参考，并备份旧配置到 `logs/references/`。该步骤只读设备，不比较旧主臂参考的差值，不修改从臂角度基准。随后仍检查七轴状态、温度、限位、模式和通信。

本机已经复测的响应参数为前六轴 ACC/DEC 4/−4。入口在启动前备份，正常结束或异常退出时恢复并读回验证。参数恢复未完成时保留 `logs/profile.active`，再次启动会拒绝执行；使用下面的恢复入口，恢复后保持失能：

```powershell
python windows/launcher.py --restore
```

需要 J2 小范围启动回收时，按[回收条件](../../../docs/b601_payload.md)确认现场后显式添加 `--recover-shoulder`。该选项仅用于空载采样，默认关闭。

按 Esc、Ctrl+C 或使用便携包的“停止.cmd”结束。退出时等待参数恢复完成。窗口关闭时也会请求后台控制进程正常退出；进程异常终止后仍需检查恢复标记。

拟合入口生成 `extra_load` 模式：先扣除自重和摩擦，再按多轴额外负载证据增加阻尼，不显示公斤数。模型通过独立空载验证后才替换 `paired_payload.json`，旧配置备份到 `logs/references/`，每次拟合使用新的标定文件名。这只通过了空载验证，拿起、持续持物和放下效果仍须实测。

展示入口也提供不限时准备：按回车确认两臂对应且可以使能后，只重记主臂参考，保持从臂零位及训练基准，再启动跟随。展示期间没有固定倒计时；Ctrl+C 或“停止.cmd”结束。旧 Windows 展示包与原 Lumos 仓库不受此更新影响。
