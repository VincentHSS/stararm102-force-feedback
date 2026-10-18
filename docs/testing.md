# 测试与验证范围

## 无硬件回归

```bash
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python -m py_compile stararm102_ro.py
python stararm102_ro.py --help
```

测试使用 Python 标准库 `unittest`，共 52 项，无需连接机械臂，其中 Windows 旧编码回归在其他系统跳过。控制器用模拟对象记录串口方法调用；主循环测试替换串口工厂、控制器和 sleep，不访问真实设备。

覆盖内容：

- 观察窗口达到条件后触发堵转、正常同向滞后不锁、反向滑动不算跟随。
- 自动解除的持续确认、缺失读数打断解除计时、仅堵转关节模式补充锁定。
- `l` 与手动锁定登记、误差回落不解除手动锁定、反馈不覆盖锁定。
- `fb off` 保留显式锁定、通信失败后的锁力重发、重发沿用原始角度。
- SDK 缓存读数失效、多从臂最大偏差选择、从臂缺失读数标记。
- 输入排队顺序、队列满时保留已接受命令、七关节参考值与非法数值解析。
- 重定向输出不周期刷屏、锁定与解除阈值一致性，以及 Windows CP1252 输出环境下中文帮助可用。
- 主循环发送跟随目标、主臂读数失效时不下发新目标、正常循环退出释放串口。

## B601 官方 LeRobot 集成

28 项检查位于 `tests/test_official_auto_feedback.py`，覆盖自动负载阻尼、官方坐标与夹爪比例、堵转锁定和恢复、不可靠 Leader/缺失从臂反馈、SDK 异常，以及不改动原动作、退出清理和安装器备份/恢复。

本地另对下载的官方主循环源代码进行了钩子插入、语法编译和 AST 对比。测试没有导入完整 LeRobot 硬件环境，也没有打开串口。

```bash
python -m pip install ./integrations/lerobot_auto_feedback
python -m stararm_auto_feedback.install --help
```

## CI

Lumos 优化版另外有 34 项模拟测试，代码仅维护在本力反馈仓库的 `integrations/lumos/`，不放入原 Lumos 仓库。运行方式：

```bash
cd integrations/lumos
python -m unittest discover -s tests -v
```

覆盖负载阻尼、堵转窗口、稳定恢复、反向运动、HD 仲裁、夹爪反馈新鲜度、异常时暂停新目标及退出清理。根目录 52 项加 Lumos 34 项，共 86 项。

GitHub Actions 配置在 Ubuntu 和 Windows 上运行 Python 3.10 与 3.12，执行依赖安装、三种方案语法检查、两组共 86 项测试与命令行帮助。CI 结果以对应提交的 Actions 页面为准；配置覆盖的平台不等于已经完成实机适配。

## 当前验证状态

本地回归在 Windows / Python 3.12 环境运行通过；SDK 使用 `fashionstar-uart-sdk==1.3.12`。未连接真实机械臂进行锁力、负载反馈或保护性能测试。

模拟测试验证软件决策和调用顺序，不验证 UART 电气通信、固件实际保持能力、供电裕量、机械限位或可安全承受的负载。首次上机需要按教程逐步验证并记录设备型号、固件版本、供电、参考电流和使用参数。
