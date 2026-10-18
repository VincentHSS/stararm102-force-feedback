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

## B601 历史 LeRobot 集成

以下 30 项保留用于旧版兼容和卸载器验证。当前 B601 推荐直接 Python 入口，见下方独立 Python 版一节。

30 项检查位于 `tests/test_official_auto_feedback.py`，覆盖自动负载阻尼、官方坐标与夹爪比例、堵转锁定和恢复、不可靠 Leader/缺失从臂反馈、SDK 异常，以及不改动原动作、退出清理和安装器备份/恢复。

本地另对下载的官方主循环源代码进行了钩子插入、语法编译和 AST 对比。测试没有导入完整 LeRobot 硬件环境，也没有打开串口。

```bash
python -m pip install ./integrations/lerobot_auto_feedback
python -m stararm_auto_feedback.install --help
```

## Lumos 独立优化版

Lumos 优化版另外有 34 项模拟测试，代码仅维护在本力反馈仓库的 `integrations/lumos/`，不放入原 Lumos 仓库。运行方式：

```bash
cd integrations/lumos
python -m unittest discover -s tests -v
```

覆盖负载阻尼、堵转窗口、稳定恢复、反向运动、HD 仲裁、夹爪反馈新鲜度、异常时暂停新目标及退出清理。根目录 52 项加 Lumos 34 项，共 88 项。

## B601 独立 Python 版

```bash
# 从仓库根目录进入
cd integrations/b601
python -m unittest discover -s tests -v
python -S teleop.py --simulate
```

目前 88 项，覆盖连续有效零位、缺包、校验错误、突跳、DM 模式与量程回读、夹爪 FORCE_POS 编码、滤波/运动限制、高空载底载、慢速正常跟随、持续堵转、故障锁存、只读采集/预览以及伪 UART/CAN 完整执行与清理。新增 30 项覆盖重力势能梯度、末端载荷、符号/摩擦标定、独立空载验证、物体持续保持与放下、空手抬臂、快速运动、标定范围和故障优先。模拟在禁用第三方包的子进程中运行，确认不需要 LeRobot 或 MotorBridge SDK。

测试保留了旧版 52 项、Lumos 34 项，加新 Python B601 88 项，共 174 项。

## CI 执行范围

GitHub Actions 配置在 Ubuntu 和 Windows 上运行 Python 3.10 与 3.12，执行依赖安装、三种方案语法检查、三组共 174 项测试与命令行帮助。CI 结果以对应提交的 Actions 页面为准；配置覆盖的平台不等于已经完成实机适配。

## 当前验证状态

本地回归在 Windows / Python 3.12 环境运行通过；原 StarArm 路径使用 `fashionstar-uart-sdk==1.3.12`。B601 直接 Python 路径不依赖该 SDK，现已完成 COM 串口只读通信与遥操作试运行，观察并处理了 Windows 计时量化及部分软件跟随延迟；仍未完成动态跟随、锁力、负载反馈或保护性能的系统实机验证。

新整臂负载路径已做合成数据验证，官方 URDF 的重力矩也通过势能有限差分核对；尚无本机两轮明确空载数据和已知重量验收。合成数据标定文件不作为真实机械臂的标定文件发布。

模拟测试验证软件决策和调用顺序，不验证 UART 电气通信、固件实际保持能力、供电裕量、机械限位或可安全承受的负载。首次上机需要按教程逐步验证并记录设备型号、固件版本、供电、参考电流和使用参数。
