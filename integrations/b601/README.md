# B601-DM 独立 Python 遥操作

本目录取代此前的 B601 LeRobot 插件作为推荐入口。直接读取 StarArm UART，直接控制 B601 的达妙串口桥或 SocketCAN，不导入 LeRobot、`motorbridge` 或 `motorbridge-smart-servo`。

核心包括自适应滤波、输入死区、按时间限速、加速度与单帧步长限制、底载采样、自动阻尼、持续堵转保持、HD 按钮仲裁及通信故障处理。没有手动 LD 扭矩控制台。

**物体负载反馈提供了独立路径：**按姿态扣除本体自重和空载摩擦，再联合估计额外竖直负载。需要两轮明确空载的标定与独立验证，步骤见[自重扣除与负载反馈](../../docs/b601_payload.md)。旧配置默认保留 `legacy` 行为，此前展示包不等于该新算法。

Windows 统一使用高精度 QPC 计时，避免旧版 Python 的 15.625 ms 时间量化。目标轨迹限速与电机追赶速度分开配置；参考轨迹增加到点制动和反转处理。运动目标只来自主臂输入，不追加从臂位置误差补偿。

```bash
python -m pip install -r requirements.txt
python teleop.py --simulate
python teleop.py --simulate --simulate-stall 0
cp b601.example.json my_b601.json
# 编辑 my_b601.json 的串口/CAN 配置，摆好两臂参考姿态
python teleop.py --config my_b601.json --capture paired.json
python teleop.py --config paired.json
# 核对方向、限位和配对姿态后，在 paired.json 设置 calibration_verified=true
python teleop.py --config paired.json --execute
```

完整步骤与参数见 [B601 Python 教程](../../docs/b601_dm.md)。

| 文件 | 内容 |
| --- | --- |
| `teleop.py` | 配对姿态采集、预览、执行、模拟与 CSV 诊断入口 |
| `controller.py` | 滤波、运动限制、底载处理与堵转状态机，含中文核心注释 |
| `gravity.py` / `payload.py` | 姿态重力矩、固定空载模型、整臂负载估计与阻尼映射 |
| `payload_tools.py` | 官方模型获取、空载拟合、独立验证与新配置生成，不打开串口 |
| `bus.py` | Leader 新回包校验、零位/突跳处理、DM 编解码及两类 CAN 适配器 |
| `config.py` / `b601.example.json` | 参数验证及官方默认映射示例 |

默认使用达妙串口桥；`adapter=socketcan` 时将 `follower_port` 填为 `can0`。六关节需要 POS_VEL 模式，夹爪需要 FORCE_POS 模式，与 Seeed 官方默认一致。本程序只检查模式，不写电机零位或 Flash。

连续零角度只要本轮确有校验通过的回包就有效；没收到新回包时不使用旧缓存。短暂丢包跳过运动帧，持续超时才保持并锁存故障。故障后检查设备并重启，避免反复锁住/释放。

```bash
python -m unittest discover -s tests -v
```

88 项无硬件测试。旧跟随路径曾在 Windows + 本机串口实测约 90–95 Hz；新负载路径尚未完成空载标定和实机验收。新增路径是准静态负载估计，旧底载处理仍不等于完整重力补偿。
