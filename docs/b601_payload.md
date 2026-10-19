# B601：本体自重扣除与物体负载反馈

本页保留公斤数估计研究模式 `payload`。当前推荐[额外负载反馈 `extra_load`](b601_extra_load.md)，优先区分空载与额外负载，不输出公斤数。本机独立空载验证通过的是 `extra_load`，本页质量估计未通过实机不确定度门槛。

目标是空手抬臂时尽量轻松，拿起物体后再增加 Leader 阻尼。新路径使用有正负号的关节力矩、实际姿态和速度，联合估计整臂额外竖直负载，不再将各轴电流分别映射为各轴阻尼。

**当前：代码和模拟回归已实现，本机空载标定及已知物体实机验证尚未完成。此前的 Windows 双击展示包仍是旧算法。** 没有通过验证的标定文件时，新模式在打开串口前拒绝运行。

## 计算逻辑

1. 从官方 URDF 的质量、重心、关节轴和实际角度计算本体静态重力矩。
2. 用明确空载的数据拟合力矩零偏、符号/比例和低速摩擦。先相减，保留力矩方向。
3. 联合多个关节拟合额外竖直负载，同时投影掉物体未知重心产生的水平力矩，减少固定重心假设的误差。
4. 用独立空载复测误差设定可分辨负载死区，持续确认、平滑后统一映射为阻尼。默认 J2、J3 权重为 1，J4 为 0.35，其余为零；夹爪电流不参与质量估计。
5. 空载模型固定，不在线把物体重量学习成自重。原堵转、HD 保持和通信故障处理继续工作，保持优先于普通阻尼。

这是**准静态近似负载估计**，不是完整逆动力学的高速称重。快速加减速时不增加新估计，短暂保留可信结果后淡出。仅凭电流和姿态，不能严格区分物体重量、人的向下施力与某些竖直接触力；该估计不能作为独立安全传感器。

## 1. 获取官方模型

在 `integrations/b601` 目录执行：

```powershell
python payload_tools.py fetch-model --output payload_data/ReBot_Arm_DM.urdf
```

工具下载固定提交 `0b0b29ef92b4e03f0553dbe7502a8afe164dbb29` 的 [Seeed 模型](https://github.com/Seeed-Projects/reBot-DevArm/blob/0b0b29ef92b4e03f0553dbe7502a8afe164dbb29/Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf)，校验 SHA-256 后保存，后续运行无需联网。模型按上游许可使用，模型参数不等于本机已完成标定。

默认底座 Z 轴向上，电机零位与 URDF `joint1..joint6` 对应。安装方向或零位不同，先核对，再使用拟合命令的 `--directions`、`--offsets-deg`、`--gravity`；这些参数只影响软件计算，不修改电机零位、PID、模式或 Flash。

## 2. 两轮明确空载的数据

沿用已核对的 `paired.json`，两臂回到对应参考姿态、夹爪不夹物、没有外部接触。以下命令会实际使能和跟随；采集中普通阻尼为最低值，堵转与通信保护保留。

```powershell
python teleop.py --config paired.json --execute --collect-unloaded payload_data/empty_train.csv --confirm-unloaded --duration 60
```

缓慢覆盖实际使用区域：J2、J3 各至少 15°，同时改变肘部与腕部组合，每个姿态停 3–4 秒并往返采样。默认只使用速度不超过约 14°/秒、加速度较小的片段。六个臂关节距软件行程边界不足 2° 的样本不参与标定，避免边界处的机械支撑或限位预紧影响自重拟合。不要反复停在完全折叠的零位；无需扫到机械限位，未覆盖区域不会强行外插。

关节从底座数起：J1 底座左右转、J2 肩部抬落大臂、J3 肘部屈伸小臂、J4 腕部俯仰、J5 腕部左右偏转、J6 末端旋转、J7 夹爪。肩肘始终同时移动，可能让小臂一直保持水平，虽然两个关节角度都变化了，仍不足以辨识肘轴自重。可分四组采样，每组移动约 5 秒、停住约 10 秒：J2/J3 都离开折叠位；固定 J2 只动 J3，让小臂明显倾斜；固定 J3 只动 J2；最后改变 J4 俯仰。所有动作都在已确认的使用区域内完成。

[Windows 引导入口](../integrations/b601/windows/README.md)允许每组不限时准备，由操作者再次按回车确认到位后才开始累计稳定数据；完成四组后退出。它没有上述固定 60 秒的准备限制。引导数据的 `.meta.json` 保存采样时间段和完成状态，拟合排除准备时间，并拒绝未完成的引导采样。

结束后重新摆回配对参考姿态，做第二轮独立空载复测，覆盖同一区域：

```powershell
python teleop.py --config paired.json --execute --collect-unloaded payload_data/empty_check.csv --confirm-unloaded --duration 60
```

CSV 与 `.meta.json` 记录用途和配对签名。旧展示日志、未知是否夹物的日志、单姿态底载不能用于此标定；工具拒绝覆盖已有采集文件。

如果 J2 在失能时因自重略微越过软件上界，先查看实际角度、状态和现场摆放。默认启动仍拒绝越界。确认两臂对应、双手离开并清空活动范围后，可给执行命令显式添加 `--recover-shoulder`：仅允许 J2 高于上界不超过 3°，先使能七轴并将 J2 目标设在上界内 5°，其余轴保持起始位置；最多移动 8°。回收限时 1.5 秒，位置、状态、温度异常时退出并尝试断使能。此选项不写电机零位、模式或 Flash，也不放宽后续跟随的软限位。

## 3. 拟合、独立验证、生成配置

```powershell
python payload_tools.py fit --mode payload --config paired.json --model payload_data/ReBot_Arm_DM.urdf --train payload_data/empty_train.csv --validation payload_data/empty_check.csv --output payload_data/unloaded_profile.json
python payload_tools.py enable --config paired.json --profile payload_data/unloaded_profile.json --output paired_payload.json
```

需要滤波后至少 120 个训练、80 个独立复测样本，至少三个有足够重力变化的关节及可辨识负载的姿态。复测至少 70% 样本可用，空载质量 RMSE 不超过 0.1 kg、绝对误差 99 分位不超过 0.2 kg。实际死区取复测误差与 0.05 kg 中较大值。这些是软件启用门槛，并非精度承诺。

失败时核对空载状态、覆盖范围、模型映射和力矩量程，不手工修改 `validated`。更换末端工具、改变安装方向、模型、配对参考或零位后需重新标定和验证。

## 4. 检查和执行

```powershell
python payload_tools.py inspect --config paired_payload.json --profile payload_data/unloaded_profile.json
python teleop.py --config paired_payload.json --simulate --simulate-payload-kg 0.5
python teleop.py --config paired_payload.json --execute --diagnostics-file payload_data/payload_run.csv
```

模拟不开硬件，只验证软件链路；模拟姿态超出标定范围时，反馈会正确暂停。实机退出用 Ctrl+C，程序断使能从臂并释放 Leader。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `feedback_mode` | `legacy` | 兼容旧配置；生成的新配置为 `payload` |
| `payload_calibration` | 空 | 标定文件，路径相对配置文件解析 |
| `payload_reference_kg` | 0.75 | 最大阻尼对应的估计负载，不是额定承重 |
| `payload_haptic_weights` | `[0,1,1,0.35,0,0,0]` | 整臂估计映射到 Leader 各轴的权重 |
| `payload_max_speed_rad_s` | 0.25 | 超过该速度不增加新估计 |
| `payload_max_accel_rad_s2` | 0.6 | 平滑实际速度的加速度门槛 |
| `payload_hold_s` | 1 秒 | 动态期间保留可信估计的最长时间 |
| `payload_smoothing_s` | 0.25 秒 | 阻尼增加的平滑时间常数 |

CSV 增加速度、采集用途、`payload_kg`、原始估计、状态、拟合误差、不确定度、六轴有符号剩余力矩及实际阻尼比例 `haptic`。原 `load` 列是旧堵转保护指标，不是物体重量。

超出标定范围约 5°、距行程边界不足 2°、不可辨识或力矩无法解释时暂停重量反馈。短暂丢帧不反复清空已确认的重量并重新起阻，持续超时仍锁存故障。边界处暂停的是普通重量阻尼，堵转与通信故障保持仍由原保护逻辑处理。

此筛选策略使用 `b601-quasistatic-payload-v2` 标定格式；旧版标定需重新拟合和独立验证。

## 实机验收

先空手在标定区域内缓慢抬落、停住，检查普通阻尼保持低值；再用已知重量的物体重复相近轨迹，验证阻尼增加、持续拿住和放下释放。最后单独验证堵转与通信故障。当前尚未完成这些本机验收，不能用模拟通过替代。
