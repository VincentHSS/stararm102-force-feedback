# B601：扣除自重后的额外负载反馈

目标是空手抬臂尽量轻松，拿起物体后增加 Leader 阻尼。推荐模式为 `extra_load`：根据多轴额外负载的证据输出阻尼，不显示公斤数。直接运行 Python，不依赖 LeRobot，也不需要手动输入 LD 扭矩。

**当前状态：本机独立空载验证通过，实际拿起、持续持物和放下释放尚待验收。** 575 个训练样本拟合固定空载模型；1010 个独立复测样本中，额外负载判据触发数为零。将包含准备、移动的完整 30,258 帧复测记录回放到运行时观察器，额外阻尼输出也为零。超出标定范围等暂停判断的帧包含在回放中，这不代表所有姿态均已验证。

## 代码逻辑

1. [`gravity.py`](../integrations/b601/gravity.py) 用官方 URDF、关节角度及底座重力方向计算六轴本体自重力矩。
2. [`payload_tools.py`](../integrations/b601/payload_tools.py) 仅用明确空载的训练数据拟合每轴零偏、力矩符号/比例、黏性与低速摩擦；剩余力矩保留正负号。各轴噪声尺度取训练残差绝对值的 95 分位，最低 0.03 Nm。
3. [`extra_load.py`](../integrations/b601/extra_load.py) 将剩余力矩按噪声归一化，再沿末端向下负载对应的多轴力矩方向作联合投影。只有证据分数大于 3、方向一致性大于 0.5，并连续满足 0.2 秒，才增加阻尼。
4. 阻尼按超出阈值的信号强度平滑增加，J2/J3 权重为 1，J4 为 0.35，其余为零。夹爪夹紧力不作为提起物体的证据。空载模型运行时固定，持物不会被在线学习成自重。
5. 快速加减速时不确认新负载；已确认输出最多保持 1 秒后淡出。放下后的有效空载证据使阻尼以 0.12 秒时间常数下降。堵转、HD 保持和通信故障处理独立保留，优先于普通阻尼。

记 `r` 为扣除自重/摩擦后的有符号力矩，`s` 为空载噪声，`h` 为单位末端竖直负载的关节力矩方向。激励充分的轴参与计算：

```text
y = r / s
u = h / s
score = dot(u, y) / norm(u)
coherence = dot(u, y) / (norm(u) * norm(y))
strength = clamp((score - 3) / 3, 0, 1)  # 通过方向与持续时间检查后
```

分数与阻尼比例都不是质量。较轻物体、信号弱的姿态或偏置重心可能不触发；人的向下施力、某些接触力也可能与物体载荷相似。该路径是准静态额外负载反馈，不能严格识别物体或替代力传感器。

此前的[公斤数估计](b601_payload.md)保留为研究模式 `payload`，仍使用独立的严格质量不确定度门槛。本机数据未通过该门槛；没有通过放宽质量门槛启用它。新模式采用独立格式 `b601-extra-load-v1`，只声明空载验证状态。

## 模型与两轮空载数据

在 `integrations/b601/` 目录执行：

```powershell
python payload_tools.py fetch-model --output payload_data/ReBot_Arm_DM.urdf
```

下载并校验固定提交 `0b0b29ef92b4e03f0553dbe7502a8afe164dbb29` 的 [Seeed 模型](https://github.com/Seeed-Projects/reBot-DevArm/blob/0b0b29ef92b4e03f0553dbe7502a8afe164dbb29/Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf)。默认底座 Z 轴向上，电机零位对应 URDF `joint1..joint6`；安装与实际零位不同时，先核对拟合命令的 `--directions`、`--offsets-deg`、`--gravity`。这些参数只改变软件计算。

沿用本机已验证的 `paired.json`，两臂处于对应姿态，夹爪空载且没有外部接触。Windows 推荐[引导入口](../integrations/b601/windows/README.md)，每组准备时间不限，到位后按回车才累计稳定数据：

```powershell
python windows/calibration_guide.py --collect train
python windows/calibration_guide.py --collect check
```

第一次回车确认姿态、空载和活动范围后才检查并使能。每组累计 10 秒稳定数据，移动时暂停；四组全部完成后停机。数据元信息记录确认时间段，拟合排除准备时间，并拒绝未完成的引导采样。

| 关节 | 对应动作 |
| --- | --- |
| J1 | 底座左右转 |
| J2 | 肩部抬落大臂 |
| J3 | 肘部屈伸小臂 |
| J4 | 腕部俯仰，夹爪点头 |
| J5 / J6 / J7 | 腕部左右偏转 / 末端旋转 / 夹爪 |

训练需分别改变 J2、J3、J4，肩肘各覆盖至少 15°。固定 J2 单独动 J3，再固定 J3 单独动 J2，避免肩肘同时移动却让小臂一直水平。缓慢往返，在多个姿态停住；无需接近机械限位。默认使用速度不超过约 14°/秒、加速度较小且六轴距软件行程边界至少 2° 的样本。

普通 Python 命令也可采集，下面是有固定时长的入口，不含不限时分组准备：

```powershell
python teleop.py --config paired.json --execute --collect-unloaded payload_data/train.csv --confirm-unloaded --duration 60
python teleop.py --config paired.json --execute --collect-unloaded payload_data/check.csv --confirm-unloaded --duration 60
```

两轮必须独立、明确空载且覆盖相近区域。工具验证 CSV 元信息与从臂映射签名，拒绝训练与复测使用同一文件、未知载荷日志或单姿态数据。保存的数据不会自动覆盖。

## 拟合与运行

```powershell
python payload_tools.py fit --mode extra_load --config paired.json --model payload_data/ReBot_Arm_DM.urdf --train payload_data/train.csv --validation payload_data/check.csv --output payload_data/extra_load_profile.json
python payload_tools.py enable --config paired.json --profile payload_data/extra_load_profile.json --output paired_payload.json
python payload_tools.py inspect --config paired_payload.json --profile payload_data/extra_load_profile.json
python teleop.py --config paired_payload.json --execute --diagnostics-file payload_data/extra_load_run.csv
```

至少需要 120 个训练与 80 个独立复测有效样本，三个重力激励充分的关节，复测中至少 70% 的样本处于训练覆盖范围且有可观测的负载方向。独立空载复测不得出现满足额外负载判据的样本。噪声尺度、拟合系数和固定阈值不会根据复测结果重新调高以通过验证。

Windows 展示入口为 `python windows/demo.py`。准备完成后按回车，先只读重记主臂参考，再检查和启动；不修改从臂零位与训练角度基准。Ctrl+C、关闭窗口或便携包“停止.cmd”请求退出并恢复临时响应参数。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `feedback_mode` | `legacy` | 新生成配置为 `extra_load`；`payload` 为公斤数研究模式 |
| `payload_calibration` | 空 | 对应模式的标定文件，相对配置文件解析 |
| `payload_haptic_weights` | `[0,1,1,0.35,0,0,0]` | 联合负载证据映射到主臂各轴的阻尼权重 |
| `payload_max_speed_rad_s` | 0.25 | 新负载确认的最大实际速度 |
| `payload_max_accel_rad_s2` | 0.6 | 平滑实际速度的加速度门槛 |
| `payload_hold_s` | 1 秒 | 动态期间保留输出的最长时间 |
| `payload_smoothing_s` | 0.25 秒 | 阻尼增加的平滑时间常数 |

`payload_reference_kg` 仅用于研究模式，`extra_load` 不使用它。新模式 CSV 的公斤数列为空值 NaN；查看 `extra_load_score`、`extra_load_coherence`、`extra_load_strength`、`external_j*_nm` 与 `haptic_j*`。旧 `load` 列仍为堵转保护指标。

超出训练范围约 5°、距行程边界不足 2°或不可观测时关闭普通额外负载阻尼。短暂丢帧不反复重新起阻，持续超时仍锁存故障。失能后 J2 若因自重略越上界，默认拒绝启动；空载采样可在确认现场后显式使用 `--recover-shoulder`，仅允许上界外 3° 内回收至上界内 5°，最多移动 8°、限时 1.5 秒，其余轴保持起始位置。该回收不用于自动绕过展示入口的限位检查。

改变工具、安装方向、从臂零位或映射后需重新训练验证；只重记主臂对应参考不会改变自重模型角度基准。标定文件在打开设备前检查算法、模型哈希、映射签名、参数和空载验收结果。

## 实机验收

先在已训练区域空手慢抬、慢落和停住，确认没有额外阻尼；再拿起合适物体，停住观察持续阻尼，放下后确认释放。软件模拟与空载回放不能代替实物验收。当前便携包是待实物复测版本，不宣称完成持物效果验证。
