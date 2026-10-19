# B601：扣除自重后的额外负载反馈

目标是空手抬臂尽量轻松，拿起物体后增加 Leader 阻尼。推荐模式为 `extra_load`：根据多轴额外负载的证据输出阻尼，不显示公斤数。直接运行 Python，不依赖 LeRobot，也不需要手动输入 LD 扭矩。

**当前状态：本机独立空载验证通过，实际拿起、持续持物和放下释放尚待验收。** 575 个训练样本拟合固定空载模型；1010 个独立复测样本中，额外负载判据触发数为零。将包含准备、移动的完整 30,258 帧复测记录回放到运行时观察器，额外阻尼输出也为零。超出标定范围等暂停判断的帧包含在回放中，这不代表所有姿态均已验证。

现场空手抬落已确认轻松；首次持物复测未达到预期。输出路径已补齐原 SDK 的独立阻尼功率命令：先发命令 24 选择停止后的控制模式，再发命令 9 设置阻尼功率，并每 0.5 秒刷新。负载确认允许不超过 50 ms 的短暂中断，但只累计有效连续证据的时间，缺失和动态帧不计入 0.2 秒。通信超时按最后有效回包计时，动态期间的估计保留按最后有效负载证据计时，避免混用两种时间导致过早清空输出。阈值、自重模型与训练噪声保持不变。

独立主臂输出自检已由操作者确认能明显感觉到阻尼；此结果只验证主臂输出链路，整臂自动持物反馈仍在复测。

## 代码逻辑

1. [`gravity.py`](../integrations/b601/gravity.py) 用官方 URDF、关节角度及底座重力方向计算六轴本体自重力矩。
2. [`payload_tools.py`](../integrations/b601/payload_tools.py) 仅用明确空载的训练数据拟合每轴零偏、力矩符号/比例、黏性与低速摩擦；剩余力矩保留正负号。各轴噪声尺度取训练残差绝对值的 95 分位，最低 0.03 Nm。
3. [`extra_load.py`](../integrations/b601/extra_load.py) 将剩余力矩按噪声归一化，再沿末端向下负载对应的多轴力矩方向作联合投影。只有证据分数大于 3、方向一致性大于 0.5，并连续满足 0.2 秒，才增加阻尼。
4. 阻尼按超出阈值的信号强度平滑增加，J2/J3 权重为 1，J4 为 0.35，其余为零。夹爪夹紧力不作为提起物体的证据。空载模型运行时固定，持物不会被在线学习成自重。
5. 连续稳定 0.5 秒后才判断新负载：六轴实际速度均不超过 0.05 rad/s、平滑加速度不超过 0.15 rad/s²，窗口内各轴位置变化不超过 0.5°。配置中更严格的速度/加速度限制仍生效。运动期间，已确认输出最多保持 1 秒后淡出。放下后的有效空载证据使阻尼以 0.12 秒时间常数下降。堵转、HD 保持和通信故障处理独立保留，优先于普通阻尼。

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

若软件显示负载但主臂没有明显手感，可先运行 `python windows/demo.py --check-damping`。它只读确认从臂七轴失能，不使能或移动从臂；主臂 J2/J3 按“低阻尼 5 秒、1000 mW 或配置上限中的较小值 12 秒、低阻尼 4 秒”进行手感比较，最后释放。操作者按回车后才开始，Ctrl+C 或“停止.cmd”可提前结束。此检查只验证主臂输出链路，不证明物体识别通过。

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

诊断同时保存从臂每轴 MOS/转子温度、主臂下发的模式/功率、主臂回报的功率/电流字段、有效确认时间和暂停判断涉及的关节。回报字段不是对阻尼命令的确认，不能代替实际手感验证。控制台显示从臂 J2/J3 实际角度与暂停原因，方便回到训练区域。

## 补充实际取放区域

若实际取放姿态经常提示 `outside calibration`，保持夹爪空载，按实际取物、刚离台面、抬起和放下前的姿态分四组补采：

```powershell
python windows/calibration_guide.py --collect workspace --recover-shoulder
python windows/launcher.py --validate-workspace
```

回收选项的动作与限制见上方说明；不需要时可省略。每组准备不限时，到位后回车，`settled_pose=1` 时才累计 10 秒。四组完成后，先放稳并支撑从臂，再回车停止；不会到点自动断使能。验证在停止后执行，不打开设备。

验证至少需要 80 个连续稳定、可观测的空载样本，可观测比例至少 70%，且没有满足额外负载判据的样本。失败时保留原配置。通过后只增加实测姿态各轴约 5° 内的邻域，不把不同姿态的各轴极值拼成整个大范围；自重系数、训练噪声和阈值保持固定。仍需独立测试持物与放下效果。

临时从运行日志截取空载区间时，可用 `windows/workspace_capture.py begin/finish` 标记操作者已确认的空载时段。开始要求日志仍在更新；完成时保存完整 CSV 快照、时间段及 SHA256。退出、日志轮换、未完成或哈希变化都会拒绝验证。混合持物日志不能整段标为空载。离线验证命令：

```powershell
python workspace_validation.py --config paired_payload.json --profile extra_load_profile.json --validation data/empty_interval.csv --output extra_load_workspace.json
```

超出训练范围约 5°、距行程边界不足 2°或不可观测时关闭普通额外负载阻尼。短暂丢帧不反复重新起阻，持续超时仍锁存故障。失能后 J2 若因自重略越上界，默认拒绝启动；确认空载、姿态对应和活动范围后，采样或展示入口可显式添加 `--recover-shoulder`。它仅允许上界外 3° 内回收至上界内 5°，最多移动 8°、限时 1.5 秒，其余轴保持起始位置。回收后只在本轮内重记主臂 J2 参考，避免又追回边界；从臂零位、训练角度基准和调用方配置保持不变。回收期间主臂明显移动、其他轴漂移、超时或状态异常时退出断使能，不放宽后续跟随限位。`--check-damping` 不接受此选项。

改变工具、安装方向、从臂零位或映射后需重新训练验证；只重记主臂对应参考不会改变自重模型角度基准。标定文件在打开设备前检查算法、模型哈希、映射签名、参数和空载验收结果。

## 实机验收

先在已训练区域空手慢抬、慢落和停住，确认没有额外阻尼；再拿起合适物体，停住约 1 秒，轻轻转动主臂比较阻尼，放下后确认释放。阻尼抵抗运动，静止时不产生持续托举力。软件模拟与空载回放不能代替实物验收。

本机 2026-10-19 记录：主臂 J2/J3 独立阻尼自检已有明确手感，自动持物反馈尚未通过。新增实际取放区域的空载记录，在原较宽运动筛选下出现 50/275 个额外负载候选；按连续稳定条件仅剩 19 个样本，未达到 80 个要求，因此没有启用该区域。便携包仍是待有效空载补采和持物复测版本。
