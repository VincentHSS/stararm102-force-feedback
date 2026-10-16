# 命令与参数参考

## 编号与单位

显示 J1–J7 对应舵机 ID 0–6；命令 `3=800` 中的 `3` 是 ID，因此控制 J4。角度为 °，电流为 mA，功率为 mW，时间为秒，带 `Hz` 的值为频率。

## 基本命令

| 命令 | 行为 |
|---|---|
| `h` | 显示帮助 |
| `s` | 总体状态 |
| `q` | 退出，尝试主臂卸力并关闭串口 |
| `n` | 主臂卸力，关闭电流反馈和堵转检测；从臂继续跟随 |
| `l` | 锁定 J1–J6，关闭电流反馈 |
| `500` | 全部关节设为 500 mW 手动阻尼，关闭电流反馈 |
| `500 600 700 800 900 1000 1100` | 按 J1–J7 指定手动阻尼 |
| `3=800 4=900` | 按 ID 指定手动阻尼，建议使用这种明确语法 |

手动输入阻尼会清除已有锁定，并将没有被此次功率设置覆盖的旧锁定关节恢复到原状态。不要用 `0` 表示卸力；协议的 `0` 使用功率保护阈值。两数字等简写具有解析优先级，教程统一采用 `id=功率` 或完整七个值。

## 电流反馈

| 命令 | 默认值或说明 |
|---|---|
| `fb` / `fbs` | 电流、峰值、映射功率和采样状态 |
| `fb on` | 开启电流反馈，保留已有锁定 |
| `fb off` | 关闭反馈并释放未锁定关节；不解除已有锁定 |
| `fbmax 3000` | 所有关节共用一个参考电流 |
| `fbmax 500,1500,3500,3000,1000,1000,1000` | 各关节参考电流 |
| `fbmax auto` | 使用衰减峰值参考 |
| `fbp 1 1200` | 最小/最大反馈功率；下限 0 按 1 处理 |
| `fbq 30` | 电流噪声门槛，默认 30 mA |
| `fbdb 20` | 功率更新死区，默认 20 mW |
| `fbhz 30` | 从臂监控采样频率，默认 30 Hz |

## 堵转与锁力

| 命令 | 默认值或行为 |
|---|---|
| `st` | 当前误差、负载、锁定和判定原因 |
| `st on` / `st off` | 开关堵转检测；关闭会解除已有锁定 |
| `st reset` | 清除锁定和检测记录并恢复原状态 |
| `st lock` | 手动锁定全部七个关节，不因误差回落自动解除 |
| `st unlock` | 解除已有锁定，恢复反馈/手动阻尼/卸力 |
| `stlock 20` | 进入观察的误差阈值，默认 20° |
| `strel 8` | 自动解除误差阈值，默认 8° |
| `stn 1` | 连续未跟随窗口数，默认 1 |
| `stwin 0.4` | 观察窗口，默认 0.4 s |
| `sttrack 0.35` | 同向跟随比例阈值，默认 0.35 |
| `sttrackmin 1.5` | 主臂有效位移门槛，默认 1.5° |
| `stload 0.3` / `stload off` | 最小负载比例，默认 0.3；off 取消此门槛 |
| `ststatic 1` / `ststatic off` | 静止时误差收敛门槛，默认 1°；off 取消收敛豁免 |
| `stscope all` / `stscope stalled` | 锁全部关节/仅堵转关节，默认 all |
| `sthold 6000` | 保持功率，默认 6000 mW，仍受固件保护阈值约束 |
| `stmethod both` | 停止保持与位置保持同时使用，默认 both |
| `stmethod stop` / `stmethod hold` | 仅停止保持/仅位置保持 |
| `strelock 1` | 重发间隔，默认 1 s；0 关闭重发 |

自动解除的持续时间为顶部常量 `STALL_RELEASE_HOLD_S=0.3`，没有对应终端命令。运行时修改参数只影响本次进程；要持久化请修改顶部常量或保存启动参数。

## 启动参数

运行完整帮助：

```bash
python stararm102_ro.py --help
```

| 参数 | 默认 |
|---|---|
| `--leader_type 102LD\|102HD` | 102LD |
| `--button_enable` / `--button_disable` | 随型号；disable 最后覆盖 |
| `--button true\|false` | 使用型号默认 |
| `--button_id` | 7 |
| `--filtered_size` | 1，必须 ≥ 1 |
| `--damping` | 未指定；指定则关闭初始电流反馈 |
| `--damping_refresh` | 0，不重发手动阻尼 |
| `--no_console` | 默认启用终端输入 |
| `--no_feedback` | 默认启用电流反馈 |
| `--fb_hz` | 30 |
| `--fb_power_min` / `--fb_power_max` | 1 / 1200 |
| `--fb_current_max` | 顶部各关节列表；支持逗号列表或 auto |
| `--fb_current_floor` | 30 |
| `--fb_deadband` | 20 |
| `--fb_auto_tau` | 10 |
| `--no_stall` | 默认启用堵转检测 |
| `--stall_lock_deg` / `--stall_release_deg` | 20 / 8 |
| `--stall_confirm` | 1 |
| `--stall_track_win` / `--stall_track_ratio` | 0.4 / 0.35 |
| `--stall_track_min_deg` | 1.5 |
| `--stall_lock_method` | both |
| `--stall_lock_scope` | all |
| `--stall_hold_mw` / `--stall_relock_s` | 6000 / 1 |
| `--stall_min_load_ratio` | 0.3，支持 off |
| `--stall_static_deg` | 1，支持 off |

`--no_console` 关闭的是输入线程，启动参数配置的阻尼和自动检测仍执行。端口和波特率目前通过文件顶部常量配置，没有 `--port` 参数。

示例：

```bash
python stararm102_ro.py --leader_type 102LD --filtered_size 3 --fb_hz 20
python stararm102_ro.py --no_console --damping 500 --no_stall
```

## 调参方向

- 观察窗口或确认次数增大：响应更慢，能过滤部分短时滞后。
- 跟随率门槛增大：更容易将较慢的从臂判为未跟随。
- 参考电流增大：同一电流的反馈阻尼更小，也更难通过负载门槛。
- 提高保持功率：只改变指令请求，实际能力仍由供电、固件保护和机械结构决定。

锁定不稳定时，先区分“未触发锁定”和“已下发锁定但保持不足”，不要同时修改多个判据。
