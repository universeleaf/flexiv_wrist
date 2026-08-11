# Flexiv 7DoF / 9DoF 实时扭矩 OSC 与腕部惯量辨识

本文对应本机工程，不使用 Bota。Flexiv 机械臂的实时关节力矩环运行在官方 C++ RDK 的
`RT_JOINT_TORQUE` 模式；Python RDK 1.9 只负责启动和腕部 CAN-FD 桥接，不能替代 Flexiv
的 1 kHz 实时接口。

官方参考：

- [Flexiv RDK 控制模式](https://www.flexiv.com/software/rdk/manual/v2.x/control_modes.html)
- [Flexiv 实时系统安装](https://www.flexiv.com/software/rdk/manual/v2.x/installation/real_time_system.html)
- [Flexiv Robot API](https://www.flexiv.com/software/rdk/manual/v2.x/api/robot.html)

## 1. 本次实现的控制结构

### 1.1 7DoF：Flexiv 关节力矩 + 操作空间控制

实时状态为 `q, dq`，Flexiv `Model` 每 1 ms 更新。当前默认演示控制真实 TCP；程序在
进入实时模式之前读取机械臂当前 TCP，并将它保存为本次运行的操作空间固定点：

```text
p_fixed = p_tcp(t_start)
p_des(t) = p_fixed
v_des(t) = 0
a_des(t) = 0
```

随后执行：

```text
M(q), c(q,dq), J_tcp(q)
Lambda = (J M^-1 J^T + lambda^2 I)^-1
tau_task = J^T Lambda [xdd_des + Kp e + Kd (xd_des - J dq)]
tau = tau_task + N^T [Kq(q0-q)-Dq dq] + c(q,dq)
```

Flexiv 内部重力补偿保持启用，关节软限位保持启用。C++ 线程通过官方 `Scheduler` 以
1 kHz 运行，并绑定到指定 CPU。默认 `orientation` 轨迹不给 TCP 任何平移命令，只做
闭合的 X/Y/Z 姿态摆动。启动后的前 2 秒（或周期的 1/4，取较小值）用五次速度曲线平滑
进入周期轨迹，随后一直重复，只有 Ctrl-C、机器人故障或通信故障才停止。

固定点不是写死的世界坐标。每次运行前，可以先用 Elements/freedrive 把机械臂移动到新的
位置；脚本会把该次启动时的当前 TCP 采集为新固定点。进入 `RT_JOINT_TORQUE` 后，该点由
OSC 位置环保持，不能再用手自由拖动。若要在程序运行中由 omega.7 或手推导纳持续更新
固定点，需要接入一个单独的位置参考输入，不能仅靠“OSC”三个字自动实现。

### 1.2 9DoF：Flexiv + 两轴腕部复合 OSC

广义坐标为：

```text
q9 = [q_arm(7), q8, q9]
```

程序按实时腕部角度计算探针正运动学、复合 `6x9` Jacobian。矩阵维度必须区分：

```text
单个刚体的空间惯量                 I_spatial: 6x6
完整 9DoF 的关节空间质量矩阵       M9(q):      9x9
末端任务的操作空间惯量             Lambda(q):  6x6
```

旧代码把 `M9` 写成 `blockdiag(M_flexiv,M_wrist)`，交叉块全为零。当前默认
`--inertia-mode auto` 会用两个活动刚体各自的 6x6 空间惯量和完整质心 Jacobian 形成
主臂—腕部交叉项。Flexiv `Model::M()` 已包含 Elements active Tool，因此自动模式把它作为
q8=q9=0 的已标定基线，只加入腕部姿态变化带来的惯量差和交叉项，避免重复加入 payload：

```text
M9(q) = M_RDK_with_Elements_tool
        + M_moving_wrist(q) - M_moving_wrist(q8=q9=0)
        + arm/wrist cross blocks + reflected motor inertia
Lambda = (J9 M9^-1 J9^T + mu_auto^2 I)^-1
```

`mu_auto` 根据 6x6 逆操作空间惯量的特征值自动调节，把病态条件数限制在约 200，避免靠近
奇异位形时产生巨大的任务力矩。再用同一套动态一致 OSC 求 9 个关节力矩。质量矩阵采用
Elements 基线和通过质量门控的腕部辨识参数，不再通过任意缩小惯量来“假装腕部更轻”。
每个控制周期另做一次两轴阻尼最小二乘姿态 IK，
把目标探针姿态相对“启动时法兰姿态”尽可能分配给 q8/q9，再作为强零空间目标加入 OSC；
Flexiv 负责平移、腕部不能产生的剩余姿态，以及补偿腕部转动引起的探针位置变化。腕部
同时加入实时 `M_wrist(q)`、Coriolis、随法兰姿态变化的重力、辨识摩擦和力矩 bias。

auto 模式会先检查腕部辨识质量：RMS 力矩残差必须不大于 `0.15 Nm`、回归条件数不大于
`500`、刚体尺度在 `[0.5,1.5]`，且两轴反射惯量不能落到 `1e-6` 的拟合下限。当前
`wrist_inertia_calibration.json` 的 RMS 为 `0.266 Nm`，q8 反射惯量恰好落到下限，因此会
自动拒绝这组惯量项，改用 Elements/RDK payload 基线和 TOML 腕部先验；已辨识摩擦仍用于
克服减速器静摩擦。终端会打印 `自动惯量来源:`，不会静默采用坏参数。

这一点修复了旧实现中“腕部基本不动”的直接原因：旧目标是相对当前、已经在运动的法兰
计算的，同时零空间又把 q8/q9 拉回启动角。Flexiv 一旦先完成姿态，腕部目标便随之消失。
现在 q8/q9 不再被拉回启动角，并会持续追踪独立的姿态分配目标。

Flexiv 力矩流为 1 kHz；当前 mjbots fdcanusb 腕部桥为 100 Hz。共享内存使用序列锁防止
读到撕裂状态，但安装实时内核不会把 USB/CAN 桥自动变成 1 kHz。因此它是多速率 9DoF
OSC 演示，不应被描述成九轴全部 1 kHz 的生产控制器。配置项
`[wrist].rt_osc_state_timeout_ms=250` 为这段非实时 USB 桥保留调度余量，Python 同时以
100 Hz 刷新最近的力矩命令。超过 250 ms 的短时丢帧会令腕部力矩归零，并令 Flexiv 在
当前关节位置阻尼保持；数据恢复后自动继续 OSC。只有连续超过
`[wrist].rt_osc_hard_timeout_ms=2000` 或 bridge 本身退出才安全停止。

启动时 Python 会只读查询 live Elements Tool，验证质量、CoM、3x3 转动惯量和 TCP，然后
用 live TCP 自动覆盖零位末端偏置与姿态。当前实机 Tool `Wrist` 的 TCP 是
`[0.0166, 0.0469, 0.1448, 1, 0, 0, 0]`（相对 flange）。自动耦合模型仍是工程近似；用于
临床/接触任务前，最终应以完整 9DoF URDF/CAD 的统一 CRBA/RNEA 模型替换。

用户提供的惯性辨识综述针对“传感器之后是单个刚体”的 10 参数在线辨识。当前腕部在
Flexiv 内置 F/T 传感器之后还有 q8/q9 两个活动关节，因此整套末端不是一个刚体，不能把
一次 RLS 得到的单个 6x6 空间惯量直接用于所有腕部角度。若做在线辨识，需要分别辨识两个
活动刚体的参数，并保留关节状态、摩擦和执行器惯量；当前 auto 模式不会在未经验证时边运动
边改写物理参数。

可在不移动任何轴的情况下单独检查 live TCP、6x6 空间惯量和 auto 选择结果：

```bash
../freedrive_python/.venv/bin/python scripts/check_9dof_auto_inertia.py
```

### 1.3 固定点姿态轨迹如何体现 9DoF 优势

两套程序使用完全相同的固定点姿态闭环：`p_des` 始终是启动瞬间采集的当前末端位置，
探针姿态同时做小幅 X、主幅 Y 和高频 Z 变化。Y/Z 方向主要激励 joint 8/9，因此 9DoF
会优先使用腕部；Flexiv 的七个关节补偿腕部旋转造成的末端平移，使真实末端点仍保持在
`p_fixed`。7DoF 则必须用整条主臂改变总成姿态。公平比较时，两次必须使用相同的
`duration/orientation` 和相同的起始 TCP。这里 `duration` 表示一个姿态周期，不是总运行
时间。

实时日志：

- 7DoF：`/tmp/flexiv_7dof_osc_rt.csv`
- 9DoF：`/tmp/flexiv_9dof_osc_rt.csv`

## 2. 实时权限（每台电脑只需一次）

先确认当前内核：

```bash
uname -a
```

输出应包含 `realtime` 或 `PREEMPT_RT`。加入实时和串口组：

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime,dialout src4
sudoedit /etc/security/limits.d/99-realtime.conf
```

在编辑器中加入：

```text
@realtime - rtprio 99
@realtime - memlock unlimited
@realtime - nice -20
```

完全注销并重新登录，然后确认：

```bash
groups
ulimit -r
ulimit -l
```

`groups` 应包含 `realtime dialout`，`ulimit -r` 必须为 `99`。Flexiv `Scheduler`
会在构造时创建优先级 99 的内部实时线程，因此 `95` 仍然会收到
`pthread sched param: Operation not permitted`。再执行：

```bash
chrt -f 99 true && echo "SCHED_FIFO 99 OK"
```

只有看到 `SCHED_FIFO 99 OK` 才能启动真实力矩 OSC。不要用 `sudo python ...` 绕过权限；
实时进程和 USB 设备应由当前登录用户运行。

## 3. 编译

官方 RDK v1.9 和依赖已安装在本机工作区。新窗口中执行：

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop
bash scripts/build_rt_osc.sh
```

生成：

```text
build_rt_osc/flexiv_7dof_torque_osc
build_rt_osc/flexiv_9dof_torque_osc
```

## 4. 先做不连接硬件的轨迹检查

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop

../freedrive_python/.venv/bin/python \
  scripts/run_7dof_torque_osc_demo.py \
  --trajectory orientation --duration-s 20 --orientation-deg 15

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_torque_osc_demo.py \
  --trajectory orientation --duration-s 20 --orientation-deg 15
```

这里只各生成一圈 `/tmp/flexiv_*_osc_loop.csv`，不会连接 Flexiv、腕部或发送力矩。相同
参数下两份 CSV 的几何目标完全相同。

## 5. 腕部惯量辨识与实时矩阵检查

“实时变化的惯量”不是每个周期重新估计质量，而是先辨识固定物理参数，再根据实时
`q8/q9` 每周期计算 `M_wrist(q8,q9)`。换探针、改变配重或安装结构后必须重新辨识。

### 5.0 拆下或更换探针后必须做的两项工作

这两项不能互相替代：

1. 在 Flexiv Elements 的 Tool 页面，为最终安装状态创建/选择独立 Tool，重新校准
   payload（mass、CoM、inertia）并保存为 active Tool。它供 Flexiv 内部非线性/重力补偿
   使用。TCP 可设为实际工作点；本 Demo 自己从 flange 计算 pivot，所以 TCP 不参与 pivot
   轨迹计算，但 payload 仍必须正确。
2. 再运行本节的 `calibrate_wrist_inertia.py --collect` 和 `--analyze`，辨识两台 moteus
   驱动的活动腕部。此脚本不会更新 Elements。

配置中的 `wrist_payload.assembly_id` 和辨识 CSV 的 `.meta.json` 会绑定硬件总成；拆探针前
的 CSV/JSON 不能再被 9DoF Demo 加载。Elements 标定完成后运行：

```bash
../freedrive_python/.venv/bin/python scripts/check_flexiv_tool.py
```

把完整输出更新到 `[flexiv_tool]`，并在确认后把 `calibration_ready=true`。当前配置已经写入
拆除探针后的 Elements 标定结果；旧腕部惯量辨识仍由 `assembly_id` 校验阻止加载。

### 5.1 采集

条件：Flexiv 保持静止并已连接；腕部周围无障碍；探针不可接触环境；急停可触及。采集
只移动 joint 8/9，Flexiv 仅被读取法兰姿态。每次执行 `--collect` 时，脚本会先调用
`set_current_wrist_zero.py`：对两台电机发送 STOP、读取当前位置、把当前姿态原子保存为新的
应用层 q8=q9=0，并自动生成旧 TOML 备份；它不会修改 moteus 固件编码器零位。随后脚本
重新加载配置并以这个零位为辨识中心。

辨识回归使用实际测得的 q/dq/torque，而不是假设位置目标被完美跟随。因此采集保留正常
20° 跟随异常阈值；若超过该值仍会 STOP，并输出 mode、fault、速度与实际力矩。

腕部 bridge 的每条位置/力矩命令都显式设置 moteus
`ignore_position_bounds=1`。这是为了解决“应用层重新保存零位后，控制器仍按旧 raw 编码器
零位附近的 `servopos.position_min/max` 夹紧新目标”的问题；它不改写 moteus 永久配置。
工程仍会按 `[wrist].joint_limit_deg` 检查实测关节角、限制命令目标，并保留跟随误差、通信
watchdog、fault 与 Ctrl-C STOP。若日志显示目标 raw 位置只变化几千分之一圈，而实测值却
持续跑向某个固定 raw 值，不能继续提高跟随误差阈值，应先确认本段修复已生效。

如果目标只有很小的正角度、实测角却快速向正方向增大，而且越过目标后实测力矩仍为正，
这不是惯量辨识问题，而是 moteus 输出反馈源、方向、减速比或位置 PID 配置问题。停止采集，
运行下面的只读诊断；它只发送 STOP，并将位置环相关参数保存到 `/tmp`。脚本逐项执行
`conf get`，不再使用可能因旧固件/串口流控而长时间阻塞的整份 `conf enumerate`：

```bash
../freedrive_python/.venv/bin/python scripts/diagnose_wrist_moteus_config.py
```

不要在看到诊断结果前修改或 `conf write` 电机配置。官方定义的
`motor_position.rotor_to_output_ratio` 是每一圈电机转动对应的最终输出转数：ID2 若为减速器
直连可按 `1/36` 核对；ID1 还有同步带，必须把带轮比乘进 `1/30`，不能仅按电机铭牌判错。
2024 固件不应依赖 `motor_position.output.sign=-1`；位置环还必须使用有效 output source、
正的 `kp` 和经过实际负载调试的阻尼 `kd`。

当前只读诊断确认 ID2 的永久 `kp/kd=1000/30`、ID1 为 `1200/20`，编码器源与方向正常，
但辨识预检中出现了明显位置过冲。程序因此不改写固件，而是在每条位置命令中临时使用
`position_kp_scale=[0.05,0.05]` 和 `position_kd_scale=[0.5,0.5]`：对应有效增益分别约为
ID2 `50/15`、ID1 `60/10`。这两个值可以在 UI 的 Wrist Motors 中保存；纯力矩 OSC 会显式
设置 `kp_scale=kd_scale=0`，不受这两个位置模式参数影响。

配置文件中的 `source 0 sign=+1`、`source 1 sign=-1` 本身不能证明物理方向正确或错误；
双编码器必须在实际转动时比较。下面的检查始终保持所选电机为 STOP，只读取编码器。准备
倒计时结束后，用手缓慢转动对应物理关节 10–30°，方向任选：

```bash
.venv_moteus/bin/python scripts/diagnose_wrist_encoder_direction.py --target 2
.venv_moteus/bin/python scripts/diagnose_wrist_encoder_direction.py --target 1
```

两个命令都必须得到 `PASS` 才能继续位置模式辨识；出现 `FAIL` 时不要通过删除跟随误差保护
来掩盖方向错误。

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop

../freedrive_python/.venv/bin/python \
  scripts/calibrate_wrist_inertia.py \
  --collect /tmp/wrist_inertia_samples.csv \
  --duration-s 40 \
  --amplitude-deg 6 10 \
  --confirm-move CALIBRATE_WRIST_INERTIA
```

若新 VS Code 终端尚未继承 `dialout`，完全注销重登；临时验证可在整条命令外使用
`sg dialout -c '...'`。

### 5.2 离线辨识

```bash
../freedrive_python/.venv/bin/python \
  scripts/calibrate_wrist_inertia.py \
  --analyze /tmp/wrist_inertia_samples.csv
```

结果写入 `config/wrist_inertia_calibration.json`，包含当前 `assembly_id`、刚体尺度、两轴反射惯量、粘性/库仑
摩擦、力矩 bias、拟合 RMS 和回归条件数。9DoF 实机脚本缺少该文件时会拒绝启动。

### 5.3 只读实时监视 `M(q)`

```bash
../freedrive_python/.venv/bin/python \
  scripts/calibrate_wrist_inertia.py --monitor --duration-s 15
```

此命令不移动 Flexiv，不命令腕部运动，只读取 q8/q9 并打印实时 `2x2 M(q)` 和特征值。

## 6. 实机前检查

```bash
../freedrive_python/.venv/bin/python scripts/run_7dof_torque_osc_demo.py --preflight
../freedrive_python/.venv/bin/python scripts/run_9dof_torque_osc_demo.py --preflight
```

确认：

1. Flexiv 已使能、Operational、无 fault；
2. active Tool 的质量、CoM、TCP 与当前腕部/探针一致；
3. 探针绕当前 TCP 改变姿态时的完整扫掠体积无障碍，人员不在工作空间；
4. 腕部没有另一个程序占用 fdcanusb；
5. 先用 3°、20 s 做低幅固定点姿态测试；
6. 操作员始终握住急停，出现噪声、抖动或方向错误立即 Ctrl-C/急停。

## 7. 分开运行两个实机 Demo

当前默认 `--trajectory orientation`：程序在切换实时力矩模式前采集真实末端/TCP 的
当前位置，以它作为该次运行的 `p_fixed`。整个周期内 X/Y/Z 目标位移、线速度和线加速度
严格为零，只改变姿态。9DoF 使用实时 q8/q9 更新真实 tip 位置和完整 9 列雅可比，并优先
把两轴可实现的姿态分配给腕部；Flexiv 会补偿腕部运动造成的末端位移。

### 7.1 7DoF Flexiv 实时力矩 OSC

首次用低幅值：

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop

../freedrive_python/.venv/bin/python \
  scripts/run_7dof_torque_osc_demo.py \
  --trajectory orientation --duration-s 20 --orientation-deg 3 \
  --real --confirm RUN_7DOF_TORQUE_OSC
```

### 7.2 9DoF Flexiv + wrist 多速率实时力矩 OSC

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_torque_osc_demo.py \
  --trajectory orientation --duration-s 20 --orientation-deg 3 \
  --inertia-mode auto \
  --real --confirm RUN_9DOF_TORQUE_OSC
```

确认低幅运动方向和固定点控制正确后，可把 `--orientation-deg` 增至 15。该参数是主要
姿态分量幅值；X 为其 0.1 倍，Y 为 1.0 倍，Z 为 0.85 倍，并采用不同谐波形成闭合姿态环。

下面两条命令使用相同固定点姿态参数做 7DoF/9DoF 对比：

```bash
../freedrive_python/.venv/bin/python \
  scripts/run_7dof_torque_osc_demo.py \
  --trajectory orientation --duration-s 20 --orientation-deg 15 \
  --real --confirm RUN_7DOF_TORQUE_OSC

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_torque_osc_demo.py \
  --trajectory orientation --duration-s 20 --orientation-deg 15 \
  --inertia-mode auto \
  --real --confirm RUN_9DOF_TORQUE_OSC
```

两者都会每 20 秒重复同一个固定点姿态环，直到 Ctrl-C。要改变固定点，先 Ctrl-C 停止，
把机械臂移动到新位置，再次启动；新位置会自动成为新的 `p_fixed`。9DoF 终端每秒额外输出
`WRIST_OSC q_deg/target_q_deg/dq_deg_s/tau_cmd_Nm`，可直接确认腕部是否收到目标并运动。
9DoF 将轨迹中腕部可实现的姿态增量按 `allocation.wrist_priority_gain` 放大，使用
`wrist_tracking_kp_nm_per_rad`/`wrist_tracking_kd_nm_s_per_rad` 直接克服已辨识的减速器静摩擦；
Flexiv 的主 6-D OSC 同时补偿这部分额外腕部运动，所以两种 demo 的探针任务轨迹仍相同。CSV 还记录
`q8/q9` 与 `target_q8/target_q9`。两个程序都保留关节额定力矩饱和、力矩变化率限制和
Flexiv 软限位；这些是实时力矩控制的执行器/连续性保护，不是接触力达到某值就退出的旧
逻辑。Ctrl-C 会发送 Flexiv Stop 和腕部零力矩/STOP。

旧轨迹仍可显式选择：`--trajectory rectangle` 为 XY 圆角矩形切向姿态轨迹，
`--trajectory loop` 为三维位置/姿态闭环。默认固定点姿态模式不会读取矩形尺寸或
`--radius-m`。

如果 9DoF 再次停止，先看错误类型：

- `RT_WARNING ... transiently stale`：短暂丢帧，系统已进入零腕部力矩/主臂保持；若随后
  出现 `RT_INFO ... recovered` 会自动恢复。
- `continuously stale beyond hard timeout`：fdcanusb/USB 连续 2 秒没有状态；检查是否被
  其他程序占用或 USB/CAN 供电与线缆，不要取消硬断线保护。
- `WRIST_OSC` 中目标角明显变化、实测角不变：查看 moteus fault/mode 和力矩值，而不是
  修改轨迹。
- 目标角本身接近零：核对腕部零位、两根轴在 `[wrist_geometry]` 中的方向，以及 active
  Tool/TCP 是否仍对应当前探针。

## 8. 不再使用 Bota 后的探针 Z 力

普通遥操作程序直接读取 Flexiv 补偿后的 world-frame endpoint force。实时腕部角度给出：

```text
R_world_probe = R_world_flange * R_flange_wrist(q8,q9)
F_probe = R_world_probe^T * F_world
F_probe_z = [0,0,1]^T F_probe
```

力向量与参考点平移无关，所以求探针 Z 方向的力只需实时姿态；若要换算探针端力矩，则还
必须加入从 Flexiv 传感器到探针 TCP 的力臂叉乘项。本工程已删除 Bota 运行桥和配置，模式
3 直接使用上述 Flexiv 力投影。

## 9. 触觉反馈稳定层

`run_9dof_teleop.py` 的反馈路径现在依次执行：Flexiv wrench bias → 坐标映射 → 低通 →
渐入 → 本地速度阻尼 → 力变化率限制 → 时间域无源性观察器/能量罐 → omega.7 额定力饱和。
它的目的不是把力隐藏，而是阻止延迟闭环净注入能量。当前保守参数在
`config/nine_dof_teleop.toml` 的 `[force_feedback]`，UI 也会自动显示全部字段。

真实稳定性仍需在本机硬件上由低增益、低接触力逐级验证。若仍抖动，先记录日志中的
`passivity_limited`、`tank_energy_j`、手柄速度和 Flexiv wrench；不要直接提高增益或关闭
无源性控制。
