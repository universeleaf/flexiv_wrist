# 控制架构与技术细节 / Control Architecture and Technical Details

## 中文

### 1. 分层和解耦

```text
run.py / UI
  ├─ scripts/                 真实任务编排
  ├─ controllers/             无硬件 I/O 的控制、运动学、动力学
  │    └─ cpp/                1 kHz Flexiv torque OSC
  ├─ hardware/                CHAI3D 与 moteus 进程桥
  ├─ tools/                   零位、Home、辨识、构建、检查
  └─ testing/                 unit 与 hardware tests
```

Python 设备桥只负责时间戳、状态、命令和 watchdog；Python 数学控制模块不打开
USB 或机器人。1 kHz C++ RT 控制进程是唯一打开 Flexiv 并发送关节力矩的进程。
任务层负责模式切换、clutch anchor、进程生命周期和 Stop。这样控制问题可以在
`controllers/` 内单独修改和单元测试。

### 2. 共同、连续且适合 7DoF 的末端轨迹

`demo7` 与 `demo9` 执行完全相同的 `loop` 任务，并持续重复直到 `Ctrl+C`。
默认周期 20 s、轨迹参数半径 10 mm；由于 X 使用 `cosφ-1`，相对启动点的最大
X 位移是 20 mm。相对启动点的位置为：

```text
Δp(φ) = [r(cosφ-1), 0.65r sinφ, 0.35r sin(2φ)]
```

姿态由三个连续谐波组成，`A=35°`：

```text
R(φ) = R0 Rx(0.10A sinφ) Ry(A sin2φ) Rz(0.85A sin3φ)
```

因此姿态一直变化，q8-like 分量约为 ±35°、q9-like 分量约为 ±29.75°，且每圈
精确回到启动位姿。目标位置、速度、加速度、角速度和
角加速度均解析计算；启动使用五次 phase-rate ramp，避免速度阶跃。9DoF 不放大
任务：q8/q9 以 1:1 自然分配承担它们能实现的全部两个姿态分量，Flexiv 只完成
剩余任务；7DoF 用同一目标但只能由七关节完成。

旧版固定点 `spin` 会要求 7DoF 本体跨越完整 360° 姿态分支。实机日志记录到
76 mm 位置误差、约 90° 姿态失跟和 24.95 N·m 合力矩，并出现快速接近自身的
构型，因此它不再是默认对比任务。`spin` 只保留为非默认离线/工程实验选项；
不能把“末端数学轨迹闭合”理解为“从任意初始关节姿态都无自碰”。

### 3. 7DoF torque + OSC

状态为 `q∈R⁷`，任务为 `x∈SE(3)`：

```text
Λ = (J M⁻¹ Jᵀ + λ²I)⁻¹
J# = M⁻¹JᵀΛ
N = I - J#J
a* = ẍd + Kp e + Kd (ẋd-ẋ)
τ = JᵀΛa* + Nᵀ(Kn(qref-q)-Dn q̇) + c(q,q̇)
```

Flexiv C++ RDK 在 `RT_JOINT_TORQUE` 下以 1 kHz 执行。重力由 RDK 流式扭矩
接口的 gravity compensation 参数处理，`model.c()` 提供 Coriolis/centripetal。
q8/q9 在此 demo 中由 moteus 位置模式保持零位，不参与任务 Jacobian。

7DoF 控制核心已从公开仓库 commit `2f1e763` 精确恢复：平移增益 `100/20`、
旋转增益 `70/16`、零空间增益 `8/2`、正则化 `0.03`，输出为
`τ=τOSC+c(q,q̇)`。没有后来试验加入的额外全关节阻尼、反馈幅值 ramp 或
“持续失跟即退出”逻辑。目标生成器仍是当前共同的 20 s / 10 mm / 35° 连续
`loop`，因此只回退控制器，没有回退用户认可的运动，也没有修改 9DoF。

仍保留与已知工作版本一致的每关节力矩额定值、每毫秒 torque slew、Flexiv
gravity compensation、官方 soft joint limits、robot fault 和 operational 检查。
`/tmp/flexiv_7dof_osc_rt.csv` 记录位置误差、姿态误差和力矩范数。该控制器不会因
普通任务误差自行退出，但它不是几何碰撞规划器；启动构型、急停和工作空间检查
仍由操作者与 Flexiv 安全系统负责。

### 4. 9DoF coupled OSC

组合状态为 `q=[q_arm,q8,q9]∈R⁹`。任务 Jacobian 为 `J∈R⁶ˣ⁹`，腕部两列同时
包含末端平移力臂和旋转轴。空间刚体惯量采用 twist 顺序
`[vx,vy,vz,wx,wy,wz]`，单刚体是 6×6；组合关节质量矩阵必须是 9×9：

```text
M9(q) = Σ Jbodyᵀ Ibody(6×6) Jbody + reflected actuator inertia
Λ6(q) = (J9 M9⁻¹ J9ᵀ + λ²I)⁻¹
```

Flexiv `Model::M()` 已包含 Elements 当前 Tool 的 q8=q9=0 固定基线。代码保留
该基线，减去零位移动腕部贡献，再加入实时 q8/q9 的两个刚体 6×6 空间惯量和
arm-wrist 耦合块，得到每周期变化的对称正定 9×9 矩阵。`λ` 根据任务惯量条件数
自动调整。

腕部自然分配策略：真实目标探针姿态直接映射一次到腕部可达的两个旋转方向，
不做倍数放大；不可达 roll 和剩余误差由 Flexiv 完成。腕部目标不会再同时进入“OSC nullspace”和“直接 joint
tracker”两条刚度通道；只保留一个显式 distal tracker，消除了旧版双重刚度。

Flexiv 七轴始终执行 1 kHz `RT_JOINT_TORQUE`。外部腕部默认执行
`hybrid_position_ff`：C++ 耦合 OSC 仍计算 q8/q9 目标和动力学前馈力矩，moteus
用内部位置 PD 闭合快速两轴跟踪环，并叠加该前馈项。这样绕过当前驱动上已实测的
“纯前馈请求约 1.8 N·m、实际仅约 0.02--0.04 N·m”问题。完成独立纯力矩验证后，
可把 `[wrist].osc_execution_mode` 改为 `pure_torque` 作对照。

为修复突然运动和抖动，原始腕部 IK 目标经过临界阻尼二阶滤波，并配置独立的
最大速度和加速度。进入扭矩模式用 2 s 五次 ramp；短时 USB stale 期间腕部输出
0 Nm、Flexiv 保持当前 q，恢复时重置目标滤波器并用 1 s ramp。扭矩每毫秒做
slew shaping，最终才按执行器连续额定值饱和。

注意：Flexiv 七关节环是硬实时 1 kHz；moteus 通过 Python + USB/CAN-FD 以
100 Hz 交换状态/命令，因此整套九执行器并非一条统一的硬实时总线。共享内存用
奇偶 sequence 防止 torn frame，并使用设备接收时间判断 stale。

### 5. 腕部动力学辨识

辨识默认采用 joint 8 ±45°、joint 9 ±60°、120 秒的多个低频平滑正弦激励，记录 q、dq、估计 ddq、输出扭矩和实时
flange 姿态。最小二乘拟合 rigid-body scale、反射惯量、粘性/库仑摩擦和 bias，
并记录 RMS 与条件数。9DoF `auto` 模式对拟合质量做 gate；不合格数据不会直接
驱动逆动力学。当前实机配置要求 `require_identified_inertia=true`，因此不合格时
直接拒绝启动，而不是回退到 Elements baseline + nominal wrist 参数。

`dynamic-inertia` 每个样本重新计算 `M_wrist(q)` 2×2 和腕部相对 flange 的
`I_spatial(q)` 6×6。6×6 空间惯量不是九关节质量矩阵，两者用途不同。

当前不把九个关节同时运动称为“一次九轴辨识”。Flexiv 七轴参数由 Elements/RDK
提供，辨识程序只求外置 Joint 8/9 的未知参数；控制器在运行时用实时姿态将两者
组装为耦合的 9×9 `M(q)`。这是完整九自由度动力学控制模型。若要真正联合回归九轴
物理参数，必须让 Flexiv 和 moteus 共用严格同步的时间戳、位置、速度、加速度和
关节力矩采样。目前两者分别为 1 kHz 硬实时和约 50--100 Hz Python/USB/CAN-FD，
直接回归会把总线延迟误认为惯量，因此没有加入这种不可靠的“九轴一起标定”。

#### 动态重力补偿

Flexiv RDK 的流式力矩接口继续以 `enable_gravity_comp=true` 运行，因此 Flexiv
根据 Elements 中的 Active Tool 补偿 `q8=q9=0` 时的七轴基础重力。控制器不会再
把移动腕部当成永远固定的 Tool。每个 1 kHz 周期都使用两个腕部刚体的实时 CoM
Jacobian 计算完整九自由度重力补偿：

```text
g_wrist,9(q8,q9) = -Σ J_COM,i(q8,q9)^T m_i g_world
Δg_arm = g_wrist,9[0:7](q8,q9) - g_wrist,9[0:7](0,0)
g_external_wrist = g_wrist,9[7:9](q8,q9)
```

`Δg_arm` 加到发送给 Flexiv 的七轴力矩，完整的 `g_external_wrist` 发给两个
moteus。减去零位项是必要的：Elements 已经补偿了零位 Tool，直接再加完整七轴
腕部重力会双重补偿。当前实际关系为：

```text
τ_Flexiv_stream = τ_OSC + c_arm + Δg_arm
Flexiv servo adds g_Elements internally
τ_joint8,9 = wrist inner-loop command + g_external_wrist
```

这个补偿对当前已辨识的两刚体质量、CoM、实时法兰姿态和 q8/q9 模型是完整的；
其物理准确度仍取决于 Elements Tool 标定和腕部辨识参数。实时 CSV 记录
`arm_wrist_gravity_delta_norm_nm`、`wrist_gravity8_nm` 和
`wrist_gravity9_nm`，便于在不同姿态下验证。

### 6. 遥操作模式

当前默认入口是单进程、单机器人会话的完整三模式工作流：

- `python3 run.py teleop` / `teleop-impedance`：Mode 1/2 使用
  `NRT_CARTESIAN_MOTION_FORCE`、`SetCartesianImpedance`、
  `SetNullSpacePosture` 和 `SetNullSpaceObjectives`。Flexiv 内部负责七轴动力学、
  重力补偿与笛卡尔弹簧/阻尼。
- 同一入口的 Mode 3 切换为柔和混合操作空间任务：固定接触点、只接受姿态输入，
  腕部采用 OSC 目标/动力学前馈加 moteus 位置内环，Flexiv 采用阻抗位置轴和
  Tool-Z 主动力轴。两个控制器不会同时占用 Flexiv。
- `python3 run.py teleop-osc`：保留下面描述的实验性 C++ 1 kHz
  `RT_JOINT_TORQUE` 全模式 OSC，供研究对照。

阻抗 Mode 2 仍然是腕部优先，但不做角度放大：先求 q8/q9 的自然 1:1 目标，再用
未整形的 IK 几何目标决定腕部可达分量，避免七轴在腕部平滑追赶期间先代偿再退回。
实际探针 TCP 尖端仍是严格位置目标。若姿态完全可由 Joint 8/9 实现，Flexiv 法兰
姿态不变，仅做抵消偏心 TCP 圆弧所必需的小量平移；手柄平移仍移动实际 TCP。
实现上，法兰姿态使用未整形腕部 IK 目标，法兰平移使用实测 q8/q9：腕部变慢只会
造成姿态暂时落后，不会让七轴代替它旋转；TCP 位置仍由实测几何保持。腕部命令的
15° 移动跟随窗口防止堵转时误差无限积累，但不限制总行程。Mode 1 另有独立的 XZ
平面旋转符号，当前相对 Mode 2 已反转。

moteus mode 11 is a command-watchdog timeout, whereas normal position control
is mode 10. The upstream coordinator-to-bridge watchdog remains 100 ms for fast
STOP behavior, but the drive-side watchdog is 1000 ms so one sequential
ID2+ID1 fdcanusb transaction cannot expire the servo. The bridge keeps
re-sending position commands to recover a transient mode 11 and reports
`WRIST_POSITION_MODE_RECOVERY`.
每次 clutch 接合都把当前七轴姿态设为 Flexiv 零空间参考。clutch 松开后使用
4%/8% 的平移/旋转标准刚度和 0.10 零空间参考权重，形成容易手推但撤力会回到
释放点的柔顺保持。

clutch 按下时捕获手柄和机器人相对 anchor；目标是相对位姿而非绝对设备坐标。
模式切换时必须先 release 再 press，防止跨模式继承旧 anchor。

默认 Mode 3 会先要求同方向至少 2 N 的真实接触，再以 3 秒斜坡从实测接触力增加到
目标力；Z 力轴最大速度为 0.005 m/s。接触低于 1 N 持续 50 ms 后只关闭力轴并
回到固定点位置保持，不退出会话。

以下段落描述 `teleop-osc` 备用后端：三个模式都由同一个 C++
`RT_JOINT_TORQUE` 控制器在 Flexiv 侧以 1 kHz 执行。Python 只读取
omega.7/脚踏板、发布目标并服务 USB/CAN-FD 腕部桥；共享内存使用奇偶序号保证
目标/状态帧一致。

Python 可以从连续目标位姿估算世界坐标系目标 twist，但真实遥操作默认把
`target_velocity_feedforward_gain` 设为 0。C++ 的 1 kHz 二阶目标整形器自行产生
连续速度，避免把 50 Hz USB 掉帧误认为很大的目标速度。

目标整形只限制每秒变化速度和加速度，不限制累计行程，也不再把目标硬夹在实测
TCP 周围的 15 mm/8° 窗口内。旧硬窗口在某些负载姿态下只能产生约 1.2 N 的位置
恢复力，会表现为运行数秒后卡住。

Python 在 100 Hz 保留按 Flexiv 配置速度换算的单周期输入边界，C++ 在 1 kHz 做
连续速度/加速度整形。平移和旋转均为 1:1。旋转使用 clutch 捕获手柄自身坐标的
`R0ᵀR`，并在探针 anchor 上后乘同一个局部增量；这避免非零起始姿态造成轴混合。
松开、回中、重新按 clutch 会以当前实测 TCP 重新捕获 anchor，
因此可以连续累计移动而不会继承旧目标误差。

- Mode 1 / 7DoF OSC：只取探针 Jacobian 的前七列 `J7∈R6×7`，计算 6D
  位姿 OSC。q8/q9 保持在进入本模式时的角度，不参与任务解；手柄平移和旋转都
  控制探针目标。
- Mode 2 / 9DoF OSC：使用腕部优先的分层分配和实时耦合 `M9(q)`。腕部 IK
  先做 1:1 自然分配；随后 Flexiv 的 6×7 OSC 只完成平移、腕部运动造成的 TCP
  偏移，以及 q8/q9 无法表达的法兰姿态残差。Flexiv 内部再按“位置主任务、残余
  姿态次任务”分层，残余姿态投影到位置零空间；腕部目标与 Flexiv 残差目标分别
  整形。Mode 2 冻结 clutch 捕获的冗余七轴参考，避免无助于末端任务的整臂漂移。
  默认 Flexiv 残余角速度/角加速度只有 0.20 rad/s 和 0.60 rad/s²，而腕部保持
  真实 1:1 IK，不使用角度倍增。
- Mode 3 / 固定点姿态+恒力 OSC：clutch 按下时捕获探针 TCP 的完整 XYZ
  接触点。在 Python 和 C++ 两层都丢弃手柄平移；q8/q9 优先完成其可达姿态，
  Flexiv 使用严格分层 OSC：先解三维 XYZ 主任务，再把较低增益的法兰姿态残差
  投影到位置任务的动力学零空间。它不是简单的 6D 加权，因此本体姿态不能牺牲
  TCP 位置来降低自身误差。独立 Tool-Z
  力任务仅在检测到正确方向、至少 2 N 的已有接触后锁存。固定点任务优先于力
  任务；位置误差从 1 mm 增至 3 mm 时力项平滑降为零，使位置 OSC 先回到捕获点：

```text
ef = Fz,target - Fz,sensed
edb = deadband(ef, 1 N)
sp = smooth_fade(position_error, full=1 mm, off=3 mm)
fcmd = sp [-Fz,target - Kp edb - Ki∫edb dt - D zᵀv]
τforce = Jfᵀ fcmd
τtotal = τmotion + τforce + c(q,qdot)
```

门控未锁存时 `fcmd=0`，所以探针不会在空气中自动搜索表面。力矩作为接触前馈
叠加在固定点阻抗上。-15 N 目标的 1 N 死区表示 -14 至 -16 N 均为达标，积分项
在此区间缓慢消退，不追逐传感器噪声。接触低于 1 N 持续 50 ms 后只关闭力项，
XYZ 固定点 OSC 和整个遥操作进程继续运行；重新建立至少 2 N 接触即可再次锁存。
当位置误差进入 1–3 mm 恢复区时，q8/q9 的新姿态推进和 Flexiv 残余姿态目标也
一起平滑暂停，避免偏置 TCP 在本体尚未补偿时继续被腕部带离表面。TCP 回到 1 mm
内后姿态跟随自动平滑恢复。本体残余姿态另有 0.20 rad/s、0.60 rad/s² 的独立目标
整形；这不放大或缩小最终姿态，只防止七轴突然绕固定点重构。
在刚性接触面上，环境反力与位置保持共同
形成平衡；物理系统不可能在同一个自由方向同时保证“数学上绝对零位移”和非零
接触力，因此会保留极小弹性位移。默认任务力读取当前配置
`pivot_orientation_osc.target_sensed_force_tool_z_n`（当前为 -15 N）；UI 可保存。

Flexiv 的 6×7 Jacobian 还会把角速度行乘以 0.25 m 特征长度，再计算最小奇异值。
低于 0.05 时，Flexiv 目标速度用平滑曲线逐渐降低；到 0.015 时保留 15% 命令权，
不会退出程序。腕部目标不受这个臂速缩放影响，因此系统会自然优先使用 q8/q9。
按下、松开 clutch 或切换模式时，七轴冗余姿态参考与任务 anchor 一起锁存当前实测
关节角。这保证手柄增量为零时，零空间不会把机械臂拉向启动时或上一个模式的旧姿态。
接合后参考在健康区以约 10 s 时间常数慢速跟随；进入奇异区时冻结本次接合中最后
一个健康参考。奇异值速度缩放仍然有效，但不会再用旧零空间目标制造自主运动。
达到目标力或触觉输出饱和不会让任务退出。`mode3_force_command_limit_n` 与各轴
连续额定力矩仍是物理输出边界，不是工作空间/会话限制。

Flexiv 七轴是 1 kHz 硬实时力矩控制。外部 q8/q9 因 USB/CAN-FD 总线限制采用
多速率执行：C++ 计算 9DoF 目标及动力学/力任务前馈，moteus 内部位置环稳定、
快速地跟踪 q8/q9 目标。这不是把腕部从 OSC 中移除；它是 OSC 的执行器内环。
任何自定义接触力控制在接触人体前都必须先在非人体、可退让的测试夹具上验证
符号、稳定性和停止行为。

松开 clutch 是独立的制动保持状态，而不是继续执行 Mode 1/2/3 的末端重分配。
松开沿捕获当时实测的七轴角度、q8/q9 和探针位姿，立即清零目标 twist，并执行
实时质量矩阵把“回位加速度 + 过阻尼刹车”换算成每个关节所需力矩。默认自然
频率 1 Hz、阻尼比 1.25；它比旧固定增益更硬，并避免回位越过目标后反弹。过去
松开时会错误地重新启动 1 秒力矩 ramp，使刚松开后的制动力矩接近零；现在 release
和重新按下都直接使用刚捕获的完整支撑刚度，最终力矩变化率边界继续保证输出连续。
启动进程和通信 stale 恢复仍保留各自的平滑 ramp。这样不会在按下后留下一个
“关节保持已经撤掉、笛卡尔支撑尚未建立”的重力残差下坠窗口。

### 7. 力反馈稳定性

Flexiv 外力经 bias、坐标变换、比例和 deadband 后进入 `StableHapticFeedback`。
它包含低通、力 slew、接入 ramp、本地耗散阻尼和 time-domain passivity observer/
energy tank。无源条件用端口功率 `P=Fᵀv` 更新能量；当待输出力会注入超过能量罐
的能量时，只移除速度方向上的主动分量。omega.7 额定 12 N 是饱和，不是退出条件。

手柄位姿输入另有连续 radial deadband 和由 Flexiv 配置速度导出的单周期最大增量，
用来阻止延迟力反馈造成的小幅闭环自激，同时不限制整个工作空间。

### 8. PID 和故障边界

moteus 永久 PID 是执行器输出位置环；`kp_scale/kd_scale` 只对单次应用位置命令
临时缩放。Torque OSC 设置位置增益为零并直接发送输出轴 Nm。不要把两者的增益
混为一谈。

保留的边界：q8 ±90°、q9 ±360°、驱动 fault、机器人 fault、非 operational、
fdcanusb 断开、连续 stale、执行器连续额定扭矩和扭矩 slew。删除这些检查会在
反馈未知时继续输出，不能视为“透明控制”。正常达到接触力或 omega.7 饱和不会退出。

## English

### 1. Separation

Mission orchestration lives in `scripts/`, reusable math in `controllers/`,
device I/O in `hardware/`, setup/calibration in `tools/`, and validation in
`testing/`. Python controller math does not open hardware. The 1 kHz C++ RT
controller process is the sole owner of the Flexiv connection and joint-torque
stream; the wrist USB bridge remains a separate lower-rate process.

### 2. Same continuous, 7DoF-feasible task

Both demos execute the exact same analytic `loop` indefinitely until Ctrl-C.
The default period is 20 s and the trajectory parameter radius is 10 mm. Since
X uses `cos(phi)-1`, its maximum displacement from the captured start is 20 mm:

```text
dp(phi) = [r(cos(phi)-1), 0.65r sin(phi), 0.35r sin(2phi)]
R(phi) = R0 Rx(0.10A sin(phi)) Ry(A sin(2phi)) Rz(0.85A sin(3phi)), A=35 deg
```

Position, velocity, acceleration, angular velocity, and angular acceleration
are analytic and continuous. A quintic phase-rate ramp removes the startup
velocity step. The wrist-like components are approximately ±35 deg on q8 and
±29.75 deg on q9. The 9DoF allocator does not amplify this task: q8/q9 take every
reachable component once, while the arm supplies only the residual.

The previous fixed-point 360-degree spin was not a reasonable common 7DoF
mission. Its real log reached 76 mm position error, about 90 degrees of attitude
loss, and 24.95 Nm aggregate torque while the arm moved toward a potentially
self-colliding configuration. `spin` therefore remains only a non-default
engineering experiment; it is not used by `python run.py demo7/demo9`.

### 3. 7DoF OSC

The 1 kHz C++ controller computes dynamically consistent operational inertia,
task torque, and null-space posture torque:

```text
Lambda = (J M^-1 J^T + lambda^2 I)^-1
tau = J^T Lambda a_task + N^T tau_posture + coriolis
```

The two wrist axes are held at zero and do not appear in the 7DoF Jacobian.
The 7DoF control core is restored exactly from published commit `2f1e763`:
translation gains 100/20, rotation gains 70/16, null-space gains 8/2,
regularization 0.03, and `tau=OSC+coriolis`. Later experimental direct joint
damping, feedback-amplitude ramping, and task-error termination are absent.
Only the controller was restored; the accepted common 20 s / 10 mm / 35 deg
continuous loop and the 9DoF implementation are unchanged.

Per-joint torque ratings, 1 ms torque slew, Flexiv gravity compensation, vendor
soft joint limits, robot faults, and operational-state checks remain. The RT
CSV records position error, orientation error, and torque norm. This controller
does not stop merely because task error crosses a custom threshold, but it is
not a geometric collision planner; physical preflight remains mandatory.

### 4. 9DoF OSC and dynamic inertia

The combined Jacobian is 6×9. Each moving body has a 6×6 spatial inertia; the
assembled joint mass matrix is 9×9 and the operational inertia is 6×6. The
runtime preserves the Elements Tool baseline and replaces its zero-wrist
moving contribution with live q8/q9 rigid-body terms, including arm-wrist
coupling.

The wrist target is handled by one distal tracking channel rather than being
duplicated in both null-space and direct tracking. A critically damped target
shaper, velocity/acceleration limits, startup torque ramp, stale-recovery ramp,
and torque slew shaping address the previous oscillation and sudden motion.

The seven Flexiv joints remain in 1 kHz RT joint-torque mode. By default the
external wrist uses `hybrid_position_ff`: the coupled OSC publishes both its
q8/q9 target and model feed-forward torque, while moteus closes the fast inner
position loop. This works around the measured feed-forward-only behaviour of
the commissioned 2024 firmware (about 1.8 Nm requested but only 0.02--0.04 Nm
measured). `pure_torque` remains available for drive-level comparison.

The Flexiv loop is hard realtime at 1 kHz. The external moteus wrist exchanges
state and torque through Python and USB/CAN-FD at 100 Hz, so the entire
nine-actuator chain is not one hard-realtime bus. Sequence-protected shared
memory and receive timestamps detect stale frames.

Gravity compensation is configuration dependent as well. Flexiv keeps its
internal Elements gravity compensation enabled for the calibrated wrist-zero
Tool. At 1 kHz, the controller evaluates the two moving wrist-body CoM
Jacobians and computes the complete nine-coordinate wrist gravity vector. It
adds only the live-minus-zero first seven components to Flexiv and sends the
full last two components to moteus:

```text
tau_flexiv_stream = tau_OSC + coriolis_arm + delta_g_wrist_to_arm
tau_wrist = inner_loop_command + g_wrist_joints
```

Subtracting the zero-pose arm contribution prevents double compensation of the
payload already present in Elements. The RT CSV logs the arm correction norm
and both wrist gravity torques for validation.

### 5. Teleoperation and force

The default entry is now one process and one robot session for all three modes.
`python3 run.py teleop` and `teleop-impedance` use Flexiv's own
`NRT_CARTESIAN_MOTION_FORCE`, `SetCartesianImpedance`, and a clutch-captured
null-space posture for Modes 1 and 2. Flexiv therefore owns the seven-axis
dynamics and gravity compensation. In Mode 2, q8/q9 first receive the natural
1:1 reachable orientation, and the arm target is computed from that wrist
geometric IK target—not from the rate-limited or measured wrist lag. Pure
orientation keeps the actual probe TCP fixed. If joint 8/9 can produce the
entire requested rotation, the Flexiv flange orientation remains unchanged and
the arm only translates enough to cancel the offset TCP arc. This removes the
observed temporary arm rotation followed by a return as joint 8 catches up.
Haptic translation still moves the actual TCP target. On clutch release,
stiffness drops to 4% of nominal for
translation and 8% for rotation, with 0.10 null-space posture tracking, while
the captured release pose remains the equilibrium.

Mode 3 runs in that same session as a gentle hybrid operational-space task.
Haptic translation is discarded and the captured probe contact point remains
the target. q8/q9 take their reachable orientation through an OSC target/model
feed-forward outer loop and the stable moteus position inner loop. Flexiv uses
Cartesian impedance for the position/orientation axes and activates only the
probe Tool-Z force axis after a same-sign 2 N contact is measured. The command
ramps from measured contact to its target over 3 seconds, with force-axis speed
limited to 0.005 m/s. A 50 ms contact loss disables only the force axis and
returns to position hold.

`python3 run.py teleop-osc` retains the experimental all-mode custom 1 kHz
joint-torque OSC; it is no longer required to access Mode 3.

The remainder of this section describes the optional torque-OSC backend. In
that backend all three modes run in the same C++ 1 kHz Flexiv `RT_JOINT_TORQUE` loop;
Python only publishes coherent targets and services haptic, pedal, and wrist
I/O.

Python can estimate target twist from consecutive mapped poses, but real
teleoperation defaults its feed-forward gain to zero. The 1 kHz C++ target
shaper creates a coherent velocity without amplifying a delayed 50 Hz USB
sample into a large velocity command.

The shaper bounds target rate and acceleration, not accumulated travel. The
old hard 15 mm / 8 degree tracking window has been removed because it could
cap the available restoring wrench and make a loaded pose appear stuck.

Python keeps a 100 Hz per-cycle input bound derived from the configured Flexiv
velocity, while C++ performs continuous 1 kHz velocity/acceleration shaping.
Translation and orientation are both natural 1:1. Orientation is measured in
the clutch-captured handle frame as `R0.T * R` and post-multiplied onto the
probe anchor, preventing axis mixing at a non-identity engagement attitude.
Releasing, recentering, and pressing
the clutch captures the measured TCP as a new anchor, allowing accumulated
travel without inheriting a stale target.

- Mode 1 uses the arm-only 6x7 probe Jacobian for six-dimensional pose OSC;
  q8/q9 hold their engagement angles.
- Mode 2 uses a wrist-first hierarchy with the live coupled 9x9 mass matrix.
  Wrist IK takes each reachable orientation component once, without artificial
  amplification. A 6x7 arm OSC then handles translation, TCP displacement due
  to wrist motion, and only the remaining flange-orientation residual. Inside
  the arm controller, Cartesian position is the primary task and residual
  orientation is projected into its dynamically consistent null space. The
  redundant arm posture captured at clutch engagement is frozen in Mode 2.
  Wrist and arm-residual targets have independent shapers; the arm residual is
  limited to 0.20 rad/s and 0.60 rad/s2 while wrist IK remains natural 1:1.
- Mode 3 captures the complete probe TCP contact point on clutch engagement
  and discards haptic translation on both sides of the process boundary. q8/q9
  take their reachable orientation first. Flexiv now uses strict task hierarchy:
  3D XYZ is solved first, then residual flange orientation is projected into
  the dynamically consistent null space of that position task. This is not a
  weighted 6D compromise, so arm orientation cannot intentionally trade TCP
  position for smaller angular error. The Tool-Z force loop is
  zero in free space and latches only after a same-sign contact of at least
  2 N is measured, so it cannot autonomously search for a surface. The -15 N
  target has a 1 N deadband, so readings from -14 to -16 N are accepted without
  PI correction. If XYZ error grows from 1 to 3 mm, the force term fades smoothly
  to zero and fixed-point OSC recovers the captured point first. Contact below
  1 N for 50 ms disables only the force term, not the process; 2 N contact can
  latch it again.
  When position error enters the 1--3 mm recovery band, both new q8/q9
  orientation progress and residual arm-orientation progress pause smoothly.
  They resume after the TCP returns within 1 mm. Residual Flexiv orientation
  shaping is independently limited to 0.20 rad/s and 0.60 rad/s^2; this changes
  transition speed, not the final requested orientation. A rigid contact reaches
  force equilibrium with a very small elastic pose deflection; exact zero
  displacement and nonzero force cannot both be independently imposed on a
  free axis.

The arm Jacobian is unit-normalized using a 0.25 m characteristic length and
monitored through its minimum singular value. Below 0.05, only Flexiv target
speed is reduced smoothly; at 0.015 it retains 15% authority and the process
does not abort. The wrist remains at natural 1:1 response, so distal motion is
preferred near an arm singularity. Every clutch engage/release or mode edge now
captures both the measured task anchor and the current redundant arm posture.
Therefore a zero haptic delta cannot pull the arm toward a startup/previous-mode
null-space target. During an engagement the reference follows healthy motion
with an approximately 10 s time constant and freezes at that engagement's last
healthy posture near a singularity; singularity speed scaling remains active.

The configured target is currently -15 N. A PI-plus-axial-damping force law and
physical force/actuator output bounds remain active; reaching the target or
haptic saturation does not terminate teleoperation. Mode 3 does not use Bota.
The Flexiv seven-axis loop is hard realtime. q8/q9 remain a multi-rate actuator
path: C++ computes their coupled OSC targets/feed-forward, while the moteus
position loop tracks those targets over the slower Python/USB bridge. Validate
custom contact control on a compliant non-human fixture before any medical use.

Clutch release is a separate immediate joint brake/hold state. The controller
captures the measured seven arm joints and q8/q9, clears target linear/angular
velocity, and applies `K(q_release-q)-D*qdot`. The previous implementation
restarted a one-second torque ramp on clutch edges, temporarily removing either
braking or Cartesian support. Engage and release now use the complete stiffness
about the just-captured measured pose; the final torque-rate limiter still
guarantees continuous actuator commands. Startup and stale-state recovery keep
their independent ramps.

### 6. Stable bilateral feedback

The force path uses bias removal, coordinate mapping, gain/deadband, low-pass,
slew limiting, engagement ramp, local damping, and a time-domain passivity
energy tank. The 12 N omega.7 rating saturates output but never terminates the
mission. Small continuous input deadbands prevent delayed force from moving the
handle, moving the robot, and returning a larger force in a positive loop.

### 7. Safety boundaries

Physical travel, actuator continuous ratings, torque slew, communication
watchdogs, robot/drive faults, and operational-state checks remain. They do not
limit normal workspace use; they prevent torque output when feedback is unknown.
