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

设备桥只负责时间戳、状态、命令和 watchdog；控制器不打开 USB 或机器人。
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

辨识默认采用 joint 8 ±15°、joint 9 ±30°、90 秒的多个不同频率平滑正弦激励，记录 q、dq、估计 ddq、输出扭矩和实时
flange 姿态。最小二乘拟合 rigid-body scale、反射惯量、粘性/库仑摩擦和 bias，
并记录 RMS 与条件数。9DoF `auto` 模式对拟合质量做 gate；不合格数据不会直接
驱动逆动力学。当前实机配置要求 `require_identified_inertia=true`，因此不合格时
直接拒绝启动，而不是回退到 Elements baseline + nominal wrist 参数。

`dynamic-inertia` 每个样本重新计算 `M_wrist(q)` 2×2 和腕部相对 flange 的
`I_spatial(q)` 6×6。6×6 空间惯量不是九关节质量矩阵，两者用途不同。

### 6. 遥操作模式

clutch 按下时捕获手柄和机器人相对 anchor；目标是相对位姿而非绝对设备坐标。
模式切换时必须先 release 再 press，防止跨模式继承旧 anchor。

- Mode 1：腕部 q8 保持零、q9 STOP；Flexiv 用 NRT Cartesian motion 执行
  7DoF 末端目标。
- Mode 2：腕部 IK 优先吸收两个姿态方向，moteus 使用平滑位置目标；Flexiv
  每周期补偿实时腕部引起的探针位姿变化。
- Mode 3：`orientation_only_target()` 完全替换掉手柄平移，探针 X/Y 锚定，
  探针 Z 由力控制允许小位移；腕部用 `τw=Jωᵀ(KReR-Dω)+g(q)`，Flexiv 使用
  自身 `NRT_CARTESIAN_MOTION_FORCE` 的 Tool-Z 内环保持 -15 N。

Mode 3 是混合任务空间运动/力控制；当前 Flexiv 部分使用厂商内部力环，而不是
自写的 RT_JOINT_TORQUE 接触控制器。这一选择避免在没有经过接触稳定性验证时把
自定义 1 kHz 关节扭矩环直接用于人体/医疗接触。独立 air demo 才使用自写
RT torque OSC。代码和文档刻意不把 NRT hybrid force control 冒充成自写 RT OSC。

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

Mission orchestration lives in `scripts/`, hardware-free math in `controllers/`,
device I/O in `hardware/`, setup/calibration in `tools/`, and validation in
`testing/`. Controller code never opens the robot or USB devices.

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

### 5. Teleoperation and force

Mode 1 is 7DoF Cartesian teleoperation. Mode 2 gives reachable orientation to
the wrist and compensates its live kinematics at the Flexiv flange. Mode 3
mathematically discards haptic translation, accepts orientation only, controls
the wrist with rotational impedance, and uses Flexiv's internal hybrid
Cartesian Tool-Z force loop at -15 N. It does not use Bota.

Mode 3 is hybrid operational-space motion/force control, but its Flexiv contact
part is the vendor NRT force controller—not a custom RT joint-torque contact
controller. This distinction is deliberate and is safer until a custom contact
controller has been validated on a non-human test fixture.

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
