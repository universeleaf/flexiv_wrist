# 三模式控制算法 / Three-Mode Control Strategy

本文说明 `scripts/run_9dof_teleop.py` 的脚踏板/UI 遥操作。独立的 Flexiv C++ 1 kHz
7DoF/9DoF 力矩 OSC Demo 见 [`RT_TORQUE_OSC_ZH.md`](RT_TORQUE_OSC_ZH.md)。

## 中文

### 公共状态机与坐标关系

脚踏板或 UI 按钮只选择模式；omega.7 clutch 才是运动 dead-man。切换模式时立即冻结当前
Flexiv TCP 目标并更新腕部保持状态。如果切换时 clutch 已按下，必须先松开，再次按下时才
捕获新的手柄、探针和腕部锚点，因此不会沿用旧模式的相对位移而跳变。

腕部正运动学为：

```text
world_T_probe(q8,q9) = world_T_flange * flange_T_probe(q8,q9)
```

给定目标探针位姿和实测腕部角，主臂目标由下式补偿：

```text
world_T_flange* = world_T_probe* * inverse(flange_T_probe(q8,q9_measured))
```

因此腕部运动不会被错误地当成探针 TCP 漂移；Flexiv 会同时补偿腕部产生的平移和姿态变化。

### Mode 1：7DoF 遥操作

控制类型：Flexiv NRT 笛卡尔位姿控制；腕部不参与手柄目标。

- 选择 Pedal/UI Mode 1 后，joint 8 / ID2 回到配置零位；
- joint 9 / ID1 保持 STOP，不在启动时回零，也不被位置环使能；
- clutch 按下后，omega.7 相对平移和旋转形成目标探针位姿；
- 根据实测腕部几何换算法兰/active TCP 目标，由 Flexiv 7 个关节完成；
- clutch 松开后 Flexiv 保持当前 TCP，触觉用户力归零。

这是 7DoF 主臂遥操作，不是用户自定义关节力矩 OSC。真正的 7DoF RT torque OSC 是独立
`run_7dof_torque_osc_demo.py`。

### Mode 2：9DoF 协同位姿遥操作

控制类型：Flexiv NRT 笛卡尔位姿控制 + moteus 腕部位置控制 + 几何任务分配。

手柄给出完整探针位姿目标。探针姿态误差先变换到法兰坐标，并乘以
`wrist_priority_gain`，再用阻尼最小二乘迭代求腕部目标：

```text
dq_w = Jw^T (Jw Jw^T + lambda^2 I)^-1 e_R
```

两轴腕部瞬时最多覆盖两个旋转方向。可达部分优先由 q8/q9 完成；第三个方向、平移和腕部
运动造成的尖端位移由 Flexiv 通过上面的法兰补偿完成。求解器每次迭代限制角步长并保留
关节边界 margin，moteus 以位置/速度/加速度约束跟踪腕部目标。

“尽可能使用腕部”由以下参数共同决定：

- `[allocation].wrist_priority_gain`：放大腕部承担的可达旋转；
- `[allocation].damping`：阻尼最小二乘奇异点正则化；
- joint 8/9 几何、零位和实际关节范围；
- Flexiv 对腕部运动的法兰补偿。

真正的 9DoF 动态一致 torque OSC 是独立 `run_9dof_torque_osc_demo.py`；它使用复合
Jacobian 和实时 `M_wrist(q8,q9)`，而不是这里的位置分配器。

### Mode 3：只输入姿态 + 探针 Z 恒力

控制类型：Flexiv NRT 笛卡尔位置/力混合控制 + moteus 两轴旋转任务空间力矩阻抗。

- omega.7 的 XYZ 平移被完全丢弃；移动手柄位置不能改变机器人目标；
- 只有 omega.7 的相对旋转改变探针目标姿态；
- X/Y 保持 clutch 按下时的探针位置；
- 探针 Z 是 Flexiv 内置力控制轴，默认 sensed target 为 `-15 N`；
- Z 可以随力控制产生小位移，因此位置不是刚性固定；
- 腕部优先承担它能实现的两个姿态方向，Flexiv 完成剩余姿态并补偿 TCP。

腕部力矩阻抗：

```text
e_R = log(R_target R_measured^T)
omega_w = Jw dq_w
m_R = K_R e_R - D_R omega_w
tau_w = Jw^T m_R + tau_gravity(q_w, R_world_flange)
```

输出经过模组连续额定力矩饱和、关节软限位和通信 watchdog。这里的 OSC 是两轴腕部的旋转
任务空间阻抗；Flexiv 侧使用其内置的笛卡尔运动/力控制器。若需要整个 7+2 轴统一的动态
一致 OSC，应运行独立 9DoF C++ Demo。

### 探针 Z 力：只使用 Flexiv

当前运行不启动也不读取外置 Bota。Flexiv 给出的补偿 world-frame endpoint force 通过实时
法兰姿态与 q8/q9 转到探针坐标：

```text
R_world_probe = R_world_flange * R_flange_probe(q8,q9)
F_probe = R_world_probe^T F_world
F_probe_z = F_probe[2]
```

纯力向量不因 TCP 参考点平移而变化；如果要换算力矩，则还需要力臂叉乘。模式 3 把
arbitrary force-control frame 更新到探针姿态，因此 Flexiv command wrench 的 Z 分量就是
探针 Z 方向。

### 稳定触觉反馈

Flexiv wrench 经启动 bias、坐标逆映射和比例增益后，不再直接输出给 omega.7。稳定层执行：

1. 一阶低通，抑制估计器/齿轮纹波；
2. engagement ramp，消除按下 clutch 时的力阶跃；
3. 矢量 slew limit，限制每秒力变化；
4. 本地速度阻尼，直接在 1 kHz 触觉端耗散能量；
5. 时间域无源性观察器/有界能量罐，只削掉延迟闭环将要净注入的能量分量；
6. omega.7 连续额定力饱和，饱和不会退出程序。

当关闭反馈后抖动消失，说明机械结构/位姿控制本身未必是主要激励源，延迟双边力回路更
可疑。应从当前保守增益逐级实测，不应先关闭无源性层再提高反馈增益。

## English

### Common state machine and transforms

The pedals or UI buttons latch a mode; the omega.7 clutch remains the motion dead-man. A mode
change freezes the current Flexiv target. If the clutch is already held, it must be released before
the next press captures fresh haptic, probe, and wrist anchors.

The live probe pose is

```text
world_T_probe(q8,q9) = world_T_flange * flange_T_probe(q8,q9)
```

and the arm target compensates the measured wrist motion:

```text
world_T_flange* = world_T_probe* * inverse(flange_T_probe(q8,q9_measured))
```

### Mode 1: 7-DoF teleoperation

Control type: Flexiv NRT Cartesian pose control; the wrist does not follow the handle.

- Joint 8 / ID2 returns to the configured zero.
- Joint 9 / ID1 remains in STOP and is not zeroed at startup.
- While clutched, relative omega.7 translation and rotation define the probe target.
- The Flexiv seven-joint arm realizes the compensated active-TCP target.
- Releasing the clutch holds the current TCP and clears user haptic force.

This is not user-defined joint-torque OSC. The separate 7-DoF RT implementation is
`run_7dof_torque_osc_demo.py`.

### Mode 2: coordinated 9-DoF pose teleoperation

Control type: Flexiv NRT Cartesian pose control, moteus wrist position control, and geometric task
allocation.

The desired probe orientation is transformed into the flange frame, amplified by
`wrist_priority_gain`, and allocated using damped least squares:

```text
dq_w = Jw^T (Jw Jw^T + lambda^2 I)^-1 e_R
```

The wrist handles the two instantaneously reachable rotation directions. Flexiv handles translation,
the residual rotation, and compensation for wrist-induced tip motion. The distinct dynamically
consistent 9-DoF torque OSC is implemented by `run_9dof_torque_osc_demo.py`.

### Mode 3: orientation input only plus constant probe-Z force

Control type: Flexiv NRT Cartesian motion/force control plus two-axis wrist rotational task-space
torque impedance.

- All omega.7 translational input is discarded.
- Only relative handle rotation changes the probe orientation.
- X/Y remain at the clutch engagement point.
- Probe Z is the Flexiv built-in force-controlled axis, with a default sensed target of `-15 N`.
- Z may move slightly to regulate contact force; it is not rigidly position-fixed.
- The wrist handles reachable orientation components first and Flexiv supplies the residual.

The wrist torque law is

```text
e_R = log(R_target R_measured^T)
m_R = K_R e_R - D_R Jw dq_w
tau_w = Jw^T m_R + tau_gravity
```

### Probe-Z force without an external sensor

No external Bota process or configuration is used. The compensated Flexiv world-frame endpoint
force is projected through the live flange and q8/q9 orientation:

```text
F_probe = R_world_probe^T F_world
F_probe_z = F_probe[2]
```

Force is invariant to reference-point translation; moment conversion would additionally require the
lever-arm cross product.

### Stable haptic feedback

The feedback path applies bias removal, frame mapping, low-pass filtering, engagement ramping,
vector slew limiting, local damping, a bounded time-domain passivity energy tank, and the omega.7
continuous-force saturation. The passivity layer removes only the energy-injecting component caused
by the delayed bilateral loop; it does not deliberately hide static contact force.
