# Flexiv 7/9-DoF 项目运行手册 / Project Runbook

## 中文

### 1. 项目目录和唯一入口

项目目录：

```bash
cd /home/src4/Flexiv-main/rizon4_tasks/chai3d_teleop
```

以后优先使用一个入口，不再记忆长参数：

```bash
python run.py --help
```

`run.py` 会自动选择 `freedrive_python/.venv` 或 `.venv_moteus`，并读取
`config/nine_dof_teleop.toml`。UI 和命令行使用同一份配置。

#### 1.1 新终端中激活 Python 环境

Ubuntu 默认可能只有 `python3` 命令。新建 VS Code 终端后，如果运行
`python run.py --help` 时看到 `Command 'python' not found`，说明当前终端
还没有激活虚拟环境；不是代码、硬件或 `dialout` 权限故障。

推荐每次打开新终端后执行：

```bash
cd /home/src4/Flexiv-main/rizon4_tasks/chai3d_teleop
source ../freedrive_python/.venv/bin/activate
```

终端提示符开头出现 `(.venv)` 后即可使用短命令：

```bash
python run.py --help
python run.py demo7
python run.py demo9
python3 run.py teleop
python run.py ui
```

即使已经激活 `freedrive_python/.venv`，`run.py` 仍会根据子命令自动调用
正确的解释器：Flexiv 任务使用 `freedrive_python/.venv`，moteus 专用任务
使用 `chai3d_teleop/.venv_moteus`，不需要手动来回切换环境。

如果只想临时查看帮助而不激活环境，也可以使用系统解释器：

```bash
python3 run.py --help
```

离开虚拟环境时运行：

```bash
deactivate
```

### 2. 新 VS Code 窗口的系统检查

每次重新登录或打开新窗口后先执行：

```bash
cd /home/src4/Flexiv-main/rizon4_tasks/chai3d_teleop
groups
ls -l /dev/serial/by-id/usb-mjbots_fdcanusb_0FD8590C-if00
uname -r
ulimit -r
chrt -f 99 true
```

预期：

- `groups` 包含 `dialout`；
- fdcanusb 链接存在，目标通常是 `/dev/ttyACM0` 或 `/dev/ttyACM1`；
- 内核名称包含 `realtime` 或 `rt`；
- `ulimit -r` 至少为 `99`；
- `chrt -f 99 true` 没有报错。

如果刚执行过 `sudo usermod -aG dialout $USER`，仅重开终端不够。必须完全注销桌面会话并重新登录，再彻底关闭和重新打开 VS Code。`sg dialout` 只作为旧会话的临时补救，UI 会在需要时自动使用它。

### 3. 首次构建与静态验证

```bash
python run.py build
python run.py test
python run.py check-tool
```

`build` 编译两个 C++ 1 kHz 实时控制器；`test` 不连接任何硬件；
`check-tool` 只读 Flexiv Elements 当前 Tool、质量、CoM、惯量和 TCP。

物理腕部、探针或传感器发生拆装后，必须先在 Flexiv Elements 重新做
Tool payload/TCP 标定，再把读出的数值更新到 `[flexiv_tool]`。腕部惯量辨识
不能替代 Elements 的机械臂负载补偿。

### 4. 腕部零位

#### 4.1 把当前机械姿态定义为 q8=q9=0

1. 关闭其他所有占用 fdcanusb 的程序。
2. 手动把 joint 8 和 joint 9 放到真实机械/几何零位。
3. 运行：

```bash
python run.py set-zero
```

程序只先发送 STOP 和 QUERY，然后把 ID2、ID1 原始位置写入
`wrist.zero_position_rev`。保存是原子的，并保留一份最近备份；不会修改
moteus 编码器永久偏置，也不会命令电机运动。

配置顺序始终为 `[ID2, ID1] = [joint 8, joint 9]`。joint 8 的范围是
`-90°..+90°`；joint 9 的范围是 `-360°..+360°`。

#### 4.2 让腕部运动到已保存零位

```bash
python run.py go-zero
```

程序以配置的速度、加速度、位置 PID 缩放和额定输出扭矩运动到零位；达到
零位后发送 STOP 并退出。

#### 4.3 整机 Home

```bash
python run.py home
```

程序不再执行低位的官方 `Home` primitive。它使用 Flexiv 文档中 Rizon 4(s)
的默认 Home 关节位姿 `[0,-40,0,90,0,40,0]°`，通过当前机器人的实时标定
模型计算该位姿的法兰位姿，再叠加 Elements 当前 active Tool TCP，得到原始
Home TCP。最终目标只把这个 Home TCP 的世界 Z 增加 `0.20 m`，X/Y/姿态不变。
原生 `Home` 是关节空间 `JPOS` 命令，不是 TCP 目标；它不会因工具变长而自动抬高
工具尖端。本项目的原始 Home 关节角只作为正运动学参考，最终实际控制的是 active
Tool TCP，最终七轴关节角由 IK 求得，因此通常不等于原始 Home 关节角。

顺序是：两腕关节回零并保持 → RDK 正运动学计算高位 Home TCP → RDK IK 检查
目标可达并选择接近 Home 的冗余姿态 → Flexiv 从当前位置直接运动到高位 Home
TCP → 在 `3 mm` 内稳定 `0.30 s` 后 Stop。运动过程不会先经过低位 Home 点。
默认速度为 `0.05 m/s`，加速度为 `0.20 m/s²`。上述数值和 Home 参考关节角在
`[home]` 中，可从 UI 修改并保存。更换长工具后，必须先在 Elements 正确标定
并选中 active Tool；当前路径到高位目标的完整扫掠空间仍必须清空。

### 5. 电机 PID

当前推荐且与驱动器实测一致的永久位置 PID：

| Joint | ID | Kp | Ki | Kd |
|---|---:|---:|---:|---:|
| joint 8 | 2 | 1000 | 1 | 30 |
| joint 9 | 1 | 1200 | 1 | 20 |

正常任务不会反复写永久 PID。UI 编辑 `[motor_pid]` 后，只有执行下面命令才会写入两个 moteus 并 `conf write`：

```bash
python run.py apply-pid
```

位置任务还使用临时 `position_kp_scale=[0.2,0.2]` 和
`position_kd_scale=[1.0,1.0]`。当前 9DoF 默认使用
`osc_execution_mode="hybrid_position_ff"`：腕部跟踪同一个耦合 OSC 给出的
q8/q9 目标，同时叠加模型前馈力矩。原因是这套 2024 moteus 固件虽接受纯前馈
命令，却只测得约 0.02--0.04 N·m 实际力矩。Flexiv 七轴仍是 RT 关节力矩控制。

### 6. 两个对比 Demo

先确认 Flexiv 已 Enable、无 fault、Tool 正确、急停可触及。

7DoF：

```bash
python run.py demo7
```

9DoF：

```bash
python run.py demo9
```

两者都从 `[demo]` 读取完全相同的末端闭环轨迹，默认周期 20 s、轨迹参数半径
10 mm（相对启动点最大 X 位移 20 mm）、姿态幅度 35°，并一直循环到 Ctrl-C。
两者启动时都先让 q8/q9 回零。
轨迹同时包含三个不同频率的姿态分量，但不会穿越 180°/360° 姿态分支。

- 7DoF 中 q8/q9 保持零位，只有 Flexiv 七关节完成位置和高频姿态变化；
- 9DoF 中 q8/q9 以 OSC 目标 + 前馈力矩混合内环优先承担其可达的两个旋转方向，Flexiv 负责
  剩余姿态、平移和冗余补偿，因此同一末端任务应需要更小的机械臂整体动作。

旧的固定点完整 360° 轨迹已从默认任务移除。它在 7DoF 实机日志中造成 76 mm
位置失跟和约 90° 姿态失跟，不能作为安全的共同任务。当前 7DoF 已恢复到公开
commit `2f1e763` 的已知工作控制器，不再因自定义位置/姿态误差阈值主动退出。
当前安全 `loop` 轨迹没有回退，9DoF 没有修改。启动前仍需选择远离奇异位形和
机身的姿态，并确保急停可触及。

按 Ctrl-C 后，Flexiv 切到 IDLE，腕部冻结当前位置并 STOP。

### 7. 三模式遥操作

```bash
python3 run.py teleop
```

这是完整三模式入口；启动一次后，脚踏板和 UI 可直接切换全部模式：

- Pedal 1 / Mode 1：Flexiv 七轴执行内置笛卡尔阻抗，q8/q9 不参与任务。
- Pedal 2 / Mode 2：同一 Flexiv 阻抗控制器；q8/q9 先以自然 1:1 承担腕部可达
  姿态，实际探针 TCP 尖端仍严格跟踪目标点。若旋转完全属于 Joint 8/9 可达方向，
  Flexiv 法兰不旋转，只做抵消转轴到 TCP 偏心圆弧所需的小量平移；手柄平移仍
  移动实际 TCP 目标。腕部命令采用随实测角移动的 15° 跟随窗口，不限制累计行程，
  但不会因机构停住而积累大误差并退出。
- Pedal 3 / Mode 3：同一 Flexiv 会话内的柔和混合操作空间控制。按下 clutch 时固定
  探针 TCP 接触点，完全丢弃手柄平移，只接受手柄旋转。q8/q9 优先承担可达姿态，
  腕部采用“OSC 目标和动力学前馈 + moteus 内部位置环”；Flexiv 仅补偿偏心 TCP
  圆弧和腕部无法表达的剩余姿态。先在非人体测试面建立同方向至少 2 N 的轻接触，
  Tool-Z 力轴才会启用，并从实测接触力在 3 秒内缓升到 -15 N。接触丢失时只退回
  固定点位置保持，不退出程序。

Mode 1 的 XZ 平面旋转方向与 Mode 2 分开配置，当前已反转；如果只验证这个方向，
请踩 Pedal 1、松开 clutch、重新按下，然后只做小角度 XZ 平面旋转。

腕部正常位置/混合模式应显示 `servo_mode=[10,10]`。`mode=11` 是 moteus 命令 watchdog
超时；当前驱动 watchdog 已设为 1000 ms，以覆盖双电机 fdcanusb 顺序事务，桥上游
安全 watchdog 仍为 100 ms。若看到 `WRIST_POSITION_MODE_RECOVERY`，表示桥正在重发
位置命令使驱动自动回到 mode 10。

显式运行同一个阻抗入口：

```bash
python3 run.py teleop-impedance
```

保留的实验性“全部模式均使用 1 kHz Flexiv 关节力矩 OSC”入口：

```bash
python3 run.py teleop-osc
```

启动后先用脚踏板或 UI 选择模式，再松开 omega.7 clutch，重新按下开始。

- Pedal 1 / Mode 1：7DoF `RT_JOINT_TORQUE` OSC。腕部保持选中模式时的角度，Flexiv 七轴跟踪 omega.7 的 6D 相对位姿。
- Pedal 2 / Mode 2：腕部优先的 9DoF `RT_JOINT_TORQUE` OSC。q8/q9 先以 1:1 自然角度承担可达的两个姿态分量；Flexiv 用位置优先的分层 OSC 只完成平移和较慢的剩余姿态，并保持 clutch 捕获的冗余臂姿态，不做腕部倍数放大。
- Pedal 3 / Mode 3：固定接触点姿态 + TCP-Z 恒力 OSC。按下 clutch 时捕获完整 TCP XYZ 点；omega.7 平移在 Python/C++ 两层都被丢弃。q8/q9 优先完成姿态，Flexiv 以较低增益连续补偿固定点和剩余法兰姿态，不设会突然停止的残差角度截断。力控制必须先检测到同方向至少 2 N 的已有接触才锁存，空气中输出为零，不会自动搜索表面。

Mode 3 不使用 Bota。默认统一入口的 Flexiv 侧采用厂商混合笛卡尔运动/力控制，
腕部侧采用 OSC 外环加 moteus 位置内环；`teleop-osc` 仅保留作全力矩研究对照。
Flexiv 补偿外力投影到实时探针 Z 后闭环，达到目标力或 12 N 触觉饱和都不会退出
任务。进入 Mode 3 前先轻触非人体测试面，再按 clutch；状态里的
`flexiv_force_z_cmd_N` 从接触力缓升时才表示恒力环已经接管。

松开 clutch 时停止遥操作：控制器捕获当时实测的七轴角度和 q8/q9，立即清零
目标速度，并进入阻尼关节保持。松开不会触发运动接入 ramp，因此机械臂不会继续
追赶释放前的滤波目标。保持硬度由
`[teleop_osc].clutch_hold_natural_frequency_hz` 配置，回位是否反弹由
`clutch_hold_damping_ratio` 配置；默认 1 Hz、1.25（过阻尼）。它不会把
omega.7 拉回中心；omega.7 保持厂商重力补偿。任何模式切换都
要求“先松开再重新按下”，避免旧零点导致跳变。

运行日志出现以下行表示动态重力补偿已经启用：

```text
DYNAMIC_GRAVITY flexiv_internal_baseline=1 wrist_arm_delta=1 wrist_full=1
```

含义是：Flexiv 保留 Elements 零位 Tool 的基础补偿；控制器实时把 q8/q9 造成的
七轴重力变化加给 Flexiv，并把完整两轴重力发给 moteus。实时 CSV
`/tmp/flexiv_9dof_osc_rt.csv` 可查看三个重力诊断列。

### 8. 触觉反馈稳定配置

触觉力经过：空载 bias → 0.25 比例 → 死区 → 3 Hz 低通 → 15 N/s
slew → 1.2 s 接入斜坡 → 18 N/(m/s) 本地阻尼 → 能量罐无源控制 →
omega.7 12 N 额定饱和。达到 12 N 不会结束遥操作。

同时，手柄 0.4 mm 平移和 0.25° 旋转死区阻断“力推动手柄—手柄再命令机器人—
机器人产生更大力”的小信号正反馈。所有值都可在 UI 修改和保存。

### 9. 惯量辨识和动态矩阵

自动采集并拟合腕部参数：

```bash
python run.py identify-inertia
```

该任务不会重定义零位。默认执行 120 秒、速度较缓的多正弦运动：joint 8 为
±45°、joint 9 为 ±60°，峰值速度约为 38.6°/s 和 44.1°/s；同时使用辨识专用
位置增益以越过减速器静摩擦。幅度、时间和位置环缩放均保存在
`[inertia_identification]`，可从 UI 修改并保存。它保存原始数据到
`/tmp/wrist_inertia_samples.csv`，拟合结果写入
`config/wrist_inertia_calibration.json`。

这不是让九个关节同时运动的一次辨识。Flexiv 七轴的刚体模型来自 Elements/RDK，
本任务辨识 RDK 不知道的 Joint 8/9；实时控制器再把两部分耦合成随姿态更新的 9×9
关节质量矩阵。Flexiv 为 1 kHz 硬实时，而腕部经 Python/USB/CAN-FD 约 50--100 Hz，
目前若直接把两路未严格同步的数据做九轴回归，通信时延会被错误拟合成惯量。

只读确认真实 Elements Tool 基线与辨识是否被采用：

```bash
python run.py check-inertia
```

只有输出 `IDENTIFICATION_STATUS=PASS` 且
`auto_source=elements_baseline+identified_wrist` 才允许运行 9DoF。当前配置
`require_identified_inertia=true`，所以辨识不合格会明确停止，不会回退到名义腕部模型。

移动腕部并逐时刻输出/记录矩阵：

```bash
python run.py dynamic-inertia
```

默认运动为 joint 8 峰值 20°、joint 9 峰值 45°、周期 8 秒。CSV 为
`/tmp/wrist_dynamic_inertia.csv`，包含每个样本的 q/dq、2×2 腕部关节
质量矩阵和 6×6 flange 空间惯量。此任务只移动腕部，不命令 Flexiv。

### 10. UI

```bash
python run.py ui
```

浏览器打开 `http://127.0.0.1:8765/`。UI 可编辑并保存整个 TOML，包括
Motor PID、Demo、OSC、腕部、Tool、触觉反馈和 Mode 3；可启动所有 mission/tool，
显示实时日志，并可用 UI 按钮或脚踏板切换遥操作模式。一次只允许运行一个任务。

统一遥操作启动后，页面下方的 **Real-Time Tracking Plots** 会以 20 Hz 接收控制
进程已经计算好的状态，并保留最近约 60 秒数据。可选择显示最近 15/30/60 秒，或用
**Clear Plots** 清空曲线。共 14 张图：

1. Mode 3 Tool-Z 力全宽大图：实时测量、低通估计、目标和缓升命令（N）。
2. 探针 TCP 实际/规划位置（世界坐标 X/Y/Z，mm）。
3. 探针 TCP 实际/规划姿态（世界坐标旋转向量 RX/RY/RZ，deg）。
4. TCP 位置误差 X/Y/Z 及误差模长（mm，重点图）。
5. TCP 最短旋转误差 RX/RY/RZ 及误差角（deg，重点图）。
6. Joint 1--9 的九张独立关节误差图（deg）。

Mode 3 白色力线是 Flexiv 外力实时投影到探针 Z，蓝线是控制使用的额外低通估计，
不是两个独立传感器。当前没有 Bota，因此白线也不能称为独立绝对地面真值。

J8/J9 的规划值是腕部实际收到的位置目标。Flexiv 厂商 Cartesian 控制器不公开内部
关节轨迹，因此 J1--J7 的“规划值”定义为本项目传给 Flexiv 的零空间姿态参考，而
不是厂商内部逆解出来的瞬时关节目标。末端位置/姿态及其误差不受这个限制，均使用
真实探针 TCP 的实际值和任务目标值。

### 11. 常见错误

- `Permission denied ttyACM*`：完全注销/登录；确认 `dialout`；确认没有第二个程序占用串口。
- `device disconnected or multiple access`：关闭 moteus_tool、旧 UI、旧 Python 进程；重新插拔直连 USB。
- `wrist state stale`：短抖动进入保持并平滑恢复；连续 3 s 无有效状态会安全 Stop，不能为了“不断程序”而在未知反馈下继续输出扭矩。
- `active Tool mismatch`：在 Elements 选择/标定正确 Tool，再更新 `[flexiv_tool]`。
- 9DoF 抖动：先检查零位、驱动器方向、Elements payload、惯量辨识质量和 USB；不要先无条件增加 Kp。

## English

### 1. One entry point

```bash
cd /home/src4/Flexiv-main/rizon4_tasks/chai3d_teleop
python run.py --help
```

`run.py` selects the correct Python environment and loads
`config/nine_dof_teleop.toml`. The UI and CLI share that file.

#### 1.1 Activate Python in a new terminal

Ubuntu may provide only the `python3` command by default. If a new VS Code
terminal reports `Command 'python' not found`, the virtual environment has not
been activated in that terminal. This is not a code, hardware, or `dialout`
permission failure.

Run the following whenever you open a new terminal:

```bash
cd /home/src4/Flexiv-main/rizon4_tasks/chai3d_teleop
source ../freedrive_python/.venv/bin/activate
```

After `(.venv)` appears at the beginning of the prompt, the short commands are
available:

```bash
python run.py --help
python run.py demo7
python run.py demo9
python3 run.py teleop
python run.py ui
```

`run.py` still selects the appropriate interpreter for each subcommand:
Flexiv tasks use `freedrive_python/.venv`, while moteus-only tasks use
`chai3d_teleop/.venv_moteus`. There is no need to switch environments manually.

For a one-off help command without activation, use:

```bash
python3 run.py --help
```

To leave the environment, run:

```bash
deactivate
```

### 2. New-session checks

```bash
groups
ls -l /dev/serial/by-id/usb-mjbots_fdcanusb_0FD8590C-if00
uname -r
ulimit -r
chrt -f 99 true
```

The user must be in `dialout`, the adapter link must exist, the realtime kernel
must be active, and SCHED_FIFO priority 99 must work. After changing groups or
realtime limits, fully log out and back in and restart VS Code.

### 3. Build and verify

```bash
python run.py build
python run.py test
python run.py check-tool
```

Recalibrate the active Tool payload/TCP in Flexiv Elements whenever the
physical wrist/probe assembly changes. Wrist inertia identification does not
replace Elements payload compensation.

### 4. Wrist and system tools

Manually align both joints to their true geometric neutral, then save it:

```bash
python run.py set-zero
```

Move both axes to the saved zero and STOP:

```bash
python run.py go-zero
```

Move directly to a Cartesian destination 0.20 m above Flexiv Home:

```bash
python run.py home
```

The command no longer executes the low official Home primitive. It evaluates
the documented Rizon 4(s) Home joint posture `[0,-40,0,90,0,40,0] deg` with the
live calibrated RDK model, composes the active Tool TCP, adds 0.20 m only to
world Z, checks reachability using RDK IK seeded at Home, and moves directly
from the current pose to that elevated Cartesian destination. Home X/Y and
orientation are unchanged. It stops after remaining within 3 mm for 0.30 s.
The reference posture and motion settings are editable under `[home]` in the UI.

The active Tool in Elements must describe the long physical tool correctly,
and the direct path's complete sweep must still be clear.

The fixed ordering is `[ID2, ID1] = [joint 8, joint 9]`; joint 8 is ±90° and
joint 9 is ±360°.

### 5. Demos

```bash
python run.py demo7
python run.py demo9
```

Both run the exact same 20 s closed endpoint task indefinitely until Ctrl-C:
a loop with a 10 mm radius parameter (20 mm maximum X displacement from the
captured start) plus continuous multi-axis orientation with a 35-degree base
amplitude. The 7DoF task holds q8/q9 at zero. The 9DoF task gives every
reachable orientation component, without amplification, to q8/q9 so the Flexiv
arm should require less gross motion for the same endpoint path.

The old fixed-point 360-degree spin is no longer the default because its 7DoF
real log showed severe tracking loss and potentially self-colliding arm motion.
The 7DoF control core is now restored from the known-working published commit
`2f1e763` and no longer terminates on custom task-error thresholds. The current
safe loop trajectory and the 9DoF implementation are unchanged. This still
does not replace Flexiv collision safety or the operator's physical preflight.

### 6. Teleoperation

```bash
python3 run.py teleop
```

The default entry is one complete three-mode workflow. It uses Flexiv's built-in
`NRT_CARTESIAN_MOTION_FORCE` Cartesian impedance controller for Modes 1 and 2.
Mode 1 holds the external wrist out of the task. Mode 2 gives each reachable
orientation component once to q8/q9 while keeping the actual probe TCP at its
target. For a rotation fully reachable by joint 8/9, the Flexiv flange does not
rotate; it only translates by the small amount required to cancel the arc of
the offset TCP. The arm also does not temporarily perform a reachable wrist
rotation while the rate-limited wrist catches up. Haptic translation moves the
actual TCP target. At every clutch
engagement the current seven-axis posture is sent to Flexiv as the null-space
reference, reducing redundant whole-arm wandering.

Mode 2 uses desired wrist IK only for flange orientation and measured q8/q9
for the translation that cancels the live offset-TCP arc. A slow wrist may lag
in orientation, but the arm does not temporarily rotate in its place and TCP
position remains fixed. A moving 15-degree following window prevents command
lead from accumulating when an axis is obstructed; it does not cap total joint
travel. Mode 1 has a separate, reversed XZ-plane rotation sign.

With the clutch released, translational stiffness is 4% of Flexiv nominal and
rotational stiffness is 8%; null-space reference tracking is reduced to 0.10.
The released TCP remains the equilibrium, so hand force can deflect it easily
and removing that force returns it to the release point.

Mode 3 runs in the same process and robot session. It captures the probe contact
point, discards haptic translation, and accepts orientation only. q8/q9 take
their reachable orientation first using an OSC outer target/model feed-forward
with the stable moteus position inner loop. Flexiv supplies only TCP-offset
compensation and residual orientation through a gentle hybrid operational-space
motion/Tool-Z-force task. A same-sign contact of at least 2 N is required before
the force axis activates; its command then ramps from the measured contact force
to -15 N over 3 seconds. Contact loss returns to position hold without exiting.

Use `python3 run.py teleop-impedance` for the same unified backend. Use
`python3 run.py teleop-osc` only for the retained experimental all-mode C++
1 kHz joint-torque OSC. Release and re-press the clutch after selecting a mode.
Bota is not used.

On release, the controller captures the measured seven arm joints and q8/q9,
zeros the target twist, and immediately enters a damped joint hold. Release no
longer restarts the motion-engagement torque ramp, so the arm does not keep
following a residual filtered target. Configure the mass-shaped hold with
`[teleop_osc].clutch_hold_natural_frequency_hz` and
`clutch_hold_damping_ratio`.

The startup line
`DYNAMIC_GRAVITY flexiv_internal_baseline=1 wrist_arm_delta=1 wrist_full=1`
confirms that Flexiv retains the Elements zero-wrist baseline while the
controller adds the live q8/q9 arm-side gravity delta and sends full wrist-axis
gravity to moteus. The three gravity diagnostics are recorded in
`/tmp/flexiv_9dof_osc_rt.csv`.

### 7. Identification and live matrices

```bash
python run.py identify-inertia
python run.py dynamic-inertia
```

The first command collects and fits wrist dynamics without redefining zero. Its
default 120 s excitation moves joint 8 through ±45° and joint 9 through ±60°,
with peak speeds of about 38.6°/s and 44.1°/s. These values are editable in the
UI under `[inertia_identification]`.

This is intentionally not a simultaneous nine-axis parameter regression. The
Flexiv 7-axis rigid-body model comes from Elements/RDK; this command identifies
the external q8/q9 dynamics that RDK cannot know. Runtime control couples both
parts into a configuration-dependent 9×9 joint mass matrix. A naive joint fit
would mix the 1 kHz Flexiv stream with the 50--100 Hz Python/USB/CAN-FD wrist
stream and can falsely identify communication delay as inertia.
The second moves q8/q9 and records the live 2×2 wrist joint mass matrix and 6×6
spatial inertia at every sample.

### 8. UI and PID

```bash
python run.py ui
python run.py apply-pid
```

The localhost UI edits every saved setting and launches all tasks. Applying
PID is deliberately a separate action because it persists values to moteus.
Current commissioned PID is ID2 `(1000,1,30)` and ID1 `(1200,1,20)`.

After unified teleoperation starts, **Real-Time Tracking Plots** receives the
already-computed controller state at 20 Hz and retains about 60 seconds. The
14 charts are: one large Mode-3 Tool-Z force chart; actual/planned probe-TCP
position; actual/planned orientation; position error; orientation error; and
one error chart for each of joints 1--9. The force chart shows the projected
real-time Flexiv measurement, its additional low-pass controller estimate,
the sensed-force target, and the gated/ramped command. It is not an independent
Bota ground-truth measurement. Position is in world-frame millimetres. Orientation is shown as a
world-frame rotation vector in degrees, while orientation error is the
shortest target-to-actual rotation. The two task-error charts are highlighted.

For J8/J9, target means the commanded wrist position. Flexiv's Cartesian
controller does not expose its internal joint trajectory, so J1--J7 target
means the null-space posture reference supplied by this project, not a hidden
instantaneous IK target. The physical probe-TCP task plots are unaffected by
this distinction.

Communication watchdogs, actuator ratings, physical travel, fault handling,
and E-stop behavior are retained. They are safety requirements, not arbitrary
workspace restrictions.
