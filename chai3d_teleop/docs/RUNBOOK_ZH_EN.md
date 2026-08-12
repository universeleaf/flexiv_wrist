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

顺序是：两腕关节回零并保持 → Flexiv 执行官方 `Home` primitive → Flexiv
Stop → 腕部 Stop。运行前必须清空机械臂、腕部和工具的完整扫掠空间。

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
python run.py teleop
```

启动后先用脚踏板或 UI 选择模式，再松开 omega.7 clutch，重新按下开始。

- Pedal 1 / Mode 1：7DoF。腕部保持零位；omega.7 的平移和旋转映射给 Flexiv。
- Pedal 2 / Mode 2：9DoF。末端目标与 Mode 1 相同，但腕部优先承担两个可达的姿态分量。
- Pedal 3 / Mode 3：方向 + TCP-Z 恒力。omega.7 平移输入被明确丢弃；只有旋转改变探针方向。X/Y 由位置环保持，沿实时探针 Z 的位置只由 Flexiv 内部力环调节，目标默认 `-15 N`。

Mode 3 不使用 Bota。Flexiv 补偿后的世界坐标外力由实时 flange、q8、q9 姿态旋转到探针坐标，用于显示和触觉反馈；控制力轴使用 Flexiv 的 Tool-Z 混合运动/力控制。

松开 clutch 时停止遥操作并冻结 Flexiv 当前目标；不会把 omega.7 拉回中心。omega.7 保持厂商重力补偿。任何模式切换都要求“先松开再重新按下”，避免旧零点导致跳变。

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

该任务不会重定义零位。默认执行 90 秒明显可见的多正弦运动：joint 8 峰值
15°、joint 9 峰值 30°；同时提高辨识专用位置增益以越过减速器静摩擦。它保存原始数据到
`/tmp/wrist_inertia_samples.csv`，拟合结果写入
`config/wrist_inertia_calibration.json`。

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

Move the wrist to zero and execute the Flexiv Home primitive:

```bash
python run.py home
```

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
python run.py teleop
```

Mode 1 is 7DoF, Mode 2 is wrist-priority 9DoF, and Mode 3 ignores haptic
translation and accepts orientation only while Flexiv controls probe-Z force
at -15 N. Release and re-press the clutch after selecting a mode. The Bota
sensor is not used.

### 7. Identification and live matrices

```bash
python run.py identify-inertia
python run.py dynamic-inertia
```

The first command collects and fits wrist dynamics without redefining zero.
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

Communication watchdogs, actuator ratings, physical travel, fault handling,
and E-stop behavior are retained. They are safety requirements, not arbitrary
workspace restrictions.
