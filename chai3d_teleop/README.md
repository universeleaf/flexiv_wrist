# CHAI3D → Flexiv Rizon 遥操作（安全起步版）

三脚踏板选择 7-DoF、9-DoF 和固定探针点姿态控制的算法、标定与运行步骤见
[`docs/NINE_DOF_TELEOP.md`](docs/NINE_DOF_TELEOP.md)。

真正使用 Flexiv C++ `RT_JOINT_TORQUE` 的独立 7DoF/9DoF OSC Demo、腕部惯量辨识及
PREEMPT_RT 配置见
[`docs/RT_TORQUE_OSC_ZH.md`](docs/RT_TORQUE_OSC_ZH.md)。这些 Demo 默认只生成轨迹，
只有提供各自的实机确认口令才会发送力矩。

9-DoF 配置、启动、停止、实时日志和 Mode 1/2/3 切换也可以全部在本机 Web UI 中完成：

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop
python3 scripts/teleop_control_ui.py
```

浏览器将打开 `http://127.0.0.1:8765/`。UI 模式按钮与三个实体脚踏板同时有效，详细说明见
[`docs/START_9DOF_TELEOP_VSCODE_ZH.md`](docs/START_9DOF_TELEOP_VSCODE_ZH.md)。

本工程读取 CHAI3D 支持的触觉设备，并把**相对位移**映射到 Flexiv Rizon TCP。
它面向本机已有的 Robot Software 3.11.x 和 Python `flexivrdk==1.9.0`；不要链接
工作区中的旧版 `flexiv_rdk-1.7` 来控制当前机器人。

## 设计边界

- 默认 dry-run，不连接 Flexiv。
- 真实运动需要同时提供 `--arm --confirm MOVE_RIZON`。
- 设备按钮 0 默认是持续按住才运动的 dead-man/clutch；松开即保持当前 TCP。
- 每次按下按钮都重新捕获设备零点和当前 TCP，因此不会跳到设备绝对坐标。
- 默认只映射平移；姿态需要显式 `--enable-rotation`。
- 默认单次 clutch 限制为 5 cm；真实模式另有相对启动 TCP 的整次 1 cm 总限位。这两项可由
  `--unlimited-translation` 一起关闭。默认平移比例为 2.0、Flexiv NRT 命令频率为 100 Hz、
  omega.7 设备循环为 1000 Hz，设备 watchdog 为 100 ms。
- 程序不会自动清 fault、Enable 或回 Home。异常、数据超时或 Ctrl-C 都会调用
  `Robot.Stop()`。应用层原有的 15 N / 5 Nm 碰撞停机阈值已移除；Flexiv 控制器自身的
  安全保护不会被绕过。
- 可显式启用 Flexiv TCP 外力到触觉设备的三轴力反馈。反馈只在 clutch 按下时输出；松开、
  命令超时、异常和退出都会归零。
- bridge 默认显式启用并验证 Force Dimension DHD 重力补偿；用户反馈为零时，SDK 仍会在
  每个 1 kHz 力命令中叠加支撑设备手柄自身重量所需的竖直力。
- 保存配置使用可回驱的重力平衡模式：clutch 松开时停止机器人遥操作并归零用户反馈力，但
  DHD 重力补偿继续运行；无人施力时手柄保持平衡，手推时仍可移动。可选的 DRD 刚性位置保持
  代码仍保留，但当前配置不启用。
- omega.7 支持三轴力输出，但不支持姿态力矩输出，因此本工程不会伪造力矩反馈。

这些软件保护不能代替机械限位、安全围栏、示教器急停和现场风险评估。

## 0. 使用保存的透明配置

当前确认满意的配置保存在 `config/transparent_omega7.toml`，文件内每个参数都有中文注释。
启动脚本会自动切换到项目的 Flexiv Python 虚拟环境，因此无需激活环境或重新输入参数：

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop
python3 scripts/run_transparent_teleop.py
```

该配置包含 `--arm` 和两个实机确认字符串，所以命令启动后会连接真实 Flexiv。程序仍要求
启动前把手柄放在希望的物理中间位置并松开 clutch，程序会锁存该位置且不主动回中；随后执行
3 秒倒计时。运动期间松开 clutch 即原地保持，反馈立即归零。需要修改
机器人序列号、网卡 IP、比例、速率或反馈参数时，直接编辑 TOML 文件即可。原始
`python3 scripts/run_teleop.py` 无参数运行仍是 dry-run。

## 1. 获取 CHAI3D

从工作区根目录执行：

```bash
cd /path/to/Flexiv-main
git clone --depth 1 --single-branch \
  https://github.com/manips-sai-org/chai3d.git chai3d
```

仓库较大（GitHub 报告约 257 MiB），下载可能需要几分钟。

## 2. 安装 Linux 构建依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev \
  libgl1-mesa-dev libopenal-dev libusb-1.0-0-dev
```

本机当前已经检测到这些开发包。具体设备还可能需要厂商 SDK，例如 Force Dimension
设备使用 CHAI3D 仓库内的 DHD/DRD 库；设备权限则可能需要厂商 udev 规则。

## 3. 构建设备 bridge

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop
cmake -S . -B build -DCHAI3D_ROOT=/path/to/Flexiv-main/chai3d
cmake --build build --target chai3d_device_stream -j4
```

先枚举设备：

```bash
./build/chai3d_device_stream --probe
```

再观察设备 0 的原始数据；没有 stdin 反馈命令时不会控制机器人，也不会施加触觉力：

```bash
./build/chai3d_device_stream --device 0 --rate 250 --samples 20
```

如果 `count=0`，先不要运行遥操作。检查 USB、厂商驱动/SDK、udev 权限，并确认没有
其他程序独占设备。

## 4. dry-run 验证 dead-man 与坐标轴

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop
source ../freedrive_python/.venv/bin/activate
python3 scripts/run_teleop.py
```

按住设备按钮 0，缓慢沿单一设备轴移动。终端显示 7 维目标
以及设备原始位置 `raw_mm`、本次 clutch 相对位移 `delta_mm` 和映射目标
`target_mm`。确认三根设备轴与机器人 world 坐标的方向后再设置映射。
例如 `--axis-map x,-z,y` 表示设备 x/y/z 分别映射为 world x/-z/y；映射必须保持右手系。

本机 omega.7 已实测需要同时反转 X/Y，默认映射因此设为 `-x,-y,z`：

```bash
python3 scripts/run_teleop.py --axis-map=-x,-y,z --scale 0.3
```

若设备按钮不是 0，改用 `--switch 1` 等。不要绕过 dead-man。

程序现在每次启动都会检查 Force Dimension 设备在本次上电后是否已经初始化，必要时会先
自动初始化。若要强制重新初始化，可清空触觉设备周围空间、松开手柄，然后单独运行：

```bash
python3 scripts/run_teleop.py --calibrate-device
```

此参数会触发 omega.7 自动初始化运动，所以禁止与 `--arm` 同时使用。初始化完成后退出，
再重新运行普通 dry-run。若仍不变化，关闭可能占用设备的其他 CHAI3D/DHD 程序并重试。

## 5. 第一次真实机械臂测试

1. 清空工作区，卸下不必要负载，把速度限制设低，确认 TCP/负载配置正确。
2. 一人握住示教器急停，另一人操作触觉设备；先用机器人 Reduced 模式。
3. 在 Flexiv UI 中人工清 fault 并 Enable；本程序不会替你做这两步。
4. 第一轮保持默认“仅平移”、`--scale 0.1`、最大 1 cm：

```bash
python3 scripts/run_teleop.py Rizon4s-123456 \
  --network-interface-ip 127.0.0.1 \
  --axis-map=-x,-y,z --scale 0.1 --max-translation 0.01 \
  --max-session-translation 0.01 --max-step 0.0005 \
  --arm --confirm MOVE_RIZON
```

把序列号和轴映射换成现场真实值。启动后三秒倒计时内保持按钮松开；按住按钮才运动，
松开后保持。任何异常立即松开按钮并按急停。

平移映射验证无误后，可加 `--enable-rotation`。默认旋转保护为：单次 clutch 最多 20°、
整次程序相对启动姿态最多 30°、每周期最多 0.2°。姿态映射错误的风险显著高于纯平移。

如果已经完成低角度实机验证，可显式添加 `--unlimited-rotation` 取消单次和整次角度范围
限制。此模式仍保留每周期姿态步长、RDK 最大角速度、watchdog 以及 Flexiv 自身
安全系统限制；“无限”仅表示本程序不再按累计角度退出。

如果需要在整个机器人可达工作空间内通过重复 clutch 移动，可添加
`--unlimited-translation`，并从命令中省略 `--max-translation` 和
`--max-session-translation`。此模式同时取消单次 clutch 与相对程序启动 TCP 的平移范围
检查。添加 `--unlimited-step` 可进一步取消应用层平移目标步长限制；
`--unlimited-angular-step` 对姿态执行相同行为。Flexiv NRT 运动生成器、关节限位、奇异点、
可达空间、dead-man、watchdog 及机器人安全系统仍然有效。

## 6. 三轴力反馈

使用 `--enable-force-feedback` 后，程序可读取 filtered 或 raw world-frame TCP 外力，减去进入
运动模式时记录的空载 bias，再通过 `axis-map` 的逆映射输出给触觉设备。启动倒计时和记录
bias 时，机器人 TCP 必须没有接触环境。保存配置使用 compensated/filtered 外力、0.5 N
连续死区、0.25 增益和 20 N/s slew；这是 1:1 NRT 闭环实测自激后的稳定基线。

全工作空间、2:1 位移和保守稳定力反馈命令：

```bash
python3 scripts/run_teleop.py Rizon4s-123456 \
  --network-interface-ip 127.0.0.1 \
  --axis-map=-x,-y,z --scale 2.0 \
  --unlimited-translation --unlimited-step \
  --enable-rotation --unlimited-rotation --unlimited-angular-step \
  --device-rate 1000 --command-rate 100 \
  --max-linear-velocity 0.5 --max-angular-velocity 1.0 \
  --max-linear-acceleration 2.0 --max-angular-acceleration 5.0 \
  --enable-force-feedback --force-feedback-source filtered \
  --force-feedback-gain 0.25 --force-feedback-deadband 0.5 \
  --max-device-force 4.0 --force-slew-rate 20 --teleop-damping 12 \
  --arm --confirm MOVE_RIZON \
  --confirm-force-feedback FORCE_1_TO_1
```

bridge 仍按照设备报告的 12 N 连续额定力做最终硬件饱和，并在 100 ms 内收不到新反馈命令时
立即归零。`--teleop-damping 12` 在 omega.7 的 1 kHz 循环中加入速度阻尼；静止时阻尼力为零。
终端的
`ext_force_N` 和 `device_force_N` 可用于检查 1:1 映射；由于 X/Y 轴反向，分量符号会按照
`--axis-map=-x,-y,z` 改变，但向量幅值应相同，直到达到 12 N 饱和。

omega.7 的 CHAI3D 规格是最大连续力 12 N、`actuatedRotation=false`、最大角力矩 0 N·m，
所以终端会显示 Flexiv TCP moment 供观察，但手柄只能产生 XYZ 力反馈。

启动日志应包含类似 `gravity=1 effector_mass_kg=...`。`gravity=1` 表示厂商重力补偿已确认
开启；如果手柄仍明显下坠，请停止程序并保存整行 `CHAI3D_READY` 日志。此时可能需要根据
设备实际安装角度或改装后的末端质量调整 DHD 参数，不能用一个猜测的固定向上力代替，因为
正确补偿力会随机构姿态变化。

当前保存配置设置 `hold_when_released=false`，只使用厂商重力补偿实现可回驱平衡。
`hold_stiffness`、`hold_damping`、`hold_max_force` 和 `hold_slew_rate` 不会生效；它们只供
旧弹簧兼容模式使用。若以后启用 `hold_when_released=true`，则会切换为不能手推移动的 DRD
刚性位置保持。

该程序仍使用 Flexiv 的 NRT 模式。提高到 100 Hz 并移除应用层平滑可减少可感延迟，但不等于
严格意义上的透明双边遥操作；后者通常需要 1 kHz RT 控制、实时操作系统以及针对通信延迟的
passivity/stability controller。
