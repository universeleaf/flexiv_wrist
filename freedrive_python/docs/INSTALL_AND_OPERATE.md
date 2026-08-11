# Flexiv Rizon 4S 笛卡尔拖动与中间脚踏板操作说明

本程序面向：

- Flexiv Rizon 4S（机器人上报型号必须为 `Rizon4s` 或等价空格写法）
- Robot Software 3.11.x
- Flexiv RDK / `flexivrdk` 1.9.0
- Linux 与 iKKEGOL 兼容三联 USB 脚踏板

默认控制流程为：

```python
robot.ExecutePrimitive("ZeroFTSensor", {})
robot.ExecutePrimitive(
    "FloatingCartesian",
    {"floatingAxis": [1, 1, 1, 1, 1, 1], "enableElbowMotion": 0},
)
```

`floatingAxis` 打开 TCP 的 X/Y/Z/Rx/Ry/Rz 六个方向；
`enableElbowMotion=0` 关闭独立肘部/零空间拖动。程序不执行 `FloatingJoint`，
因此不能单独选择并拖动 A1、A2 等关节。操作员可以抓住末端工具或最后关节外壳，
调整末端 TCP 的位置和姿态。

## 启动与停止行为

- 程序启动后先调用 `Stop()`，保持锁定；不会执行 `Home`。
- 启动时自动运行一次官方 `ZeroFTSensor`；此阶段不要接触法兰、工具或机器人。
- 默认通过 X11 XInput 的物理设备 ID 监听 `PCsensor FootSwitch`，不读取 `/dev/input`。
- 确认 Stop 后才清空连接期间的旧踏板事件并武装脚踏板；武装时中踏板必须已松开。
- 第一次踩中间踏板：进入 `FloatingCartesian` TCP Freedrive。
- 第二次踩中间踏板：调用 `Stop()`，回到锁定/IDLE。
- Stop 期间排队的重复按键会被丢弃；中踏板松开后才重新武装下一次开启。
- 后续每次有效的 KEY_DOWN 继续交替开/关。
- KEY_UP、键盘自动重复、抖动和未映射按键会被忽略。
- 脚踏板断开、机器人断开、故障、异常或 Ctrl+C 都会进入 `Stop()` 清理。
- 实体急停始终是紧急停止手段；USB 脚踏板不是认证急停或使能装置。

## 1. 检查机器人

在 Flexiv Elements 中确认：

1. 机器人确实是 Rizon 4S，Robot Software 为 3.11.x。
2. RDK 已启用，运行模式为 `Auto`（部分版本写作 Auto/Remote；你当前的 Elements
   只显示 Manual/Auto 时选择 Auto）。
3. 动力学、运动学和关节力矩传感器标定正常。
4. 当前 Tool/TCP 与实际安装工具一致。
5. 工具质量、质心和惯量已标定；官方原语说明要求工具重量误差小于 100 g。
6. 急停可随时触及，工作区无人且没有奇异位形风险。

不要让程序自动清故障或自动修改 Tool/TCP/载荷；本程序也不会这样做。

## 2. 安装 Python 环境

```bash
cd /path/to/Flexiv-main/rizon4_tasks/freedrive_python
sudo apt update
sudo apt install -y python3-venv
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
python3 -c "import flexivrdk; assert flexivrdk.__version__ == '1.9.0'; print(flexivrdk.__version__)"
```

以后每次打开新终端都先执行：

```bash
cd /path/to/Flexiv-main/rizon4_tasks/freedrive_python
source .venv/bin/activate
```

## 3. 脚踏板读取方式（默认不修改 Linux 权限）

当前电脑识别到的脚踏板键盘接口是：

- 名称：`PCsensor FootSwitch Keyboard`
- USB ID：`3553:b001`
- 当前中间踏板配置：`KEY_B`，Linux key code `48`

默认启动脚本使用 X11 XInput，根据物理设备名称、slave-keyboard ID 和 USB ID
选择踏板。操作动作来自这个物理设备，不是来自字符 `B`：

- 普通键盘按 `B` 不会切换 Freedrive。
- 不需要加入 `input` 组。
- 不需要安装 udev 规则。
- 不需要读取 `/dev/input/event*`。
- 必须从本机 X11 桌面终端运行，不能从纯 SSH、Wayland 或无 `DISPLAY` 的服务运行。

## 4. 无机器人测试物理踏板

以下命令不连接机器人，也不使用 `/dev/input`：

```bash
python3 scripts/diagnose_xinput_foot_pedal.py --seconds 30
```

测试顺序：

1. 在实体键盘按一次 `B`，不应出现踏板事件。
2. 短踩并松开中间踏板，应只出现一条：

```text
PEDAL_2_TOGGLE count=1 physical_device='PCsensor FootSwitch ...' key_code=48
```

3. 再短踩并松开一次，应只增加到 `count=2`。

如果提示找不到 XInput 设备，运行 `xinput list --short`，确认其中有
`PCsensor FootSwitch` 的 `slave keyboard`。不要用 sudo 运行机器人程序。

## 5. 离线检查

以下命令不连接机器人：

```bash
python3 -m compileall -q src scripts tests
python3 -m pytest -q
python3 scripts/run_freedrive.py Rizon4s-123456 --print-command-only
```

打印结果应显示：

- primitive：`FloatingCartesian`
- `floatingAxis`：六个 1
- `enableElbowMotion`：0
- `zero_ft_before_freedrive`：true
- `home_before_freedrive`：false

## 6. 正式运行

```bash
./scripts/start_freedrive.sh
```

程序连上机器人并进入锁定等待状态后：

1. 第一次踩中间踏板，进入笛卡尔拖动。
2. 只在末端工具/TCP 附近引导机器人，并从很小的力开始。
3. 再踩一次中间踏板，退出拖动并锁定。
4. 按 Ctrl+C 结束整个程序。

如果想在正式运行前再次只读检查连接（不会 Enable、Stop 或移动机器人）：

```bash
python3 scripts/check_robot_connection.py \
  --robot-sn Rizon4s-123456 \
  --network-interface-ip 127.0.0.1
```

正常时应看到 `flexivrdk=1.9.0`、`connection=OK`、
`robot_software=v3.11` 和 `fault=False`。

也可以使用等价命令：

```bash
python3 scripts/run_freedrive.py Rizon4s-123456 \
  --confirm-motion --foot-pedal --foot-pedal-backend xinput
```

## 7. 首次低风险测试顺序

1. 先不踩踏板，确认机器人没有移动，终端显示等待 Pedal 2。
2. 踩一次后不接触机器人，观察 2–3 秒；若自行漂移，立即再踩一次并按需急停。
3. 仅对末端施加很小的平移力，确认方向正确。
4. 再踩一次，确认立刻退出拖动。
5. 再次启动后只做小角度姿态引导。
6. 确认只能通过 TCP 笛卡尔任务调整末端，不能像关节 Freedrive 那样单独拖动 A1/A2。

任何无接触漂移都应停止测试，并检查 Tool/载荷、安装方向、动力学标定和传感器偏置，
不要通过降低阈值或增加灵敏度来掩盖问题。

## 诊断

只连接并记录状态，不启动拖动：

```bash
python3 scripts/diagnose_freedrive_drift.py Rizon4s-123456 \
  --confirm-motion
```

程序会要求输入 `EXECUTE FREEDRIVE`，CSV 保存在 `output/`。

常见故障：

| 现象 | 处理 |
|---|---|
| 找不到脚踏板 | 运行 `xinput list --short`，确认 FootSwitch 是 XInput slave keyboard |
| `DISPLAY`/X11 错误 | 必须从当前本机 X11 桌面终端运行，不要使用纯 SSH |
| Permission denied | 默认 XInput 后端不会打开 `/dev/input`；确认没有传 `--foot-pedal-backend evdev` |
| 型号被拒绝 | 确认连接的是 Rizon 4S，机器人上报型号应为 Rizon4s |
| 软件版本被拒绝 | RDK 1.9.0 配套 Robot Software 3.11.x；不要继续使用旧的 1.7.0 |
| `ZeroFTSensor`/原语 license 错误 | 在 Elements 中检查 RDK 与 Primitive 许可 |
| 无接触自行漂移 | 再踩踏板停止，必要时急停；检查载荷、安装、动力学与传感器 |
| 最后关节难以拖动 | 确认终端显示 `ZeroFTSensor finished` 后再踩踏板 |

官方参考：

- [Flexiv RDK Free Drive](https://www.flexiv.com/software/rdk/manual/free_drive.html)
- [Flexiv RDK 1.9 RobotStates](https://www.flexiv.com/software/rdk/api/structflexiv_1_1rdk_1_1_robot_states.html)
- [Robot Software 3.11 FloatingCartesian](https://primitive.flexiv.com/primitives/en/3.11/rizon4/Zero%20Gravity%20Floating.html)
