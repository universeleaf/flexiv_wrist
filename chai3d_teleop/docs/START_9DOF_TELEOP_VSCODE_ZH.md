# 新 VS Code 窗口中的完整启动命令

本文针对本机路径与用户 `src4`。当前程序不使用外置 Bota。

## 1. 新登录会话先检查组权限

```bash
id
groups
ls -l /dev/serial/by-id/usb-mjbots_fdcanusb_YOUR_SERIAL-if00
readlink -f /dev/serial/by-id/usb-mjbots_fdcanusb_YOUR_SERIAL-if00
```

`groups` 必须包含 `dialout`；实时力矩 Demo 还必须包含 `realtime`。如果已经执行过
`usermod`，但当前 VS Code 中看不到新组，完全退出 VS Code、注销 Ubuntu 用户并重新登录。
`newgrp dialout` 只适合当前终端临时刷新。

## 2. Python 环境分工

不要先激活虚拟环境，直接使用完整路径：

- `../freedrive_python/.venv/bin/python`：NumPy、Flexiv Python RDK、主启动器；
- `.venv_moteus/bin/python`：moteus bridge，由主程序自动启动；
- `build_rt_osc/*`：官方 Flexiv C++ RDK 的 1 kHz torque OSC。

不要用 `.venv_moteus/bin/python` 运行主协调器，也不要用 `sudo python` 启动机器人程序。

## 3. 普通三模式遥操作

### 3.1 命令行启动

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_teleop.py --check-config

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_teleop.py
```

首次只验证模式 3 的受力方向时：

```bash
../freedrive_python/.venv/bin/python \
  scripts/run_9dof_teleop.py --mode3-force-n -2
```

### 3.2 UI 启动

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop
python3 scripts/teleop_control_ui.py
```

访问 `http://127.0.0.1:8765/`。UI 只监听本机，提供：

- 编辑并按原类型验证 `config/nine_dof_teleop.toml` 的全部字段；
- Save Configuration 和 Save & Start；
- 实时日志、Ctrl-C Stop；
- Mode 1/2/3 按钮，与实体脚踏板进入同一状态机。

启动前松开 omega.7 clutch 和脚踏板。模式切换后必须松开再按 clutch。关闭浏览器标签不会
停止机器人；使用 Stop 按钮或 UI 终端 Ctrl-C。

端口被占用时：

```bash
python3 scripts/teleop_control_ui.py --port 8766
```

## 4. 腕部只读通信检查

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop

.venv_moteus/bin/python -m moteus.moteus_tool \
  --fdcanusb /dev/serial/by-id/usb-mjbots_fdcanusb_YOUR_SERIAL-if00 \
  --no-tel-stop --info
```

应看到 can ID 1 和 2。若提示端口 multiple access/disconnected，关闭所有占用 fdcanusb 的
Python/moteus 进程，重新插入适配器后再检查；不要同时启动两个腕部程序。

## 5. 模式行为

- Mode 1：ID2/joint 8 回配置零位，ID1/joint 9 保持 STOP；Flexiv 执行 7DoF 位姿遥操作。
- Mode 2：Flexiv 和两轴腕部共同执行 9DoF 位姿遥操作，优先使用腕部的两个姿态方向。
- Mode 3：手柄平移被丢弃，只有手柄旋转改变探针姿态；X/Y 保持，Flexiv 的探针 Z 力轴
  保持配置力。探针 Z 可因力控制产生小位移。

探针 Z 力直接由 Flexiv endpoint force 按实时法兰/q8/q9 姿态投影。没有外置传感器启动、
tare 或网卡权限步骤。

## 6. 实时 7DoF/9DoF torque OSC

详细原理、实时组配置、腕部惯量辨识和风险说明见
[`RT_TORQUE_OSC_ZH.md`](RT_TORQUE_OSC_ZH.md)。每个新窗口常用命令如下。

### 6.1 编译与 preflight

```bash
cd /path/to/Flexiv-main/rizon4_tasks/chai3d_teleop
bash scripts/build_rt_osc.sh

../freedrive_python/.venv/bin/python \
  scripts/run_7dof_torque_osc_demo.py --preflight

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_torque_osc_demo.py --preflight
```

### 6.2 dry-run，不连接硬件

```bash
../freedrive_python/.venv/bin/python \
  scripts/run_7dof_torque_osc_demo.py --duration-s 15

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_torque_osc_demo.py --duration-s 15
```

### 6.3 实机低幅测试

9DoF 实机前必须先按实时文档生成 `config/wrist_inertia_calibration.json`。

```bash
../freedrive_python/.venv/bin/python \
  scripts/run_7dof_torque_osc_demo.py \
  --duration-s 15 --radius-m 0.005 --orientation-deg 3 \
  --real --confirm RUN_7DOF_TORQUE_OSC

../freedrive_python/.venv/bin/python \
  scripts/run_9dof_torque_osc_demo.py \
  --duration-s 15 --radius-m 0.005 --orientation-deg 3 \
  --real --confirm RUN_9DOF_TORQUE_OSC
```

两条实机命令必须分开运行。清空工作空间、确认 active Tool、握住急停，先跑 5 mm/3°。
不要把普通遥操作和 torque OSC 同时连接到同一台 Flexiv 或同一个 fdcanusb。

## 7. 常见错误

### fdcanusb Permission denied

```bash
getent group dialout
id
ls -l "$(readlink -f /dev/serial/by-id/usb-mjbots_fdcanusb_YOUR_SERIAL-if00)"
```

组数据库包含 `src4` 但 `id` 没有 `dialout`，表示登录会话未刷新；注销并重新登录。

### `ModuleNotFoundError: flexivrdk`

主脚本使用了错误解释器。改为：

```bash
../freedrive_python/.venv/bin/python scripts/run_9dof_teleop.py
```

### 实时 preflight 提示 `RLIMIT_RTPRIO<90`

按 `RT_TORQUE_OSC_ZH.md` 加入 `realtime` 组和 limits 文件，然后完整注销重登。

### `wrist bridge` 通信 timeout / multiple access

说明适配器断开、CAN 供电/终端电阻异常，或另一个进程正在读同一串口。停止所有控制脚本，
检查 USB 与 24 V/CAN，总线恢复后先运行第 4 节只读检查。
