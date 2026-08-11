# Flexiv + 两轴腕部控制入口

本工程现有两条独立控制路径：

1. `scripts/run_9dof_teleop.py`：omega.7 + 脚踏板的三模式遥操作，Flexiv 使用 NRT
   笛卡尔运动/力控制，腕部通过 moteus 控制；
2. 独立的 7DoF/9DoF C++ 实时关节力矩 OSC 闭环 Demo。

完整实时控制、惯量辨识、编译、preflight、dry-run 和实机命令见：

- [`RT_TORQUE_OSC_ZH.md`](RT_TORQUE_OSC_ZH.md)

三模式遥操作控制策略的中英双语说明见：

- [`THREE_MODE_CONTROL_ZH_EN.md`](THREE_MODE_CONTROL_ZH_EN.md)

新 VS Code 终端中的权限、UI 与遥操作启动说明见：

- [`START_9DOF_TELEOP_VSCODE_ZH.md`](START_9DOF_TELEOP_VSCODE_ZH.md)

## 当前硬件模型

- 串联顺序：joint 8 / moteus ID2，然后 joint 9 / moteus ID1；
- joint 8：`±90°`，减速比 36；
- joint 9：`±180°`，减速比 30；
- 两轴的连续输出力矩配置为 `[6, 2] N·m`；
- 关节 8 轴到法兰面的距离为 112 mm；
- 当前零位探针 TCP 为 `[7.9, -31.1, 342.6] mm`；
- active Flexiv Tool 名称为 `Wrist`，必须与当前真实总成保持一致。

当前 Bota 传感器不参与任何运行、控制或配置。探针轴向力由 Flexiv endpoint force 结合
实时 `q8/q9` 姿态投影得到。

## 遥操作三模式

- Mode 1：Flexiv 7DoF 遥操作，腕部不跟随手柄；
- Mode 2：Flexiv + 两轴腕部协同 9DoF 位姿遥操作，优先使用腕部可实现的姿态方向；
- Mode 3：手柄只输入姿态，平移输入被丢弃；探针 Z 力由 Flexiv 内置力控制保持目标值。

模式选择由 UI 或脚踏板锁存；omega.7 clutch 仍是 dead-man。模式切换后必须先松开 clutch，
再按下以捕获新锚点。异常或 Ctrl-C 会停止 Flexiv、腕部并把触觉力归零。

## 触觉反馈稳定性

反馈路径现已加入：wrench 低通、渐入、矢量力变化率限制、本地速度阻尼以及时间域无源性
能量罐。这一层专门处理“开启力反馈后机械臂和手柄互相激发”的延迟闭环。参数位于
`config/nine_dof_teleop.toml` 的 `[force_feedback]`，UI 会自动显示并保存。

这仍必须由低增益、低接触力开始做现场验证。稳定层不会绕过 Flexiv 安全系统，也不会取消
omega.7 的连续额定力饱和。
