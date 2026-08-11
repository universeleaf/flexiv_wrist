# Flexiv Wrist

Research code for a Flexiv Rizon 4s equipped with a two-axis moteus wrist and
an omega.7 CHAI3D teleoperator.

The repository contains two sibling projects because the teleoperation
coordinator reuses the tested foot-pedal input package:

- [`chai3d_teleop/`](chai3d_teleop/README.md): CHAI3D device bridge, 7/9-DoF
  teleoperation, three-mode UI, wrist calibration, and real-time torque/OSC
  demonstrations.
- [`freedrive_python/`](freedrive_python/docs/INSTALL_AND_OPERATE.md): Flexiv
  Python environment and foot-pedal support used by the teleoperation scripts.

Start with the Chinese real-time control guide:
[`chai3d_teleop/docs/RT_TORQUE_OSC_ZH.md`](chai3d_teleop/docs/RT_TORQUE_OSC_ZH.md).
The three teleoperation modes are documented in Chinese and English at
[`chai3d_teleop/docs/THREE_MODE_CONTROL_ZH_EN.md`](chai3d_teleop/docs/THREE_MODE_CONTROL_ZH_EN.md).

## External dependencies

Builds require separate checkouts/installations of:

- [manips-sai-org/chai3d](https://github.com/manips-sai-org/chai3d)
- Flexiv RDK v1.9
- `moteus` in `chai3d_teleop/.venv_moteus`
- the Python dependencies in `freedrive_python/requirements.txt`

The checked-in TOML/YAML files preserve the configuration structure and
calibrated wrist model, but public copies use placeholder robot, network, and
USB identifiers. Fill in the local deployment values before running them.
Real-time torque control can move physical hardware; use the documented
low-amplitude checks first and keep the emergency stop immediately accessible.
