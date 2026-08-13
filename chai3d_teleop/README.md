# Flexiv 7/9-DoF Wrist Control

This workspace has one short entry point:

```bash
cd /home/src4/Flexiv-main/rizon4_tasks/chai3d_teleop
python3 run.py --help
```

Common commands:

```bash
python3 run.py ui
python3 run.py demo7
python3 run.py demo9
python3 run.py teleop
python3 run.py set-zero
python3 run.py go-zero
python3 run.py home
python3 run.py identify-inertia
python3 run.py dynamic-inertia
python3 run.py test
```

`home` uses live FK/IK to move directly to the modeled Flexiv Home TCP plus
0.20 m world Z; it does not visit the low official Home point. The long Tool/TCP
must be correctly selected in Elements and the complete sweep must be clear.

Read [RUNBOOK_ZH_EN.md](docs/RUNBOOK_ZH_EN.md) before real motion. Controller
equations, architecture, mode logic, stability changes, and limitations are in
[CONTROL_ARCHITECTURE_ZH_EN.md](docs/CONTROL_ARCHITECTURE_ZH_EN.md).
The detailed bilingual technical reference (Chinese first, then English) for
the Flexiv and custom OSC controllers, gravity/inertia treatment, calibration,
multi-rate communication, mode state machines, tuning, UI signals, and
diagnostics is in
[TECHNICAL_CONTROL_AND_COMMUNICATION_ZH.md](docs/TECHNICAL_CONTROL_AND_COMMUNICATION_ZH.md).

No code in this workspace is pushed automatically. Physical joint limits,
drive/robot faults, communication watchdogs, actuator ratings, and E-stop
behavior remain active safety boundaries.
