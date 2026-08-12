# Flexiv 7/9-DoF Wrist Control

This workspace has one short entry point:

```bash
cd /home/src4/Flexiv-main/rizon4_tasks/chai3d_teleop
python run.py --help
```

Common commands:

```bash
python run.py ui
python run.py demo7
python run.py demo9
python run.py teleop
python run.py set-zero
python run.py go-zero
python run.py home
python run.py identify-inertia
python run.py dynamic-inertia
python run.py test
```

Read [RUNBOOK_ZH_EN.md](docs/RUNBOOK_ZH_EN.md) before real motion. Controller
equations, architecture, mode logic, stability changes, and limitations are in
[CONTROL_ARCHITECTURE_ZH_EN.md](docs/CONTROL_ARCHITECTURE_ZH_EN.md).

No code in this workspace is pushed automatically. Physical joint limits,
drive/robot faults, communication watchdogs, actuator ratings, and E-stop
behavior remain active safety boundaries.
