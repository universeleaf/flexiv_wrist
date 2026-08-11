# EmJ Force Capture UI

This UI reads the Rizon 4S state at 50 Hz and plots all six end-effector wrench
components. It is read-only and never starts, stops, or changes the EmJ project
on the Flexiv Elements tablet.

Run it with:

```bash
cd /path/to/Flexiv-main/rizon4_tasks/freedrive_python
./scripts/start_emj_monitor.sh
```

Use the UI at `http://127.0.0.1:8775`:

1. Wait for **Robot data online**.
2. Click **Start force capture**.
3. Start EmJ from Flexiv Elements on the LAN2 tablet.
4. When EmJ finishes, click **Stop & save CSV**.

Each run creates:

```text
$HOME/EmJ/EmJ_YYYYMMDD_HHMMSS_mmm_xxxx/
└── EmJ_YYYYMMDD_HHMMSS_mmm_xxxx_force_torque.csv
```

The CSV contains compensated TCP wrench in TCP and world frames, raw flange
F/T sensor readings, PC and robot timestamps, wrench norms, robot/plan state,
TCP pose, and all seven joint positions. **Display tare** affects only the live
plot; the CSV always contains the unmodified values received from RDK.
