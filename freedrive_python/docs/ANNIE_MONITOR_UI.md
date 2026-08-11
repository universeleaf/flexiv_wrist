# Annie Camera and End-of-Arm 6-Axis Force Monitor

This local UI displays:

- A live Google Pixel USB/UVC camera feed.
- Rizon 4S end-of-arm force and moment values.
- Current Flexiv mode, Busy state, and connection status.
- Three-axis force and three-axis moment plots over a 10, 20, or 60-second window.

The monitor is strictly read-only. It never calls `Enable()`, `Stop()`,
`SwitchMode()`, `ExecutePlan()`, `ExecutePrimitive()`, or any other robot
command. Annie remains under the control of Flexiv Elements on the LAN2 tablet,
while the PC reads RDK 1.9 robot states through LAN1.

## Preview the UI with Demo Data

```bash
cd /path/to/Flexiv-main/rizon4_tasks/freedrive_python
./scripts/start_annie_monitor.sh --demo
```

Chrome should open `http://127.0.0.1:8765`. Demo mode uses synthetic force
signals and does not connect to the robot.

## Connect the Pixel Phone

1. Connect the Pixel to the PC with a USB-C cable that supports data.
2. On the Pixel, tap the **Charging this device through USB** notification.
3. Under **Use USB for**, select **Webcam**.
4. Optionally confirm that Linux detects the UVC camera:

   ```bash
   v4l2-ctl --list-devices
   ```

5. Open the UI, click **Connect camera**, and allow `127.0.0.1` to use the
   camera in Chrome.
6. If the PC has another camera, select the device whose name contains
   Pixel, Android, or Webcam.

The browser reads the Pixel camera directly. Python does not require OpenCV,
and this application does not change `/dev/video*` permissions. Open the page
in Chrome on the same PC that runs the monitor.

## Connect the Robot and Record Annie

First verify that PC LAN1 still has `127.0.0.1`:

```bash
ip -brief -4 address
```

Then run:

```bash
cd /path/to/Flexiv-main/rizon4_tasks/freedrive_python
./scripts/start_annie_monitor.sh
```

Recording workflow:

1. Wait for **Robot data online** in the upper-right corner.
2. Connect the Pixel and verify that live video is visible.
3. Click **Start synchronized capture**. There is no screen-sharing dialog.
4. Run project Annie from Flexiv Elements on the LAN2 tablet.
5. When Annie finishes, return to the UI and click **Stop & save**. Keep the
   page and terminal open while `FINALIZING` is shown; this is when the MP4 is
   converted and the camera images are packed.

Each recording is saved directly under
`$HOME/Annie/Annie_YYYYMMDD_HHMMSS_mmm_xxxx/` with three artifacts:

- `*_force_torque.csv`: Force, moment, TCP pose, joint position, PC timestamp,
  robot timestamp, and the exact matching camera filename.
- `*_camera_frames.zip`: Camera-only JPEG images captured once per real Pixel
  video frame. The `camera_frame` CSV column points each force row to the most
  recent camera image, so several 50 Hz force rows can reference one 30 FPS
  camera frame.
- `*_demo.mp4`: Full generated Annie view containing the Pixel camera, current
  force/moment values, plots, robot state, and synchronized clock. It does not
  contain the desktop, browser/player controls, download bar, or the
  `Annie_..._demo.mp4` filename as an in-video label.

Image names use both an increasing index and the camera capture PC timestamp,
for example `frame_000000123_1786044321123456789.jpg`. The CSV also records
`camera_capture_unix_ms` and `camera_delay_from_force_ms`, so alignment is
explicit. The force stream is nominally 50 Hz while the Pixel is normally 30
FPS. Unique camera images are therefore stored at about 30 FPS without
duplicating JPEG files; the force CSV remains at 50 Hz.

The CSV preserves all three wrench sources:

- `tcp_*`: Compensated external wrench applied at the TCP, expressed in the
  TCP frame.
- `world_*`: Compensated external wrench applied at the TCP, expressed in the
  world frame.
- `sensor_*`: Raw end-of-arm F/T sensor measurement in the flange frame.

**Display tare** subtracts the current reading from the plot to make changes
easier to see. It does not modify the robot and does not alter the raw values
saved in the CSV.

## Troubleshooting

| Symptom | Check |
|---|---|
| UI does not open | Keep the terminal running and manually open `http://127.0.0.1:8765` |
| Force connection fails | Check LAN1, PC address `127.0.0.1`, Robot Software v3.11, and RDK 1.9 |
| Tablet runs but the PC receives no data | Confirm PC is on LAN1, tablet is on LAN2, and no old RDK process is still running |
| Chrome cannot see the Pixel | Confirm the cable supports data, the Pixel USB mode is Webcam, and run `v4l2-ctl --list-devices` |
| `lsusb` shows `18d1:4ee1 ... (MTP)` | The cable works, but the Pixel is in File transfer/MTP mode. On the Pixel, change **Use USB for** to **Webcam** |
| Saving fails while `FINALIZING` | Keep the terminal open and inspect its error plus the partial timestamped directory in `$HOME/Annie`; the monitor uses direct browser MP4 when available and this PC's GStreamer otherwise |
| Raw Fz includes tool weight | Select the compensated TCP wrench; raw flange sensor readings can include payload and bias |

Official references:

- Google Pixel Webcam:
  <https://support.google.com/pixelcamera/answer/14274129>
- Flexiv RDK v1.9, compatible with Software Package v3.11:
  <https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v1.9>
- Flexiv `RobotStates`:
  <https://www.flexiv.com/software/rdk/api/structflexiv_1_1rdk_1_1_robot_states.html>
a
