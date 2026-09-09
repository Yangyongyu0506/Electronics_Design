# slam-task

Wheel odometry + N10 LiDAR, streamed over WiFi/TCP to a PC that runs ROS 2
Humble + `slam_toolbox` for 2D LiDAR SLAM. The ESP32-S3 only acquires sensor
data (LiDAR, wheel encoders) and streams it; all SLAM (scan matching, loop
closure, map management) runs on the PC.

## Wiring (repository root README)

- TB6612 motors + encoders: A=41/40, B=8/3, D=17/18 (same as `motor-test`);
  motors are braked at boot (duty 0, IN1=IN2=1).
- N10 lidar: TX→GPIO9, RX→GPIO10 (UART1 remapped to these pins via the GPIO
  matrix, 230400 8N1). The console stays on UART0 — monitor as usual.

## Firmware (`main/slam-task.c`)

- `odom_task` (100 Hz): PCNT encoders (512 counts/rev, wheel 5.5 cm, base
  9 cm) → omni inverse kinematics → `yaw += wz·dt`, world-frame `x/y`
  integration; `z = 0`, yaw-only quaternion.
- `lidar_task`: parses the N10/N10_P protocol (header `A5 5A`, length byte 58
  or 108, 16 points × 3 B from offset 7 — distance mm + intensity, start/end
  angle in 0.01°, CRC8 checksum). Points are accumulated and a complete 360°
  scan is handed to the net task on angle wrap.
- `net_task` (50 Hz): one TCP stream to `192.168.0.198:34567`
  (`PC_IP_ADDR`/`PC_PORT`) carrying:
  - odom (61 B): `<I magic=0x0D0D0001 Q stamp 3f x,y,z f yaw 4f qw..qz
    3f vx,vy,vz f wz B seq>`
  - scan: `<I magic=0x0D0D0002 B seq H n  H[n] angle_mdeg  H[n] dist_mm
    B[n] intensity>`
- WiFi STA (`iarc-thu5G` / `12345678` in `main/slam-task.c`), reconnect forever.
- 5 Hz log: `odom: x=.. y=.. yaw=.. | vx=.. vy=.. wz=..`

## ROS 2 SLAM (PC, `ros2_ws/`)

A minimal `slam_bringup` package (ament_python) receives the TCP stream and
runs `slam_toolbox`. TF tree: `map → odom` (slam_toolbox), `odom → base_link`
(translater from `/odom`), `base_link → laser_link` (static).

The translater also subscribes `/cmd_vel` (m/s, m/s, rad/s) and forwards each
command to the ESP32 as a 17-byte packet `<I magic=0x0D0D0003 B seq 3f vx,vy,wz>`.
The ESP32 drives the 3 wheels with the motor-test PID speed loop; a 500 ms
command watchdog stops the car.

See `ros2_ws/README.md` for build, launch and map-saving steps.

## Tuning knobs (`main/slam-task.c`)

- `LIDAR_BAUD` (230400): the older N10 (58-byte packets); use 460800 for
  N10_P.
- `WHEEL_A/B/D_SIGN`: flip a wheel's encoder sign if pushing the car forward
  decreases x.
- `COUNTS_PER_REV` (512), `WHEEL_DIAMETER_CM` (5.5), `WHEEL_BASE_CM` (9.0):
  odometry constants.
- `ODOM_SCALE` (0.01): cm-to-metre conversion for the odometry (encoder distance is in cm). The motor PID targets use counts directly (512 counts/rev is correct), so no scale applies there.
- `CMD_TIMEOUT_MS` (500): stop the car when cmd_vel stops arriving.

## Build & flash (ESP32)

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor   # monitor attaches over USB-JTAG
```
