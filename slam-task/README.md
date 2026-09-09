# slam-task

Wheel odometry + N10 LiDAR: the 3 omni-wheel encoders provide body-frame velocities via the inverse kinematics, integrated into world-frame `x/y` with the wheel-derived yaw (roll/pitch = 0). The N10 LiDAR is read over UART and its scans are streamed over the same WiFi/TCP link. An rclpy node on the PC republishes `/odom` and `/scan` (LaserScan, `base_footprint`).

## Wiring (repository root README)

- TB6612 motors + encoders: A=41/40, B=8/3, D=17/18 (same as `motor-test`); motors are braked at boot (duty 0, IN1=IN2=1).
- N10 lidar: TX→GPIO9, RX→GPIO10 (UART1 remapped to these pins via the GPIO matrix, 230400 8N1). The console stays on UART0 — monitor as usual.

## Firmware

- `odom_task` (100 Hz): PCNT encoders (512 counts/rev, wheel 5.5 cm, base 9 cm) → omni inverse kinematics → `yaw += wz·dt`, world-frame `x/y` integration; `z = 0`, yaw-only quaternion.
- `lidar_task`: parses the N10/N10_P protocol (header `A5 5A`, length byte 58 or 108, 16 points × 3 B from offset 7 — distance mm + intensity, start/end angle in 0.01°, CRC8 checksum). Points are accumulated and a complete 360° scan is handed to the net task on angle wrap.
- `net_task` (50 Hz): one TCP stream to `192.168.0.200:34567` (`PC_IP_ADDR`/`PC_PORT`) carrying:
  - odom (61 B): `<I magic=0x0D0D0001 Q stamp 3f x,y,z f yaw 4f qw..qz 3f vx,vy,vz f wz B seq>`
  - scan: `<I magic=0x0D0D0002 B seq H n  H[n] angle_mdeg  H[n] dist_mm  B[n] intensity>`
- WiFi STA (`iarc-thu5G` / `12345678` in `main/slam-task.c`), reconnect forever.
- 5 Hz log: `odom: x=.. y=.. yaw=.. | vx=.. vy=.. wz=..`

## PC translater node (ROS 2 Jazzy, rclpy)

```sh
source /opt/ros/jazzy/setup.bash
python3 host/translater_node.py
```

Publishes:
- `/odom` (`nav_msgs/Odometry`), frames `odom` → `base_footprint`
- `/scan` (`sensor_msgs/LaserScan`), frame **`base_footprint`** (720 slots, 0.5°, ranges/intensities projected by angle; `inf` where empty)

## Tuning knobs (`main/slam-task.c`)

- `LIDAR_BAUD` (230400): the older N10 (58-byte packets); use 460800 for N10_P.
- `WHEEL_A/B/D_SIGN`: flip a wheel's encoder sign if pushing the car forward decreases x.
- `COUNTS_PER_REV` (512), `WHEEL_DIAMETER_CM` (5.5), `WHEEL_BASE_CM` (9.0): odometry constants.

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor   # monitor attaches over USB-JTAG
```
