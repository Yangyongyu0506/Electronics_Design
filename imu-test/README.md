# imu-test

MPU6500 IMU reading + attitude estimation + network streaming. Fuses gyro (Mahony complementary filter, no magnetometer) and accelerometer into a quaternion, prints RPY angles and the double-integrated position, and streams the raw state over WiFi/TCP to the PC, where an rclpy node republishes it as standard ROS 2 topics.

## Wiring (repository root README, `MPU6500` section)

| MPU6500 | Board |
|---------|-------|
| SCL     | GPIO21 |
| SDA     | GPIO47 |
| VCC     | 3V3 |
| GND     | GND |
| AD0     | floating (I2C addr 0x68) |
| INT     | floating |

Motors: all three TB6612 channels are braked at boot (PWM duty 0 + IN1=IN2=1) so the wheels stay at zero speed.

## Firmware output

- 5 Hz log: `rpy(deg): R=.. P=.. Y=.. | pos(m): x=.. y=.. | gz=.. dps | va=.. vb=.. vd=.. cm/s`
- ROS2 convention: x forward, y left, z up; RPY = roll/pitch/yaw. The MPU6500 axes match ROS2 (verified: RPY signs confirmed by synthetic tests).
- **Attitude**: Mahony complementary filter on the raw IMU only (gyro bias calibrated for 5 s at boot — keep the car still). Roll/pitch come straight from the filter quaternion.
- **Encoder sign calibration**: right after boot, rotate the car **in place CCW for 3 s** (the log prompts you). Each wheel's encoder sign is flipped automatically if its counts disagree with the gyro-z rotation direction — no manual tuning.
- **Odometry pose = gyro + 3 encoders**: the yaw rate `wz` comes from the **gyro** (wheel-sum noise cannot inject translation), and `vx/vy` come from the three wheel speeds with the rotation component subtracted (`va' = va − R·gz` etc., exact inverse of `set_speed`). Rotating in place therefore produces almost no translation.
- **Tilt freeze**: while |roll| or |pitch| exceeds 15° (`TILT_FREEZE_RAD`) the heading/position are frozen — pitching the car by hand only pitches the pose arrow in place and does not move the odometry.
- The published quaternion is composed as `Rz(heading)·Ry(pitch)·Rx(roll)` (ZYX).
- WiFi STA (`iarc-thu5G` / `12345678`, in `main/imu-test.c` `WIFI_SSID`/`WIFI_PASSWORD`), reconnect forever.
- TCP client streams a fixed 89-byte little-endian packet at 50 Hz to `192.168.0.200:34567` (`PC_IP_ADDR`/`PC_PORT`):

```
<I magic(0x0D0D0001) Q stamp_us 19f r,p,y q0..q3 px,py,pz vx,vy,vz gx,gy,gz ax,ay,az B seq>
```

(`px/py` = encoder odometry, `pz` = 0, `vx/vy` = body-frame wheel velocities; `gx/gy/gz` = gyro, `ax/ay/az` = raw accelerometer.)

## PC translater node (ROS 2 Jazzy, rclpy)

```sh
source /opt/ros/jazzy/setup.bash
python3 host/translater_node.py
```

Listens on TCP 34567, parses the packets and publishes:
- `/imu` (`sensor_msgs/Imu`): orientation quaternion, angular velocity (gyro), linear acceleration
- `/odom` (`nav_msgs/Odometry`): integrated position + quaternion, linear velocity + angular velocity; frames `odom` → `base_footprint`

Verify with `ros2 topic echo /odom` / `ros2 topic echo /imu`.

## Tuning knobs (`main/imu-test.c`)

- `ACC_AX/AY/AZ_SIGN`, `GYR_GX/GY/GZ_SIGN`: sign flip per axis if the module is mounted rotated on the car. Sanity check: pitch should rise when the car nose is tilted up.
- `TILT_FREEZE_RAD` (0.2618 = 15°): beyond this tilt the odometry is frozen (hand-holding the car).
- `SIGN_CAL_TIME_MS` (3000), `SIGN_CAL_MIN_GZ` (0.8): encoder sign calibration window and minimum total rotation.
- `COUNTS_PER_REV` (512), `WHEEL_DIAMETER_CM` (5.5), `WHEEL_BASE_CM` (9.0): encoder odometry constants.
- `IMU_PERIOD_MS` (10): loop rate; `NET_PERIOD_MS` (20): stream rate.
- Ranges/scales: gyro ±250 dps, accel ±2 g, DLPF 41 Hz. WHO_AM_I 0x68 is also accepted (MPU6050-compatible clones use the same register set).

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor
```
