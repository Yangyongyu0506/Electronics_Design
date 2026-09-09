# slam_bringup (ROS 2 Humble)

Receives the ESP32 `slam-task` TCP stream (wheel odometry + N10 LiDAR scan)
and runs `slam_toolbox` for 2D LiDAR SLAM.

## Prerequisites (Ubuntu 22.04)

```sh
sudo apt install -y ros-humble-slam-toolbox ros-humble-rviz2 ros-humble-tf2-ros
```

## Build

```sh
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Power/flash the ESP32 first (`slam-task` firmware), it will keep trying to
connect to `PC_IP_ADDR:PC_PORT` (default `192.168.0.198:34567`).

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_bringup slam_bringup.launch.py use_rviz:=true
```

Options:

- `use_rviz:=true|false` — start RViz (default `false`).
- `laser_z:=0.15` — LiDAR height above `base_link` (m), used for the
  `base_link → laser_link` static TF.

## Verify (no robot, optional)

```sh
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_link   # after slam_toolbox starts
ros2 run tf2_ros tf2_echo base_link laser_link
```

## Save the map

After mapping, in a new terminal:

```sh
ros2 run nav2_map_server map_saver_cli -f ~/map
```

This writes `~/map.pgm` + `~/map.yaml`.

## Startup order

1. Flash + boot the ESP32 (LiDAR spinning, WiFi connected, TCP connected).
2. Build + source the workspace.
3. `ros2 launch slam_bringup slam_bringup.launch.py use_rviz:=true`.
4. Slowly push/drive the car around the area; the `/map` fills in RViz.
5. Save the map with `map_saver_cli`.
