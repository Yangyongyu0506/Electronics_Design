#!/usr/bin/env python3
"""Translater node: receives wheel odometry and LiDAR scan data from the ESP32
car over one TCP connection and republishes them as standard ROS 2 topics:

- /odom (nav_msgs/Odometry), frame odom -> base_link
- /scan (sensor_msgs/LaserScan), frame laser_link
- tf odom -> base_link (tf2_ros TransformBroadcaster)

TCP stream (little-endian):
  msg 1 (odom, 61 B): <I magic=0x0D0D0001 Q stamp_us 12f px,py,pz,yaw,qw,qx,
                       qy,qz,vx,vy,vz,wz B seq>
  msg 2 (scan):       <I magic=0x0D0D0002 B seq H n_points
                       H[n] angle_mdeg  H[n] dist_mm  B[n] intensity>

Header stamps use the PC's ROS wall clock (not the ESP32's boot-relative
stamp_us) so /odom and /scan share one clock domain for tf2 / SLAM Toolbox
lookups; stamp_us is kept only for dropped-packet diagnostics.

Usage (Ubuntu 22.04 / ROS 2 Humble):
    ros2 run slam_bringup translater_node
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

ODOM_MAGIC = 0x0D0D0001
SCAN_MAGIC = 0x0D0D0002
LISTEN_PORT = 34567
ODOM_STRUCT = struct.Struct("<IQ12fB")
ODOM_BYTES = ODOM_STRUCT.size
SCAN_SLOTS = 720
RANGE_MIN = 0.15
RANGE_MAX = 30.0

(
    I_PX, I_PY, I_PZ,
    I_YAW,
    I_QW, I_QX, I_QY, I_QZ,
    I_VX, I_VY, I_VZ,
    I_WZ,
) = range(12)


class TranslaterNode(Node):
    def __init__(self):
        super().__init__("wheel_odom_translater")
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.lock = threading.Lock()
        self.odom_latest = None
        self.scan_latest = None
        self.last_seq = None
        self.publish_timer = self.create_timer(0.02, self.publish_latest)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", LISTEN_PORT))
        self.server.listen(1)
        self.get_logger().info(f"listening on TCP 0.0.0.0:{LISTEN_PORT} "
                               "for ESP32 odometry + lidar stream")
        threading.Thread(target=self.accept_loop, daemon=True).start()

    def accept_loop(self):
        while rclpy.ok():
            try:
                conn, addr = self.server.accept()
            except OSError:
                return
            self.get_logger().info(f"ESP32 connected from {addr[0]}:{addr[1]}")
            threading.Thread(target=self.recv_loop, args=(conn,),
                             daemon=True).start()

    def recv_loop(self, conn):
        buf = b""
        conn.settimeout(5)
        try:
            while rclpy.ok():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 4:
                    magic = struct.unpack("<I", buf[:4])[0]
                    if magic == ODOM_MAGIC:
                        if len(buf) < ODOM_BYTES:
                            break
                        data, buf = buf[:ODOM_BYTES], buf[ODOM_BYTES:]
                        _, stamp_us, *vals, seq = ODOM_STRUCT.unpack(data)
                        if self.last_seq is not None:
                            dropped = (seq - self.last_seq - 1) & 0xFF
                            if dropped:
                                self.get_logger().debug(
                                    f"dropped {dropped} odom packets")
                        self.last_seq = seq
                        with self.lock:
                            self.odom_latest = vals
                    elif magic == SCAN_MAGIC:
                        if len(buf) < 7:
                            break
                        seq = buf[4]
                        n = buf[5] | (buf[6] << 8)
                        need = 7 + 5 * n
                        if len(buf) < need:
                            break
                        data, buf = buf[:need], buf[need:]
                        angles = struct.unpack(f"<{n}H", data[7:7 + 2 * n])
                        dists = struct.unpack(f"<{n}H", data[7 + 2 * n:7 + 4 * n])
                        inten = data[7 + 4 * n:7 + 5 * n]
                        with self.lock:
                            self.scan_latest = (seq, angles, dists, inten)
                    else:
                        self.get_logger().warn("bad magic; resyncing stream")
                        buf = buf[1:]
        except OSError as exc:
            self.get_logger().warn(f"socket error: {exc}")
        finally:
            self.get_logger().info("ESP32 disconnected")
            conn.close()

    def publish_latest(self):
        now = self.get_clock().now().to_msg()

        with self.lock:
            odom = self.odom_latest
            scan = self.scan_latest

        if odom is not None:
            v = odom
            msg = Odometry()
            msg.header.stamp = now
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = float(v[I_PX])
            msg.pose.pose.position.y = float(v[I_PY])
            msg.pose.pose.position.z = float(v[I_PZ])
            msg.pose.pose.orientation.w = float(v[I_QW])
            msg.pose.pose.orientation.x = float(v[I_QX])
            msg.pose.pose.orientation.y = float(v[I_QY])
            msg.pose.pose.orientation.z = float(v[I_QZ])
            msg.twist.twist.linear.x = float(v[I_VX])
            msg.twist.twist.linear.y = float(v[I_VY])
            msg.twist.twist.linear.z = float(v[I_VZ])
            msg.twist.twist.angular.z = float(v[I_WZ])
            self.odom_pub.publish(msg)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        if scan is not None:
            _seq, angles, dists, inten = scan
            ranges = [float("inf")] * SCAN_SLOTS
            intensities = [float("nan")] * SCAN_SLOTS
            for a, d, i in zip(angles, dists, inten):
                if d == 0xFFFF:
                    continue
                dist = d / 1000.0
                if dist < RANGE_MIN or dist > RANGE_MAX:
                    continue
                slot = int(round(a * SCAN_SLOTS / 36000.0)) % SCAN_SLOTS
                ranges[slot] = dist
                intensities[slot] = float(i)
            msg = LaserScan()
            msg.header.stamp = now
            msg.header.frame_id = "laser_link"
            msg.angle_min = 0.0
            msg.angle_max = 6.283185307179586
            msg.angle_increment = 6.283185307179586 / SCAN_SLOTS
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = RANGE_MIN
            msg.range_max = RANGE_MAX
            msg.ranges = ranges
            msg.intensities = intensities
            self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = TranslaterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
