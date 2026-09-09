#!/usr/bin/env python3
"""Translater node: receives IMU + odometry packets from the ESP32 car over TCP
and republishes them as standard ROS 2 topics (/imu sensor_msgs/Imu and
/odom nav_msgs/Odometry).

Binary packet layout (little-endian, packed struct, 89 bytes):
    <I  magic     0x0D0D0001
     Q  stamp_us
     19f r,p,y, q0,q1,q2,q3, px,py,pz, vx,vy,vz, gx,gy,gz, ax,ay,az
     B   seq

Usage (Ubuntu 24.04 / ROS 2 Jazzy):
    source /opt/ros/jazzy/setup.bash
    python3 translater_node.py
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

PACKET = struct.Struct("<IQ19fB")
MAGIC = 0x0D0D0001
LISTEN_PORT = 34567

(
    I_ROLL, I_PITCH, I_YAW,
    I_Q0, I_Q1, I_Q2, I_Q3,
    I_PX, I_PY, I_PZ,
    I_VX, I_VY, I_VZ,
    I_GX, I_GY, I_GZ,
    I_AX, I_AY, I_AZ,
) = range(19)


class TranslaterNode(Node):
    def __init__(self):
        super().__init__("imu_odom_translater")
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.lock = threading.Lock()
        self.latest = None
        self.last_seq = None
        self.publish_timer = self.create_timer(0.02, self.publish_latest)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", LISTEN_PORT))
        self.server.listen(1)
        self.get_logger().info(f"listening on TCP 0.0.0.0:{LISTEN_PORT} "
                               "for ESP32 imu/odometry stream")
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
                while len(buf) >= PACKET.size:
                    data, buf = buf[:PACKET.size], buf[PACKET.size:]
                    magic, stamp, *vals, seq = PACKET.unpack(data)
                    if magic != MAGIC:
                        self.get_logger().warn("bad magic; resyncing stream")
                        buf = b""
                        break
                    if self.last_seq is not None:
                        dropped = (seq - self.last_seq - 1) & 0xFF
                        if dropped:
                            self.get_logger().debug(f"dropped {dropped} packets")
                    self.last_seq = seq
                    with self.lock:
                        self.latest = (stamp, vals)
        except OSError as exc:
            self.get_logger().warn(f"socket error: {exc}")
        finally:
            self.get_logger().info("ESP32 disconnected")
            conn.close()

    def publish_latest(self):
        with self.lock:
            latest = self.latest
        if latest is None:
            return
        stamp_us, v = latest
        stamp = rclpy.time.Time(nanoseconds=stamp_us * 1000).to_msg()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = float(v[I_Q0])
        imu.orientation.x = float(v[I_Q1])
        imu.orientation.y = float(v[I_Q2])
        imu.orientation.z = float(v[I_Q3])
        imu.angular_velocity.x = float(v[I_GX])
        imu.angular_velocity.y = float(v[I_GY])
        imu.angular_velocity.z = float(v[I_GZ])
        imu.linear_acceleration.x = float(v[I_AX])
        imu.linear_acceleration.y = float(v[I_AY])
        imu.linear_acceleration.z = float(v[I_AZ])
        self.imu_pub.publish(imu)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = float(v[I_PX])
        odom.pose.pose.position.y = float(v[I_PY])
        odom.pose.pose.position.z = float(v[I_PZ])
        odom.pose.pose.orientation.w = float(v[I_Q0])
        odom.pose.pose.orientation.x = float(v[I_Q1])
        odom.pose.pose.orientation.y = float(v[I_Q2])
        odom.pose.pose.orientation.z = float(v[I_Q3])
        odom.twist.twist.linear.x = float(v[I_VX])
        odom.twist.twist.linear.y = float(v[I_VY])
        odom.twist.twist.linear.z = float(v[I_VZ])
        odom.twist.twist.angular.x = float(v[I_GX])
        odom.twist.twist.angular.y = float(v[I_GY])
        odom.twist.twist.angular.z = float(v[I_GZ])
        self.odom_pub.publish(odom)


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
