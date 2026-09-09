#!/usr/bin/env bash
# micro-ROS agent for the ESP32 car (Ubuntu 22.04 / ROS 2 Humble).
# UDP transport on the port configured in imu-test/sdkconfig.defaults
# (CONFIG_MICRO_ROS_AGENT_PORT, default 8888).
set -euo pipefail

PORT=${PORT:-8888}

if ! command -v docker >/dev/null 2>&1; then
    echo "docker not found. Install it first:" >&2
    echo "  https://docs.docker.com/engine/install/ubuntu/" >&2
    exit 1
fi

echo "Starting micro-ROS agent (UDP port ${PORT})..."
echo "The ESP32 must send to this machine's LAN IP (CONFIG_MICRO_ROS_AGENT_IP)."
echo "This machine's IP candidates:" 
hostname -I

exec docker run -it --rm --net=host \
    microros/micro-ros-agent:humble \
    udp4 --port "${PORT}" -v6
