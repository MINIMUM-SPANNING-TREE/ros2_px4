#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

unset PYTHONPATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_EXE

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/../install/setup.bash"

ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:57600
# ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

# 传感器和算法
# ros2 launch lslidar_driver lslidar_n10_uart_launch.py &
# ros2 launch camera uav_system.launch.py &
# ros2 launch vision vision_tracking_system.launch.py &
# 导航
# ros2 launch navigation navigation.launch.py &



wait
