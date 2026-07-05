#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

unset PYTHONPATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_EXE
export PATH="/usr/bin:/usr/local/bin:/bin:$PATH"

BACKEND_IP="${BACKEND_IP:-192.168.10.59}"
BACKEND_URL="ws://${BACKEND_IP}:9200/ws/bridge"
BOARD_URL="ws://${BACKEND_IP}:9200/ws/board"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/../install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run uav_mavros2 telemetry_node &
sleep 3
ros2 launch uav_mavros2 uav_launch.py &
sleep 3


cd "${SCRIPT_DIR}/../bridge"
/usr/bin/python3 main.py --url "${BACKEND_URL}" &
sleep 1

cd "${SCRIPT_DIR}/../client"
/usr/bin/python3 main.py --url "${BOARD_URL}" &

wait
