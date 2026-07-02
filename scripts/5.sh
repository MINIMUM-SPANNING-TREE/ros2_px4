#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

unset PYTHONPATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_EXE
export PATH="/usr/bin:/usr/local/bin:/bin:$PATH"
export PYTHONPATH="${SCRIPT_DIR}/..:$PYTHONPATH"

source ~/mavros2_ws/install/setup.bash
source "${SCRIPT_DIR}/../install/setup.bash"
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM1:57600
sleep 3
ros2 launch uav_mavros2 uav_launch.py &
sleep 3
cd "${SCRIPT_DIR}/../bridge"
/usr/bin/python3 main.py --url "${BACKEND_URL:-ws://localhost:9200/ws/bridge}" &
sleep 1

cd "${SCRIPT_DIR}/../client"
/usr/bin/python3 main.py &

wait
