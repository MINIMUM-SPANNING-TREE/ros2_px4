#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

unset PYTHONPATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_EXE

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/../install/setup.bash"

ros2 launch lslidar_driver lslidar_n10_uart_launch.py &

wait
