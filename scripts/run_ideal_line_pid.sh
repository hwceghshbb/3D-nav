#!/usr/bin/env bash
set +u

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -euo pipefail

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_ideal_line_pid}"
mkdir -p "$ROS_LOG_DIR"
export DISPLAY="${DISPLAY:-:1}"

kp="${1:-0.10}"
ki="${2:-0.0}"
kd="${3:-0.05}"
duration="${IDEAL_LINE_DURATION:-25s}"
log_file="${IDEAL_LINE_LOG:-/tmp/ideal_line_pid.log}"

echo "ideal_line_pid: kp=$kp ki=$ki kd=$kd duration=$duration"
echo "log_file=$log_file"

timeout "$duration" ros2 launch bxi_example_py_elf3 example_launch_ideal_line_pid.py \
  line_pid_kp:="$kp" \
  line_pid_ki:="$ki" \
  line_pid_kd:="$kd" \
  error_sign:=-1.0 \
  line_max_yawdot:=0.10 \
  line_max_amp_input_yawdot:=0.10 \
  line_max_yawdot_rate:=0.50 \
  2>&1 | tee "$log_file"
