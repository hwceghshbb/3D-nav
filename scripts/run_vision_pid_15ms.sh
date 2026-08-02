#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export DISPLAY="${DISPLAY:-:1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log_vision_pid_15ms}"
mkdir -p "$ROS_LOG_DIR"

LOG_FILE="${1:-/tmp/vision_pid_15ms_run.log}"

echo "log_file=$LOG_FILE"
echo "ros_log_dir=$ROS_LOG_DIR"
echo "mode=AMP running, forward=1.5m/s, lateral=0, visual=PID center offset"

exec ros2 launch bxi_example_py_elf3 example_launch_400m_run.py \
  wait_for_start_signal:=false \
  enable_line_follow:=true \
  line_enable_lateral_correction:=false \
  line_forward_vel:=1.5 \
  line_max_amp_input_vx:=1.5 \
  line_control_mode:=pid \
  line_pid_error_source:=offset \
  line_yaw_gain:=0.28 \
  line_pid_kp:=-1.0 \
  line_pid_ki:=0.0 \
  line_pid_kd:=0.01 \
  line_max_yawdot:=0.10 \
  line_max_yawdot_rate:=0.35 \
  line_max_amp_input_yawdot:=0.10 \
  2>&1 | tee "$LOG_FILE"
