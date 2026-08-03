#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE_DIR"

# ROS setup files may read unset variables when the parent shell enables nounset.
set +u
source /opt/ros/humble/setup.bash
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
set -u

pkill -f "octo_global_planner_node" || true
pkill -f "scan_planner_node" || true
pkill -f "closed_loop_controller" || true
pkill -f "cmd_vel_to_motion_commands" || true
pkill -f "clicked_point_3d_goal" || true
pkill -f "nav_rgbd_camera_publisher" || true
pkill -f "nav_camera_pose_publisher" || true

DEFAULT_MAP="$PWD/maps/bxi_elf3_rtabmap_20260801_172631_cloud_crop_cloud.ply"
INPUT_PCD="${1:-${INPUT_PCD:-$DEFAULT_MAP}}"

ros2 launch bxi_scan_nav elf3_nav3d.launch.py \
  input_pcd:="$INPUT_PCD" \
  "${@:2}"
