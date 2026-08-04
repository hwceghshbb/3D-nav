#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE_DIR"

# ROS setup scripts may inspect variables that are not defined yet.
set +u
source /opt/ros/humble/setup.bash
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
source install/setup.bash

exec ros2 launch bxi_scan_nav elf3_rtab_cuvslam_nav.launch.py \
  rtabmap_database:="$WORKSPACE_DIR/maps/elf3_rtabmap_20260803_201219.db" \
  cuvslam_map:="$WORKSPACE_DIR/maps/cuvslam_elf3_20260803_201219" \
  octo_input_pcd:="$WORKSPACE_DIR/maps/elf3_rtabmap_20260803_201219_octo_cloud.ply" \
  start_controller:=true \
  start_scan_planner:=true \
  start_octo_global_planner:=true \
  start_rviz:=true \
  "$@"
