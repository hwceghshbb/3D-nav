#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/hwc/code/elf-nav/bxi_rl_controller_ros2_example"
MAP_PCD="${1:-${WORKSPACE}/maps/bxi_elf3_rtabmap_20260801_172631_cloud_crop_cloud.ply}"
MODEL_FILE="${MODEL_FILE:-${WORKSPACE}/install/share/bxi_example_py_elf3/data/mujoco_simulation/elf3_rooms_bxsim_nav.xml}"
START_CONTROLLER="${START_CONTROLLER:-false}"
CONTROLLER_POLICY="${CONTROLLER_POLICY:-depth_origin}"
CONTROLLER_ONNX_FILE="${CONTROLLER_ONNX_FILE:-${WORKSPACE}/install/share/bxi_example_py_elf3/data/mjlab_model/model_normal.onnx}"
DEPTH_POLICY_FILE="${DEPTH_POLICY_FILE:-/home/hwc/code/elf-nav/bx_sim/policy/lyp2/dagger1.onnx}"
DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/simulation/origin_depth/depth/image_raw}"
START_DEPTH_POLICY_CAMERA="${START_DEPTH_POLICY_CAMERA:-true}"
OCTO_ROBOT_RADIUS="${OCTO_ROBOT_RADIUS:-0.20}"
OCTO_PATH_Z_OFFSET="${OCTO_PATH_Z_OFFSET:-0}"
SCAN_BODY_RADIUS="${SCAN_BODY_RADIUS:-0.22}"
SCAN_COLLISION_CLEARANCE="${SCAN_COLLISION_CLEARANCE:-0.22}"
SCAN_MAX_VEL="${SCAN_MAX_VEL:-0.25}"
SCAN_MAX_VYAW="${SCAN_MAX_VYAW:-0.35}"

cd "${WORKSPACE}"

source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash

set -u

if [[ "${CLEAN_OLD:-1}" == "1" ]]; then
  pkill -f "elf3_octo_scan_nav.launch.py" 2>/dev/null || true
  pkill -f "octo_global_planner_node" 2>/dev/null || true
  pkill -f "scan_planner_node" 2>/dev/null || true
  pkill -f "closed_loop_controller" 2>/dev/null || true
  pkill -f "bspline_path_visualizer" 2>/dev/null || true
  pkill -f "cmd_vel_to_motion_commands" 2>/dev/null || true
  pkill -f "clicked_point_3d_goal" 2>/dev/null || true
  pkill -f "origin_depth_rgbd" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  sleep 1
fi

exec ros2 launch bxi_scan_nav elf3_octo_scan_nav.launch.py \
  start_controller:="${START_CONTROLLER}" \
  model_file:="${MODEL_FILE}" \
  controller_policy:="${CONTROLLER_POLICY}" \
  controller_onnx_file:="${CONTROLLER_ONNX_FILE}" \
  depth_policy_file:="${DEPTH_POLICY_FILE}" \
  depth_image_topic:="${DEPTH_IMAGE_TOPIC}" \
  start_depth_policy_camera:="${START_DEPTH_POLICY_CAMERA}" \
  start_scan_planner:=true \
  start_octo_global_planner:=true \
  start_clicked_point_3d_goal:=true \
  start_rviz:=true \
  input_pcd:="${MAP_PCD}" \
  octo_cloud_scale:=1.0 \
  octo_resolution:=0.20 \
  octo_robot_radius:="${OCTO_ROBOT_RADIUS}" \
  octo_path_z_offset:="${OCTO_PATH_Z_OFFSET}" \
  octo_require_ground_support:=true \
  octo_ground_support_xy_radius_cells:=2 \
  octo_ground_support_depth_cells:=3 \
  octo_enable_preblocked_costmap:=false \
  octo_vertical_padding_below:=1.0 \
  octo_vertical_padding_above:=0.6 \
  scan_body_radius:="${SCAN_BODY_RADIUS}" \
  scan_collision_clearance:="${SCAN_COLLISION_CLEARANCE}" \
  scan_max_vel:="${SCAN_MAX_VEL}" \
  scan_max_vyaw:="${SCAN_MAX_VYAW}"
