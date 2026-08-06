#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
LOCALIZATION_BACKEND="${LOCALIZATION_BACKEND:-rtabmap}"
MAP_ROOT="${MAP_ROOT:-$WORKSPACE_DIR/maps}"
MAP_NAME="${MAP_NAME:-elf3_hw_$(date +%Y%m%d_%H%M%S)}"
RTABMAP_DATABASE="${RTABMAP_DATABASE:-$MAP_ROOT/$MAP_NAME.db}"
ORB_MAP_DIRECTORY="${ORB_MAP_DIRECTORY:-$MAP_ROOT/${MAP_NAME}_orb}"
ORB_ATLAS_NAME="${ORB_ATLAS_NAME:-atlas}"
ORB_MAX_TIME_DIFF="${ORB_MAX_TIME_DIFF:-0.008}"
CUVSLAM_MAP_DIRECTORY="${CUVSLAM_MAP_DIRECTORY:-$MAP_ROOT/${MAP_NAME}_cuvslam}"
NAV_CAMERA_NAME="${NAV_CAMERA_NAME:-head_depth_camera}"
COLOR_INFO_TOPIC="${COLOR_INFO_TOPIC:-/hardware/$NAV_CAMERA_NAME/color/camera_info}"
ALIGNED_DEPTH_TOPIC="${ALIGNED_DEPTH_TOPIC:-/hardware/$NAV_CAMERA_NAME/aligned_depth_to_color/image_raw}"
ALIGNED_DEPTH_INFO_TOPIC="${ALIGNED_DEPTH_INFO_TOPIC:-/hardware/$NAV_CAMERA_NAME/aligned_depth_to_color/camera_info}"
ORB_SETTINGS="${ORB_SETTINGS:-$WORKSPACE_DIR/install/share/bxi_orbslam3_ros2/config/elf3_head_1280x720_rgbd.yaml}"

case "${1:-}" in
  cuvslam|rtabmap|orbslam3)
    LOCALIZATION_BACKEND="$1"
    shift
    ;;
  localization_backend:=*)
    LOCALIZATION_BACKEND="${1#*=}"
    shift
    ;;
esac

case "$LOCALIZATION_BACKEND" in
  cuvslam|rtabmap|orbslam3) ;;
  *)
    echo "localization_backend must be cuvslam, rtabmap, or orbslam3" >&2
    exit 2
    ;;
esac

mkdir -p "$MAP_ROOT" "$ORB_MAP_DIRECTORY" "$CUVSLAM_MAP_DIRECTORY"

source /opt/ros/humble/setup.bash
if [[ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
source "$WORKSPACE_DIR/install/setup.bash"

echo "Hardware mapping backend: $LOCALIZATION_BACKEND"
echo "RTAB-Map database: $RTABMAP_DATABASE"
echo "ORB-SLAM3 atlas: $ORB_MAP_DIRECTORY/$ORB_ATLAS_NAME.osa"

exec ros2 launch bxi_scan_nav elf3_rtab_cuvslam_nav.launch.py \
  localization_backend:="$LOCALIZATION_BACKEND" \
  localization_mode:=mapping \
  rtabmap_localization:=false \
  rtabmap_database:="$RTABMAP_DATABASE" \
  orb_map_directory:="$ORB_MAP_DIRECTORY" \
  orb_atlas_name:="$ORB_ATLAS_NAME" \
  orb_max_time_diff:="$ORB_MAX_TIME_DIFF" \
  orb_settings:="$ORB_SETTINGS" \
  cuvslam_map:="$CUVSLAM_MAP_DIRECTORY" \
  start_simulation:=false \
  start_controller:=false \
  start_simulated_cameras:=false \
  start_scan_planner:=false \
  start_octo_global_planner:=false \
  start_rtabmap:=true \
  start_rviz:=true \
  global_frame:=map \
  color_topic:="/hardware/$NAV_CAMERA_NAME/color/image_raw" \
  depth_topic:="$ALIGNED_DEPTH_TOPIC" \
  raw_depth_topic:="/hardware/$NAV_CAMERA_NAME/depth/image_rect_raw" \
  depth_camera_info_topic:="/hardware/$NAV_CAMERA_NAME/depth/camera_info" \
  registered_depth_camera_info_topic:="$ALIGNED_DEPTH_INFO_TOPIC" \
  start_depth_registration:=false \
  camera_info_topic:="$COLOR_INFO_TOPIC" \
  camera_width:=1280 \
  camera_height:=720 \
  rtabmap_use_imu:=false \
  imu_topic:=/hardware/imu_data \
  joint_states_topic:=/hardware/joint_states \
  body_pose_topic:=/nav/base_footprint/pose \
  nav_camera_pose_topic:=/nav/head_depth_camera/pose \
  nav_camera_frame_id:="${NAV_CAMERA_NAME}_color_optical_frame" \
  body_camera_uses_head_mount:=false \
  "$@"
