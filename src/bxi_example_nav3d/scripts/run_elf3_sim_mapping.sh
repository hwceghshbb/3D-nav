#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
LOCALIZATION_BACKEND="${LOCALIZATION_BACKEND:-rtabmap}"
MAP_ROOT="${MAP_ROOT:-$WORKSPACE_DIR/maps}"
MAP_NAME="${MAP_NAME:-elf3_sim_$(date +%Y%m%d_%H%M%S)}"
RTABMAP_DATABASE="${RTABMAP_DATABASE:-$MAP_ROOT/$MAP_NAME.db}"
ORB_MAP_DIRECTORY="${ORB_MAP_DIRECTORY:-$MAP_ROOT/${MAP_NAME}_orb}"
ORB_ATLAS_NAME="${ORB_ATLAS_NAME:-atlas}"
CUVSLAM_MAP_DIRECTORY="${CUVSLAM_MAP_DIRECTORY:-$MAP_ROOT/${MAP_NAME}_cuvslam}"

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

echo "Simulation mapping backend: $LOCALIZATION_BACKEND"
echo "RTAB-Map database: $RTABMAP_DATABASE"
echo "ORB-SLAM3 atlas: $ORB_MAP_DIRECTORY/$ORB_ATLAS_NAME.osa"

exec ros2 launch bxi_scan_nav elf3_rtab_cuvslam_nav.launch.py \
  localization_backend:="$LOCALIZATION_BACKEND" \
  localization_mode:=mapping \
  rtabmap_localization:=false \
  rtabmap_database:="$RTABMAP_DATABASE" \
  orb_map_directory:="$ORB_MAP_DIRECTORY" \
  orb_atlas_name:="$ORB_ATLAS_NAME" \
  cuvslam_map:="$CUVSLAM_MAP_DIRECTORY" \
  start_simulation:=false \
  start_controller:=false \
  start_simulated_cameras:=true \
  start_scan_planner:=false \
  start_octo_global_planner:=false \
  start_rtabmap:=true \
  start_rviz:=true \
  global_frame:=world \
  color_topic:=/simulation/d435i/color/image_raw \
  depth_topic:=/simulation/d435i/depth/image_rect_raw \
  camera_info_topic:=/simulation/d435i/color/camera_info \
  imu_topic:=/simulation/imu_data \
  joint_states_topic:=/simulation/joint_states \
  body_pose_topic:=/nav/base_footprint/pose \
  nav_camera_pose_topic:=/nav/head_depth_camera/pose \
  "$@"
