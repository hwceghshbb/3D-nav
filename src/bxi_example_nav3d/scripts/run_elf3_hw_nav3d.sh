#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
LOCALIZATION_BACKEND="${LOCALIZATION_BACKEND:-rtabmap}"
RTABMAP_DATABASE="${RTABMAP_DATABASE:-/opt/bxi/maps/site.db}"
ORB_MAP_DIRECTORY="${ORB_MAP_DIRECTORY:-/opt/bxi/maps/site_orb}"
ORB_ATLAS_NAME="${ORB_ATLAS_NAME:-atlas}"
ORB_MAX_TIME_DIFF="${ORB_MAX_TIME_DIFF:-0.008}"
ORB_SETTINGS="${ORB_SETTINGS:-$WORKSPACE_DIR/install/share/bxi_orbslam3_ros2/config/elf3_head_1280x720_rgbd.yaml}"
CUVSLAM_MAP_DIRECTORY="${CUVSLAM_MAP_DIRECTORY:-/opt/bxi/maps/site_cuvslam}"
OCTO_INPUT_PCD="${OCTO_INPUT_PCD:-/opt/bxi/maps/site_cloud.ply}"
RTABMAP_USE_IMU="${RTABMAP_USE_IMU:-false}"
RTABMAP_FILTER_IMU="${RTABMAP_FILTER_IMU:-false}"
RTABMAP_FORCE_3DOF="${RTABMAP_FORCE_3DOF:-true}"
RTABMAP_USE_INTRA_PROCESS="${RTABMAP_USE_INTRA_PROCESS:-false}"
ENFORCE_FLAT_FLOOR="${ENFORCE_FLAT_FLOOR:-true}"
NAV_CAMERA_NAME="${NAV_CAMERA_NAME:-head_depth_camera}"
IMU_TOPIC="${IMU_TOPIC:-/hardware/$NAV_CAMERA_NAME/imu}"
START_RVIZ="${START_RVIZ:-true}"
START_SCAN_PLANNER="${START_SCAN_PLANNER:-true}"
START_OCTO_GLOBAL_PLANNER="${START_OCTO_GLOBAL_PLANNER:-true}"
EXTRA_ARGS=()

while (($#)); do
  case "$1" in
    cuvslam|rtabmap|orbslam3)
      LOCALIZATION_BACKEND="$1"
      ;;
    --localization-backend)
      shift
      LOCALIZATION_BACKEND="${1:-}"
      ;;
    --localization-backend=*)
      LOCALIZATION_BACKEND="${1#*=}"
      ;;
    localization_backend:=*)
      LOCALIZATION_BACKEND="${1#*=}"
      ;;
    *)
      EXTRA_ARGS+=("$1")
      ;;
  esac
  shift
done

case "$LOCALIZATION_BACKEND" in
  cuvslam|rtabmap|orbslam3) ;;
  *)
    echo "localization_backend must be cuvslam, rtabmap, or orbslam3" >&2
    exit 2
    ;;
esac

source /opt/ros/humble/setup.bash
if [[ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
source "$WORKSPACE_DIR/install/setup.bash"

if [[ ! -f "$RTABMAP_DATABASE" ]]; then
  echo "Missing RTAB-Map database: $RTABMAP_DATABASE" >&2
  exit 1
fi
if [[ "$START_OCTO_GLOBAL_PLANNER" == "true" ]] && \
   [[ ! -f "$OCTO_INPUT_PCD" ]]; then
  echo "Missing OctoPlanner PLY map: $OCTO_INPUT_PCD" >&2
  exit 1
fi
if [[ "$LOCALIZATION_BACKEND" == "orbslam3" ]] && \
   [[ ! -f "$ORB_MAP_DIRECTORY/$ORB_ATLAS_NAME.osa" ]]; then
  echo "Missing ORB-SLAM3 atlas: $ORB_MAP_DIRECTORY/$ORB_ATLAS_NAME.osa" >&2
  exit 1
fi

exec ros2 launch bxi_cuvslam_localization elf3_cuvslam_nav.launch.py \
  localization_backend:="$LOCALIZATION_BACKEND" \
  rtabmap_database:="$RTABMAP_DATABASE" \
  orb_map_directory:="$ORB_MAP_DIRECTORY" \
  orb_atlas_name:="$ORB_ATLAS_NAME" \
  orb_max_time_diff:="$ORB_MAX_TIME_DIFF" \
  orb_settings:="$ORB_SETTINGS" \
  cuvslam_map_directory:="$CUVSLAM_MAP_DIRECTORY" \
  input_pcd:="$OCTO_INPUT_PCD" \
  rtabmap_use_imu:="$RTABMAP_USE_IMU" \
  rtabmap_filter_imu:="$RTABMAP_FILTER_IMU" \
  rtabmap_force_3dof:="$RTABMAP_FORCE_3DOF" \
  rtabmap_use_intra_process:="$RTABMAP_USE_INTRA_PROCESS" \
  enforce_flat_floor:="$ENFORCE_FLAT_FLOOR" \
  imu_topic:="$IMU_TOPIC" \
  color_topic:="/hardware/$NAV_CAMERA_NAME/color/image_raw" \
  depth_topic:="/hardware/$NAV_CAMERA_NAME/aligned_depth_to_color/image_raw" \
  raw_depth_topic:="/hardware/$NAV_CAMERA_NAME/depth/image_rect_raw" \
  depth_camera_info_topic:="/hardware/$NAV_CAMERA_NAME/depth/camera_info" \
  registered_depth_camera_info_topic:="/hardware/$NAV_CAMERA_NAME/aligned_depth_to_color/camera_info" \
  camera_info_topic:="/hardware/$NAV_CAMERA_NAME/color/camera_info" \
  nav_camera_frame_id:="${NAV_CAMERA_NAME}_color_optical_frame" \
  start_scan_planner:="$START_SCAN_PLANNER" \
  start_octo_global_planner:="$START_OCTO_GLOBAL_PLANNER" \
  start_rviz:="$START_RVIZ" \
  "${EXTRA_ARGS[@]}"
