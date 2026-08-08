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
# The mapping camera currently publishes under /hardware/head_depth_camera.
# Override NAV_CAMERA_NAME only when the robot driver namespace is changed.
NAV_CAMERA_NAME="${NAV_CAMERA_NAME:-head_depth_camera}"
# The camera IMU is useful for diagnostics, but Madgwick orientation caused
# about 0.45 m of Z drift in the first 90 s of the robot bag. Flat-floor
# mapping therefore uses visual RGB-D odometry and a hard 3DoF constraint.
RTABMAP_USE_IMU="${RTABMAP_USE_IMU:-false}"
RTABMAP_FILTER_IMU="${RTABMAP_FILTER_IMU:-false}"
RTABMAP_APPROX_SYNC="${RTABMAP_APPROX_SYNC:-true}"
# Camera input still arrives through DDS, while RGBDSync, odometry and Core
# exchange their single-consumer messages in this component container.
RTABMAP_USE_INTRA_PROCESS="${RTABMAP_USE_INTRA_PROCESS:-true}"
RTABMAP_MOTION_PROFILE="${RTABMAP_MOTION_PROFILE:-realtime}"
RTABMAP_MAPPING_PROFILE="${RTABMAP_MAPPING_PROFILE:-balanced}"
RTABMAP_FORCE_3DOF="${RTABMAP_FORCE_3DOF:-true}"
RTABMAP_PROCESS_LATEST="${RTABMAP_PROCESS_LATEST:-true}"
RTABMAP_TOPIC_QUEUE_SIZE="${RTABMAP_TOPIC_QUEUE_SIZE:-15}"
RTABMAP_SYNC_QUEUE_SIZE="${RTABMAP_SYNC_QUEUE_SIZE:-15}"
# The Python rig guard deserializes both full-resolution images only to inspect
# their headers. Keep it out of the production RTAB path; RGBDSync performs the
# actual timestamp synchronization and reports missing input itself.
START_INPUT_GUARD="${START_INPUT_GUARD:-false}"
# The robot camera manager publishes images and CameraInfo, but not the
# RealSense optical-frame TF tree. Publish the calibrated static rig TF here.
USE_REALSENSE_INTERNAL_TF="${USE_REALSENSE_INTERNAL_TF:-false}"
ENFORCE_FLAT_FLOOR="${ENFORCE_FLAT_FLOOR:-true}"
# The neutral XML/URDF pose puts bxi_base_link at 1.1 m and the camera link
# 0.2515 m above it, giving the verified 1.3515 m camera height above ground.
# head_link_* are base-relative TF values, not world/ground coordinates.
CAMERA_GROUND_HEIGHT="${CAMERA_GROUND_HEIGHT:-1.3515}"
HEAD_LINK_Z="${HEAD_LINK_Z:-0.2515}"
HEAD_LINK_ROLL="${HEAD_LINK_ROLL:-0.0}"
HEAD_LINK_PITCH="${HEAD_LINK_PITCH:-0.0}"
HEAD_LINK_YAW="${HEAD_LINK_YAW:-0.0}"
IMU_TOPIC="${IMU_TOPIC:-/hardware/$NAV_CAMERA_NAME/imu}"
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
echo "Ground-to-camera height: $CAMERA_GROUND_HEIGHT m"

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
  start_input_guard:="$START_INPUT_GUARD" \
  require_head_lock:=true \
  enforce_flat_floor:="$ENFORCE_FLAT_FLOOR" \
  use_realsense_internal_tf:="$USE_REALSENSE_INTERNAL_TF" \
  head_link_z:="$HEAD_LINK_Z" \
  head_link_roll:="$HEAD_LINK_ROLL" \
  head_link_pitch:="$HEAD_LINK_PITCH" \
  head_link_yaw:="$HEAD_LINK_YAW" \
  rtabmap_approx_sync:="$RTABMAP_APPROX_SYNC" \
  rtabmap_use_intra_process:="$RTABMAP_USE_INTRA_PROCESS" \
  rtabmap_topic_queue_size:="$RTABMAP_TOPIC_QUEUE_SIZE" \
  rtabmap_sync_queue_size:="$RTABMAP_SYNC_QUEUE_SIZE" \
  rtabmap_motion_profile:="$RTABMAP_MOTION_PROFILE" \
  rtabmap_mapping_profile:="$RTABMAP_MAPPING_PROFILE" \
  rtabmap_force_3dof:="$RTABMAP_FORCE_3DOF" \
  rtabmap_odom_always_process_most_recent_frame:="$RTABMAP_PROCESS_LATEST" \
  rtabmap_use_imu:="$RTABMAP_USE_IMU" \
  rtabmap_filter_imu:="$RTABMAP_FILTER_IMU" \
  imu_topic:="$IMU_TOPIC" \
  joint_states_topic:=/hardware/joint_states \
  body_pose_topic:=/nav/base_footprint/pose \
  nav_camera_pose_topic:=/nav/head_depth_camera/pose \
  nav_camera_frame_id:="${NAV_CAMERA_NAME}_color_optical_frame" \
  body_camera_uses_head_mount:=false \
  "$@"
