#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
cd "$WORKSPACE_DIR"

# ROS setup scripts may inspect variables that are not defined yet.
set +u
source /opt/ros/humble/setup.bash
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
source install/setup.bash

LOCALIZATION_BACKEND="${LOCALIZATION_BACKEND:-rtabmap}"
MAP_ROOT="${MAP_ROOT:-$WORKSPACE_DIR/maps}"
MAP_NAME="${MAP_NAME:-elf3_full_test}"
RTABMAP_DATABASE="${RTABMAP_DATABASE:-$MAP_ROOT/$MAP_NAME.db}"
CUVSLAM_MAP_DIRECTORY="${CUVSLAM_MAP_DIRECTORY:-$MAP_ROOT/${MAP_NAME}_cuvslam}"
ORB_MAP_DIRECTORY="${ORB_MAP_DIRECTORY:-$MAP_ROOT/${MAP_NAME}_orb}"
ORB_ATLAS_NAME="${ORB_ATLAS_NAME:-atlas}"
OCTO_INPUT_PCD="${OCTO_INPUT_PCD:-$MAP_ROOT/${MAP_NAME}_octo_cloud.ply}"
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

if [[ ! -f "$RTABMAP_DATABASE" ]]; then
  echo "Missing RTAB-Map database: $RTABMAP_DATABASE" >&2
  exit 1
fi
if [[ ! -f "$OCTO_INPUT_PCD" ]]; then
  echo "Missing OctoPlanner PLY map: $OCTO_INPUT_PCD" >&2
  exit 1
fi
if [[ "$LOCALIZATION_BACKEND" == "orbslam3" ]] && \
   [[ ! -f "$ORB_MAP_DIRECTORY/$ORB_ATLAS_NAME.osa" ]]; then
  echo "Missing ORB-SLAM3 atlas: $ORB_MAP_DIRECTORY/$ORB_ATLAS_NAME.osa" >&2
  echo "Set ORB_MAP_DIRECTORY to the atlas created with this RTAB/PLY map." >&2
  exit 1
fi

echo "ELF3 localization backend: $LOCALIZATION_BACKEND"
echo "Simulation and policy must already be running via run_elf3_sim_policy.sh"

exec ros2 launch bxi_scan_nav elf3_rtab_cuvslam_nav.launch.py \
  localization_backend:="$LOCALIZATION_BACKEND" \
  rtabmap_database:="$RTABMAP_DATABASE" \
  cuvslam_map:="$CUVSLAM_MAP_DIRECTORY" \
  orb_map_directory:="$ORB_MAP_DIRECTORY" \
  orb_atlas_name:="$ORB_ATLAS_NAME" \
  octo_input_pcd:="$OCTO_INPUT_PCD" \
  start_simulation:=false \
  start_controller:=false \
  start_simulated_cameras:=true \
  start_scan_planner:=true \
  start_octo_global_planner:=true \
  start_rviz:=true \
  "${EXTRA_ARGS[@]}"
