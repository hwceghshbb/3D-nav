#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
ORB_ROOT="${NAV3D_ORB_SLAM3_ROOT:-$NAV3D_ROOT/third_party/ORB_SLAM3}"
OCTO_ROOT="${NAV3D_OCTO_PLANNER_ROOT:-$NAV3D_ROOT/third_party/OctoPlanner3D-ROS2}"
ROS_PYTHON="${NAV3D_PYTHON_EXECUTABLE:-/usr/bin/python3}"

required_files=(
  "$ORB_ROOT/include/System.h"
  "$ORB_ROOT/lib/libORB_SLAM3.so"
  "$ORB_ROOT/Thirdparty/DBoW2/lib/libDBoW2.so"
  "$ORB_ROOT/Thirdparty/g2o/lib/libg2o.so"
  "$ORB_ROOT/Vocabulary/ORBvoc.txt"
  "$OCTO_ROOT/planner/include/global_planner.h"
  "$OCTO_ROOT/lib/liboctomap.so.1.10"
  "$OCTO_ROOT/lib/liboctomath.so.1.10"
)
for path in "${required_files[@]}"; do
  if [[ ! -f "$path" ]]; then
    echo "Missing bundled dependency: $path" >&2
    exit 1
  fi
done

source /opt/ros/humble/setup.bash
if [[ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi

if [[ ! -f /opt/ros/humble/lib/cmake/Pangolin/PangolinConfig.cmake ]]; then
  echo "Missing Pangolin development package." >&2
  echo "Install it with: sudo apt install ros-humble-pangolin" >&2
  exit 1
fi

export ORB_SLAM3_ROOT_DIR="$ORB_ROOT"
mkdir -p "$WORKSPACE_DIR/maps"
cd "$WORKSPACE_DIR"

colcon build \
  --base-paths "$NAV3D_ROOT" \
  --merge-install \
  "$@" \
  --cmake-args \
    -DORB_SLAM3_ROOT_DIR="$ORB_ROOT" \
    -DOCTO_PLANNER3D_ROOT="$OCTO_ROOT" \
    -DPython3_EXECUTABLE="$ROS_PYTHON" \
    -DPYTHON_EXECUTABLE="$ROS_PYTHON"

echo
echo "Build complete. Source: $WORKSPACE_DIR/install/setup.bash"
