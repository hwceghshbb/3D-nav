#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
INSTALL_PREFIX="${NAV3D_INSTALL_PREFIX:-$WORKSPACE_DIR/install}"
FAILED=0

pass() { printf '[OK] %s\n' "$1"; }
fail() { printf '[FAIL] %s\n' "$1" >&2; FAILED=1; }
info() { printf '[INFO] %s\n' "$1"; }

required_files=(
  "third_party/ORB_SLAM3/include/System.h"
  "third_party/ORB_SLAM3/lib/libORB_SLAM3.so"
  "third_party/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so"
  "third_party/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so"
  "third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
  "third_party/OctoPlanner3D-ROS2/planner/include/global_planner.h"
  "third_party/OctoPlanner3D-ROS2/lib/liboctomap.so.1.10"
  "third_party/OctoPlanner3D-ROS2/lib/liboctomath.so.1.10"
)
for relative_path in "${required_files[@]}"; do
  if [[ -f "$NAV3D_ROOT/$relative_path" ]]; then
    pass "$relative_path"
  else
    fail "missing $relative_path"
  fi
done

if find "$NAV3D_ROOT" -type l -xtype l -print -quit | grep -q .; then
  fail "broken symbolic links exist under $NAV3D_ROOT"
else
  pass "no broken symbolic links"
fi

source /opt/ros/humble/setup.bash
if [[ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
  pass "BXI runtime found"
else
  info "BXI runtime not found; command bridge and official policy are unavailable"
fi
if [[ -f "$INSTALL_PREFIX/setup.bash" ]]; then
  source "$INSTALL_PREFIX/setup.bash"
fi

system_packages=(
  cv_bridge
  depth_image_proc
  pcl_conversions
  rtabmap_launch
  rtabmap_slam
  rtabmap_sync
  rviz2
)
for package in "${system_packages[@]}"; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    pass "ROS package $package"
  else
    fail "missing ROS package $package"
  fi
done

optional_packages=(bxi_example_py_elf3 bxi_depth_camera communication)
for package in "${optional_packages[@]}"; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    pass "integration package $package"
  else
    info "$package is external; its related simulation/hardware mode is unavailable"
  fi
done

if python3 -c 'import cuvslam' >/dev/null 2>&1; then
  pass "cuVSLAM Python binding"
else
  info "cuVSLAM binding not installed; use orbslam3 or rtabmap"
fi

if [[ -f /opt/ros/humble/lib/cmake/Pangolin/PangolinConfig.cmake ]]; then
  pass "Pangolin development package"
else
  fail "missing ros-humble-pangolin"
fi

executables=(
  "$INSTALL_PREFIX/lib/bxi_orbslam3_ros2/rgbd_inertial"
  "$INSTALL_PREFIX/lib/bxi_orbslam3_ros2/sensor_qos_relay"
  "$INSTALL_PREFIX/lib/bxi_octo_global_planner/octo_global_planner_node"
)
for executable in "${executables[@]}"; do
  if [[ ! -x "$executable" ]]; then
    info "not built yet: $executable"
    continue
  fi
  if [[ -L "$executable" ]]; then
    link_target="$(readlink "$executable")"
    if [[ "$link_target" = /* ]]; then
      fail "absolute install symlink in $executable -> $link_target"
    else
      pass "relative install symlink in $(basename "$executable")"
    fi
  else
    pass "installed file in $(basename "$executable")"
  fi
  if readelf -d "$executable" 2>/dev/null | grep -E 'RPATH|RUNPATH' | grep -q '/home/'; then
    fail "absolute home-directory RUNPATH in $executable"
  else
    pass "portable RUNPATH in $(basename "$executable")"
  fi
  if ldd "$executable" 2>/dev/null | grep -q 'not found'; then
    fail "unresolved libraries in $executable"
  else
    pass "all libraries resolved for $(basename "$executable")"
  fi
done

installed_libraries=(
  "$INSTALL_PREFIX/lib/libORB_SLAM3.so"
  "$INSTALL_PREFIX/lib/libDBoW2.so"
  "$INSTALL_PREFIX/lib/libg2o.so"
  "$INSTALL_PREFIX/lib/liboctomap.so.1.10"
  "$INSTALL_PREFIX/lib/liboctomath.so.1.10"
)
for library in "${installed_libraries[@]}"; do
  if [[ ! -f "$library" ]]; then
    info "not installed yet: $library"
    continue
  fi
  if [[ -L "$library" && "$(readlink "$library")" = /* ]]; then
    fail "absolute install symlink in $library -> $(readlink "$library")"
  elif readelf -d "$library" 2>/dev/null | grep -E 'RPATH|RUNPATH' | grep -q '/home/'; then
    fail "absolute home-directory RUNPATH in $library"
  else
    pass "portable installed library $(basename "$library")"
  fi
done

if ((FAILED)); then
  echo "Portability check failed." >&2
  exit 1
fi
echo "Portability check passed."
