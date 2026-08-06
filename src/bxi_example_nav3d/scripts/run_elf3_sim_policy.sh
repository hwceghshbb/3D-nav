#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV3D_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${NAV3D_WORKSPACE:-$(cd "$NAV3D_ROOT/../.." && pwd)}"
MODEL_FILE="${MODEL_FILE:-$WORKSPACE_DIR/src/bxi_example_py_elf3/data/mujoco_simulation/elf3_rooms_nav3d_features.xml}"

source /opt/ros/humble/setup.bash
if [[ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
source "$WORKSPACE_DIR/install/setup.bash"

exec ros2 launch bxi_example_py_elf3 example_demo.launch.py \
  model_file:="$MODEL_FILE"
