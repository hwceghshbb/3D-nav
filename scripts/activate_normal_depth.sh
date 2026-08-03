#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi
set -u

ros2 topic pub --once /motion_commands communication/msg/MotionCommands "
height_des: 1.0
btn_10: 7
"
