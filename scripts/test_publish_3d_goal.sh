#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: $0 X Y Z"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
  source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi
set -u

ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "
header:
  frame_id: world
pose:
  position:
    x: $1
    y: $2
    z: $3
  orientation:
    w: 1.0
"
