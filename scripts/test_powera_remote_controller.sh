#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${ROOT}/install/setup.bash"
CONFIG="${ROOT}/install/remote_controller/share/remote_controller/config/xbox_default.yaml"
DEVICE="/dev/input/js0"

if [[ ! -f "$CONFIG" ]]; then
    CONFIG="${ROOT}/src/remote_controller/config/xbox_default.yaml"
fi

usage() {
    cat <<EOF
Usage: $(basename "$0") [--device /dev/input/js0] [--config PATH]

Starts the existing remote_controller and echoes communication/msg/MotionCommands.
This publishes commands on /motion_commands. Keep the robot lifted or disconnect
the motor controller while testing.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --device)
            [[ $# -ge 2 ]] || { echo "--device requires a path" >&2; exit 2; }
            DEVICE="$2"
            shift 2
            ;;
        --config)
            [[ $# -ge 2 ]] || { echo "--config requires a path" >&2; exit 2; }
            CONFIG="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ -f "$ROS_SETUP" ]] || { echo "missing ROS setup: $ROS_SETUP" >&2; exit 1; }
[[ -f "$WORKSPACE_SETUP" ]] || { echo "missing workspace setup: $WORKSPACE_SETUP" >&2; exit 1; }
[[ -f "$CONFIG" ]] || { echo "missing controller config: $CONFIG" >&2; exit 1; }

set +u
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u

[[ -e "$DEVICE" ]] || {
    echo "joystick device not found: $DEVICE" >&2
    echo "Connect the PowerA controller and check: ls -l /dev/input/js*" >&2
    exit 1
}

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 is not available after sourcing the workspaces" >&2
    exit 1
fi

controller_pid=""
temp_config=""
cleanup() {
    if [[ -n "$controller_pid" ]] && kill -0 "$controller_pid" 2>/dev/null; then
        kill -INT "$controller_pid" 2>/dev/null || true
        wait "$controller_pid" 2>/dev/null || true
    fi
    if [[ -n "$temp_config" ]]; then
        rm -f "$temp_config"
    fi
}
trap cleanup EXIT INT TERM

echo "Starting remote_controller with device ${DEVICE}"
echo "Watching /motion_commands; press Ctrl-C to stop"
echo "Mapping: left stick Y -> vx, left stick X -> vy, right stick X -> yaw"
echo "Buttons and command values are printed by ros2 topic echo"

if [[ "$DEVICE" != "/dev/input/js0" ]]; then
    temp_config="$(mktemp /tmp/powera_remote_controller.XXXXXX.yaml)"
    sed "s#/dev/input/js0#$DEVICE#g" "$CONFIG" >"$temp_config"
    CONFIG="$temp_config"
fi

ros2 run remote_controller remote_controller \
    --config "$CONFIG" \
    --no-hot-reload \
    >"/tmp/powera_remote_controller.log" 2>&1 &
controller_pid=$!

sleep 1
if ! kill -0 "$controller_pid" 2>/dev/null; then
    cat "/tmp/powera_remote_controller.log" >&2 || true
    exit 1
fi

ros2 topic echo /motion_commands communication/msg/MotionCommands
