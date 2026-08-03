#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "ros2 launch bxi_scan_nav"
  "install/lib/bxi_scan_nav/rgbd_camera_publisher"
  "install/lib/bxi_scan_nav/d435i_pose_publisher"
  "install/lib/bxi_scan_nav/depth_to_pointcloud"
  "install/lib/bxi_scan_nav/clicked_point_3d_goal"
  "install/lib/bxi_scan_nav/keyboard_motion_teleop"
  "install/lib/bxi_scan_nav/rtabmap_grid_astar_planner"
  "install/lib/bxi_scan_nav/elevation_global_planner"
  "install/lib/bxi_scan_nav/pct_global_planner"
  "install/lib/bxi_scan_nav/nav2_global_path_planner"
  "install/lib/bxi_scan_nav/global_path_local_target"
  "install/lib/bxi_scan_nav/bspline_path_visualizer"
  "install/lib/bxi_scan_nav/cmd_vel_to_motion_commands"
  "lib/nav2_planner/planner_server"
  "lib/nav2_lifecycle_manager/lifecycle_manager"
  "lib/scan_planner/scan_planner_node"
  "lib/scan_planner/closed_loop_controller"
  "install/lib/bxi_example_py_elf3/bxi_example_py_elf3_mjlab"
  "lib/rtabmap_sync/rgbd_sync"
  "lib/rtabmap_slam/rtabmap"
  "lib/rviz2/rviz2.*elf3_rtab_mapping.rviz"
  "lib/mujoco/simulation"
  "mujoco/simulation.*elf3_rooms_datagen"
  "ros2 launch elf3_gazebo_navigation"
  "gzserver .*elf3_test.world"
  "elf3_gazebo_navigation/start_bxi_stand.py"
  "__node:=elf3_robot_state_publisher"
)

kill_matches() {
  local signal="$1"
  local pattern="$2"
  local pid=""
  while read -r pid; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "${PPID:-}" ] && continue
    kill "$signal" "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

for pattern in "${patterns[@]}"; do
  kill_matches -TERM "$pattern"
done

sleep 2

for pattern in "${patterns[@]}"; do
  kill_matches -KILL "$pattern"
done

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

printf '%s\n' 'Cleaned bxi_scan_nav child processes.'
