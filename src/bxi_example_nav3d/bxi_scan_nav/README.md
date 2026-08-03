# BXI ELF3 3D Navigation Scaffold

This package is the ROS2 integration layer for running the BXI ELF3 in a MuJoCo
indoor scene with a simulated Intel RealSense D435i. The default scene is an
OpenHUTB Rooms-DataGen inspired multi-room MJCF layout: simple grey rooms,
door openings, and a few low obstacles for deterministic navigation tests.

## Start

Build from `bxi_rl_controller_ros2_example` using the existing `build.sh`, then:

```bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch bxi_scan_nav elf3_scan_nav_sim.launch.py
```

The launch assumes the existing `mujoco` ROS2 package and the ELF3 policy
controller are installed. Set `start_controller` to false while debugging only
the sensor and scene:

```bash
ros2 launch bxi_scan_nav elf3_scan_nav_sim.launch.py start_controller:=false
```

To launch the BXI room simulation with the ROS 2 SCAN-Planner branch connected:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bxi_scan_nav elf3_scan_planner_nav.launch.py start_controller:=false
```

Use `start_controller:=false` for planner bring-up and visualization. Switch it
to `true` only after the robot startup policy is ready to consume
`motion_commands`.

To use OctoPlanner3D as the global planner and keep SCAN-Planner as the local
planner:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bxi_scan_nav elf3_octo_scan_nav.launch.py start_controller:=false
```

The Octo node builds an OctoMap from `input_pcd`, subscribes to
`/simulation/base_footprint/pose` and `/move_base_simple/goal`, then publishes
the 3D global path on `/initial_path` for SCAN-Planner `fsm.navi_mode:=3`.
Override `input_pcd:=/path/to/map.pcd` when using a map that matches the
current MuJoCo scene.

To launch the full RTAB-Map + global A* + SCAN local planner stack:

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch bxi_scan_nav elf3_rtab_scan_nav.launch.py \
  start_controller:=false delete_db_on_start:=true
```

For RTAB-Map mapping, use the keyboard teleop launch. It starts the ELF3
controller and RTAB-Map, but disables SCAN and the global planner so manual
commands are the only source of `motion_commands`:

```bash
ros2 launch bxi_scan_nav elf3_rtab_mapping_keyboard.launch.py
```

Keyboard controls:

- `w/s`: latch forward/backward velocity
- `a/d`: latch left/right velocity
- `q/e`: latch yaw left/right
- `x`: clear linear velocity
- `c`: clear yaw velocity
- `space`: full stop
- `1`: normal/stand pulse
- `2`: recover pulse

Mapping mode is the default (`localization:=false`). After saving a database,
restart in relocalization mode with the same `database_path`:

```bash
ros2 launch bxi_scan_nav elf3_rtab_scan_nav.launch.py \
  start_controller:=false localization:=true \
  database_path:=/tmp/bxi_elf3_rtabmap.db
```

## Room generation

The room generator is based on the OpenHUTB Rooms documentation and its
Rooms-DataGen topology, but emits native MuJoCo MJCF so it can be loaded by the
existing BXI simulator:

```bash
ros2 run bxi_scan_nav generate_rooms_mjcf \
  --output src/bxi_example_py_elf3/data/mujoco_simulation/elf3_rooms_datagen.xml \
  --rows 2 --cols 3 --room-size 4.0 --door-width 1.35
```

For the current BXI ELF3 model, write the generated XML into
`data/mujoco_simulation` so `elf3.xml` and its relative `meshes` directory are
resolved by MuJoCo. The `--include-file` option is intended for custom robot
layouts with their own asset paths.

Reference pages: `https://openhutb.github.io/mujoco_plugin/underwater/packages/Ocean/Rooms/rooms/`
and `https://openhutb.github.io/mujoco_plugin/underwater/packages/Ocean/Rooms/Rooms-DataGen/`.

The generated multi-floor scene keeps each stair entrance inside the east-room
door opening and leaves an entry gap in the first two railing sections. After
changing the scene geometry, create a new RTAB-Map database before navigation;
an old database describes the previous stair layout.

## ROS2 topic contract

`/simulation/d435i/color/image_raw` and `/simulation/d435i/depth/image_raw`
are the simulated camera streams. The depth image is `32FC1` in meters.
`/scan_planner/local_cloud` is generated from depth only; no lidar topic is
used. `/scan_planner/odom` and `/scan_planner/depth_cloud` are the stable
planner-facing topics.
`/scan_planner/cmd_vel` is converted back to BXI `MotionCommands` on
`motion_commands`.

For SCAN-Planner, `/simulation/d435i/depth/pose` is published from
`/simulation/odom` using the D435i optical-frame offset, then synchronized with
`/simulation/d435i/depth/image_raw` by `scan_planner_node`.
The same pose node also publishes TF from `world` to
`d435i_depth_optical_frame`, and the mapping launch publishes live depth points
on `/simulation/d435i/depth/points` for RViz.

In RViz, set `Fixed Frame` to `world`, then add:

- `PointCloud2`: `/simulation/d435i/depth/points`
- `Map`: `/rtabmap/map`

## RTAB-Map and global planning

`elf3_rtab_scan_nav.launch.py` adds RTAB-Map as the mapping/relocalization
front end. RTAB-Map consumes the simulated D435i RGB-D stream and
`/simulation/d435i/depth/pose`, then publishes the 2D occupancy grid on
`/rtabmap/map`.

`rtabmap_grid_astar_planner` subscribes to `/rtabmap/map`, `/simulation/odom`,
and `/move_base_simple/goal`. It runs map-level A* on the RTAB occupancy grid
and publishes:

- `/initial_path`: sparse `nav_msgs/Path` consumed by SCAN-Planner in
  `fsm.navi_mode:=3`
- `/rtabmap_global_path`: same path for RViz/debugging

SCAN-Planner still performs the D435i depth based local 3D collision-aware
planning and replanning. Its internal `dyn_a_star` is used inside the local
trajectory optimizer over the sliding local grid; it is not a full RTAB-map
global planner by itself.

## SCAN-Planner integration

The integrated ROS 2 SCAN-Planner branch consumes D435i depth directly. The
plain SCAN launch uses `fsm.navi_mode:=1` for single-goal local planning. The
RTAB launch uses `fsm.navi_mode:=3`, where SCAN follows `/initial_path` from the
global planner and locally replans around newly observed obstacles.
