# bxi_scan_nav

`bxi_scan_nav` is a standalone ROS 2 mapping and navigation package. It does
not start or import the robot controller, remote controller, simulator, camera
driver, or a robot-specific velocity adapter.

## Interface contract

The robot/sensor bringup stack owns hardware and simulation. Before starting
navigation it must publish:

- RGB image: `sensor_msgs/Image`
- registered depth image: `sensor_msgs/Image`
- depth camera calibration: `sensor_msgs/CameraInfo`
- robot body pose: `nav_msgs/Odometry`
- depth sensor pose: `nav_msgs/Odometry`, timestamped with the depth image
- the TF chain between `map`/`odom`, `base_link`, and the camera optical frame

Navigation accepts a goal as `geometry_msgs/PoseStamped` and publishes only a
standard `geometry_msgs/Twist` velocity command. The defaults are:

```text
inputs:
  /camera/color/image_raw
  /camera/depth/image_rect_raw
  /camera/depth/camera_info
  /camera/depth/pose
  /odom
  /goal_pose

output:
  /navigation/cmd_vel
```

The robot control stack should subscribe to `/navigation/cmd_vel`. If remote
control and autonomous navigation can both command the robot, arbitration or a
velocity mux belongs in the robot control stack; only one source may control
the robot at a time.

## Build

```bash
cd /home/hwc/code/elf-nav/bxi_rl_ros2/bxi_rl_controller_ros2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
colcon build --packages-select bxi_scan_nav --merge-install
source install/setup.bash
```

## Mapping only

The mapping launch subscribes to external RGB-D and odometry topics and starts
only RTAB-Map and optional RViz:

```bash
ros2 launch bxi_scan_nav elf3_rtab_mapping.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/depth/camera_info \
  odom_topic:=/odom \
  base_frame_id:=base_link \
  map_frame_id:=map \
  publish_tf_map:=true \
  delete_db_on_start:=true
```

To use an existing database for localization:

```bash
ros2 launch bxi_scan_nav elf3_rtab_mapping.launch.py \
  localization:=true \
  database_path:=/path/to/map.db
```

## Full navigation

The standalone navigation launch starts RTAB-Map, the occupancy-grid global
planner, SCAN local planner, closed-loop velocity controller, and optional
RViz. It never starts robot or sensor processes:

```bash
ros2 launch bxi_scan_nav elf3_navigation.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/depth/camera_info \
  odom_topic:=/odom \
  sensor_pose_topic:=/camera/depth/pose \
  goal_topic:=/goal_pose \
  cmd_vel_topic:=/navigation/cmd_vel \
  base_frame_id:=base_link \
  map_frame_id:=map
```

For navigation against a saved RTAB-Map database:

```bash
ros2 launch bxi_scan_nav elf3_navigation.launch.py \
  localization:=true \
  database_path:=/path/to/map.db
```

Camera intrinsics used by SCAN's local depth map are launch arguments. Override
`camera_fx`, `camera_fy`, `camera_cx`, and `camera_cy` when the camera profile
differs from the defaults. For `32FC1` depth in metres, set `depth_scale:=1.0`;
for `16UC1` depth in millimetres, keep `depth_scale:=1000.0`.

## Ownership boundary

```text
robot/camera stack                         bxi_scan_nav
------------------                        ------------
publish RGB-D --------------------------> RTAB-Map
publish odometry and sensor pose --------> global/local planning
publish TF ------------------------------> mapping and frame transforms
subscribe /navigation/cmd_vel <---------- closed-loop controller
```

The installed package contains only `elf3_rtab_mapping.launch.py` and
`elf3_navigation.launch.py`. Legacy simulation/controller bringup files are no
longer installed by `bxi_scan_nav` and should be owned by their respective
robot, camera, or simulation packages.
