# bxi_example_nav3d

This directory is the portable source bundle for ELF3 3D mapping and
navigation. Copy the whole directory to `<new_workspace>/src/bxi_example_nav3d`;
do not copy individual child packages.

It intentionally keeps the upstream ROS packages as separate packages so
`colcon` can build their messages and C++ libraries correctly:

- `bxi_scan_nav`: launch files, RViz configs, camera/goal/command bridge nodes.
- `bxi_octo_global_planner`: OctoMap based 3D global planner.
- `plan_manage`: SCAN local planner package, installed as `scan_planner`.
- `plan_env`: local depth map environment.
- `path_searching`: local path search utilities.
- `bspline_opt`: B-spline trajectory optimization.
- `traj_utils`: trajectory message helpers and visualization utilities.
- `scan_planner_msgs`: SCAN planner message definitions.
- `bxi_cuvslam_localization`: head RGB-D cuVSLAM mapping/relocalization,
  input checks, map-frame anchoring, and guarded `/nav/odom` output.
- `bxi_orbslam3_ros2`: ORB-SLAM3 RGB-D adapter and installed vocabulary.
- `third_party`: vendored ORB-SLAM3 and OctoPlanner3D sources/libraries.
- `scripts`: portable build, validation, mapping, export and navigation entry
  points.

This is a multi-package ROS repository rather than one `package.xml`. Keeping
the packages separate is required by ROS interface generation and the SCAN
library dependency graph. It is still copied and versioned as one
`bxi_example_nav3d` module.

## Build on another machine

The target layout is:

```text
nav3d_ws/
  src/
    bxi_example_nav3d/
  maps/
```

Build and check it with:

```bash
sudo apt install ros-humble-depth-image-proc
cd nav3d_ws
./src/bxi_example_nav3d/scripts/build_nav3d.sh
source install/setup.bash
./src/bxi_example_nav3d/scripts/check_nav3d_portability.sh
```

The build script always selects the ORB-SLAM3 and OctoPlanner3D copies under
`third_party`. Their shared libraries and the ORB vocabulary are copied into
the ROS install tree. Installed executables therefore do not retain paths such
as `/home/hwc/...`.

## External contracts

The navigation algorithms are included, but these platform/system components
remain external by design:

- ROS 2 Humble and the ROS dependencies declared by each `package.xml`.
- RTAB-Map, PCL, OpenCV, Eigen, Pangolin and Boost system libraries.
- `/opt/bxi/bxi_ros2_pkg` for `communication/MotionCommands` and the official
  robot runtime.
- `bxi_example_py_elf3` and MuJoCo for simulation policy execution.
- `bxi_depth_camera` and the ELF3 hardware driver for real sensor topics.
- NVIDIA cuVSLAM Python bindings only when `localization_backend:=cuvslam` is
  selected. ORB-SLAM3 and RTAB-Map do not require this binding.

The navigation process communicates with the official control framework only
through ROS topics. Its velocity output is `/cmd_vel`; the command bridge
converts that to the official `motion_commands` interface.

Maps are runtime data and should be copied separately into `<workspace>/maps`:

```text
site.db
site_orb/atlas.osa
site_cloud.ply
```

The hardware entry points use the navigation camera topics named
`/hardware/head_depth_camera/*`. RGB and SDK-aligned depth are processed at
1280x720 and 30 Hz with the camera's measured intrinsics. ORB-SLAM3 and
RTAB-Map consume
`/hardware/head_depth_camera/aligned_depth_to_color/image_raw` directly, with
`/hardware/head_depth_camera/color/camera_info` as the RGB camera model. Do
not run that stream through `depth_image_proc` again. The camera driver must
also publish `aligned_depth_to_color/camera_info` and its internal TF; the fixed
`base_link` to camera mount transform must be calibrated on the robot.

## Scripts

```text
scripts/build_nav3d.sh
scripts/check_nav3d_portability.sh
scripts/run_elf3_sim_policy.sh
scripts/run_elf3_sim_mapping.sh
scripts/run_elf3_rtab_cuvslam_nav.sh
scripts/run_elf3_hw_mapping.sh
scripts/run_elf3_hw_nav3d.sh
scripts/export_elf3_octo_map.sh
```


The default simulation scene is
`bxi_example_py_elf3/data/mujoco_simulation/elf3_rooms_nav3d_features.xml`, which keeps
the four-level stair-room world while including the current ELF3 deployment
robot model.
