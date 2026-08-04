# bxi_example_nav3d

This directory groups the 3D mapping and navigation stack used by the ELF3
deployment example.

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

The top-level ELF3 control framework remains in `src/bxi_example_py_elf3`.
Navigation talks to it through ROS topics, especially `motion_commands`.

The default simulation scene is
`bxi_example_py_elf3/data/mujoco_simulation/elf3_rooms_nav3d.xml`, which keeps
the four-level stair-room world while including the current ELF3 deployment
robot model.
