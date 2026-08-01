from setuptools import setup
from glob import glob
import os

package_name = "bxi_scan_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "rgbd_camera_publisher = bxi_scan_nav.rgbd_camera_publisher:main",
            "depth_to_pointcloud = bxi_scan_nav.depth_to_pointcloud:main",
            "clicked_point_3d_goal = bxi_scan_nav.clicked_point_3d_goal:main",
            "scan_planner_adapter = bxi_scan_nav.scan_planner_adapter:main",
            "cmd_vel_to_motion_commands = bxi_scan_nav.cmd_vel_to_motion_commands:main",
            "generate_rooms_mjcf = bxi_scan_nav.rooms_mjcf_generator:main",
            "d435i_pose_publisher = bxi_scan_nav.d435i_pose_publisher:main",
            "rtabmap_grid_astar_planner = bxi_scan_nav.rtabmap_grid_astar_planner:main",
            "elevation_global_planner = bxi_scan_nav.elevation_global_planner:main",
            "pct_global_planner = bxi_scan_nav.pct_global_planner:main",
            "nav2_global_path_planner = bxi_scan_nav.nav2_global_path_planner:main",
            "global_path_local_target = bxi_scan_nav.global_path_local_target:main",
            "split_level_global_planner = bxi_scan_nav.split_level_global_planner:main",
            "bspline_path_visualizer = bxi_scan_nav.bspline_path_visualizer:main",
            "keyboard_motion_teleop = bxi_scan_nav.keyboard_motion_teleop:main",
        ],
    },
)
