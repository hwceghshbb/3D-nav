from setuptools import setup
import os

package_name = "bxi_scan_nav"
launch_files = [
    "launch/elf3_navigation.launch.py",
    "launch/elf3_nav3d.launch.py",
    "launch/elf3_rtab_cuvslam_nav.launch.py",
    "launch/elf3_rtab_mapping.launch.py",
]

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), launch_files),
        (
            os.path.join("share", package_name, "config"),
            [
                "config/elf3_octo_scan_nav.rviz",
                "config/elf3_rtab_mapping.rviz",
                "config/navigation.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "depth_to_pointcloud = bxi_scan_nav.depth_to_pointcloud:main",
            "clicked_point_3d_goal = bxi_scan_nav.clicked_point_3d_goal:main",
            "scan_planner_adapter = bxi_scan_nav.scan_planner_adapter:main",
            "rtabmap_grid_astar_planner = bxi_scan_nav.rtabmap_grid_astar_planner:main",
            "elevation_global_planner = bxi_scan_nav.elevation_global_planner:main",
            "pct_global_planner = bxi_scan_nav.pct_global_planner:main",
            "nav2_global_path_planner = bxi_scan_nav.nav2_global_path_planner:main",
            "global_path_local_target = bxi_scan_nav.global_path_local_target:main",
            "split_level_global_planner = bxi_scan_nav.split_level_global_planner:main",
            "bspline_path_visualizer = bxi_scan_nav.bspline_path_visualizer:main",
        ],
    },
)
