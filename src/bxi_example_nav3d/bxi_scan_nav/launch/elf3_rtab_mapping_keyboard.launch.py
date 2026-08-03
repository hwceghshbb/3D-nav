from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    launch_file = (
        get_package_share_path("bxi_scan_nav")
        / "launch"
        / "elf3_rtab_scan_nav.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file)),
                launch_arguments={
                    "start_controller": "true",
                    "start_scan_planner": "false",
                    "start_global_planner": "false",
                    "start_keyboard_teleop": "true",
                    "start_rviz": "true",
                    "start_rtabmap": "true",
                    "localization": "false",
                    "delete_db_on_start": "true",
                    "rtabmap_viz": "false",
                }.items(),
            )
        ]
    )
