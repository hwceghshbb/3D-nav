from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    scan_launch = (
        get_package_share_path("bxi_scan_nav")
        / "launch"
        / "elf3_rtab_scan_nav.launch.py"
    )
    joystick_config = (
        get_package_share_path("bxi_scan_nav")
        / "config"
        / "joystick_mapping_safe.yaml"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(scan_launch)),
                launch_arguments={
                    "start_controller": "true",
                    "start_scan_planner": "false",
                    "start_global_planner": "false",
                    "start_keyboard_teleop": "false",
                    "start_rviz": "true",
                    "start_rtabmap": "true",
                    "localization": "false",
                    "delete_db_on_start": "true",
                    "rtabmap_viz": "false",
                }.items(),
            ),
            Node(
                package="remote_controller",
                executable="remote_controller",
                name="mapping_joystick_teleop",
                output="screen",
                emulate_tty=True,
                arguments=[
                    "--config",
                    str(joystick_config),
                    "--driver",
                    "joystick",
                    "--hot-reload",
                    "true",
                    "__log_level:=debug",
                ],
            ),
        ]
    )
