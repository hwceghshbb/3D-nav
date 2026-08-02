import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_path("bxi_example_py_elf3")
    xml_file = os.path.join(
        package_share,
        "data/mujoco_simulation/elf3_400m_track.xml",
    )
    state_machine_config = os.path.join(
        package_share,
        "config/elf3_state_machine.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_demo",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/state_machine_config": state_machine_config},
                    {"/use_sim_reset": True},
                    {"/initial_base_pose": [-14.76825, 13.385, 1.1, 0.0, 0.0, 0.0, 1.0]},
                    {"/hot_reload": True},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),
        ]
    )
