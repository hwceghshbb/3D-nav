from glob import glob
import os

from setuptools import find_packages, setup


package_name = "bxi_cuvslam_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.xml"),
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hwc",
    maintainer_email="3223759606@qq.com",
    description="ELF3 head RGB-D cuVSLAM mapping and relocalization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "head_camera_rig_guard = bxi_cuvslam_localization.rig_guard:main",
            "dual_camera_rig_guard = bxi_cuvslam_localization.rig_guard:main",
            "head_cuvslam_node = bxi_cuvslam_localization.head_cuvslam_node:main",
            "imu_combiner = bxi_cuvslam_localization.imu_combiner:main",
            "localization_odom_bridge = bxi_cuvslam_localization.odom_bridge:main",
            "cuvslam_sim_error_monitor = bxi_cuvslam_localization.sim_error_monitor:main",
            "tf_pose_publisher = bxi_cuvslam_localization.tf_pose_publisher:main",
        ],
    },
)
