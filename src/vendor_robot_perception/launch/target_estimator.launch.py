"""启动基于深度点云的目标估计节点。"""

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_share = Path(
        get_package_share_directory(
            "vendor_robot_perception"
        )
    )

    config_file = (
        package_share
        / "config"
        / "target_estimator.yaml"
    )


    target_estimator = Node(
        package="vendor_robot_perception",

        executable="target_estimator",

        name="target_estimator",

        output="screen",

        parameters=[
            str(config_file),
        ],
    )


    return LaunchDescription(
        [
            target_estimator,
        ]
    )
