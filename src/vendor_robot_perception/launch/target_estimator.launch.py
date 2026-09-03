"""启动双工作区相机点云融合与目标估计。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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

    fusion = Node(
        package="vendor_robot_perception",
        executable="multi_camera_cloud_fusion",
        name="multi_camera_cloud_fusion",
        output="screen",
        parameters=[str(config_file)],
    )

    target_estimator = Node(
        package="vendor_robot_perception",
        executable="target_estimator",
        name="target_estimator",
        output="screen",
        parameters=[str(config_file)],
    )

    return LaunchDescription(
        [
            fusion,
            target_estimator,
        ]
    )