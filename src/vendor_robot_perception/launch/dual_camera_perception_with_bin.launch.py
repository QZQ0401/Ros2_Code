"""双相机融合 + minimum-area target estimator + bin estimator。"""

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

    target_config = (
        package_share
        / "config"
        / "target_estimator.yaml"
    )

    bin_config = (
        package_share
        / "config"
        / "bin_estimator.yaml"
    )

    cloud_fusion = Node(
        package="vendor_robot_perception",
        executable="multi_camera_cloud_fusion",
        name="multi_camera_cloud_fusion",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_frame": "world",
                "input_topic_1": "/workspace_camera/points",
                "input_topic_2": "/workspace_camera_2/points",
                "output_topic": "/workspace/merged_points",
                "voxel_leaf_size": 0.003,
                "sync_slop": 0.08,
                "tf_timeout": 0.20,
            }
        ],
    )


    target_estimator = Node(
        package="vendor_robot_perception",
        executable="target_estimator",
        name="target_estimator",
        output="screen",
        parameters=[
            str(target_config),
            {
                # Only override parameters needed for fused input.
                # Existing target/PlanningScene behavior stays unchanged.
                "input_cloud_topic": "/workspace/merged_points",
                "input_is_fused_cloud": True,
                "output_frame": "world",
                "selection_frame": "workspace_camera_optical_frame",
            },
        ],
    )


    bin_estimator = Node(
        package="vendor_robot_perception",
        executable="bin_estimator",
        name="bin_estimator",
        output="screen",
        parameters=[
            str(bin_config),
        ],
    )


    return LaunchDescription(
        [
            cloud_fusion,
            target_estimator,
            bin_estimator,
        ]
    )
