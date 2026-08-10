from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("G4", package_name="vendor_robot_moveit_config")
        .robot_description(mappings={"use_fake_hardware": "true"})
        .to_moveit_configs()
    )
    return generate_static_virtual_joint_tfs_launch(config)
