#!/usr/bin/env python3
"""Forward only URDF joint coordinates from Gazebo ros2_control to MoveIt."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


VALID_JOINTS = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "gripper_left_joint", "gripper_right_joint",
    "left_wheel_joint", "right_wheel_joint",
    "caster_swivel_joint", "caster_wheel_joint",
}


class JointStateFilter(Node):
    def __init__(self):
        super().__init__("joint_state_filter")
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            JointState,
            "/joint_state_broadcaster/joint_states",
            self._on_joint_state,
            10,
        )

    def _on_joint_state(self, message: JointState) -> None:
        filtered = JointState()
        filtered.header = message.header
        for index, name in enumerate(message.name):
            if name not in VALID_JOINTS:
                self.get_logger().debug("Ignoring non-URDF joint state: %s" % name)
                continue
            filtered.name.append(name)
            if index < len(message.position):
                filtered.position.append(message.position[index])
            if index < len(message.velocity):
                filtered.velocity.append(message.velocity[index])
            if index < len(message.effort):
                filtered.effort.append(message.effort[index])
        self._publisher.publish(filtered)


def main() -> None:
    rclpy.init()
    node = JointStateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
