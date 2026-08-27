#!/usr/bin/env python3
"""Synchronize selected Gazebo models with MoveIt's planning scene.

Gazebo publishes model poses in its world frame.  In this simulation the
diff-drive plugin uses the same frame as ``odom`` (at startup), so ``odom`` is
used as the planning-scene frame.  Static obstacles and grasp candidates are
kept as separate CollisionObjects; a grasp executor can later replace the
candidate with an AttachedCollisionObject.
"""

from typing import Dict, Optional

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from shape_msgs.msg import SolidPrimitive


class GazeboPlanningSceneSync(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_planning_scene_sync")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("update_rate", 2.0)
        self.declare_parameter("obstacle_models", ["work_table"])
        self.declare_parameter("pick_models", ["grasp_box"])
        self.declare_parameter("table_size", [1.2, 0.8, 0.8])
        self.declare_parameter("grasp_box_size", [0.05, 0.05, 0.05])

        self._frame = str(self.get_parameter("world_frame").value)
        self._models: Dict[str, str] = {}
        for name in self.get_parameter("obstacle_models").value:
            self._models[str(name)] = "obstacle"
        for name in self.get_parameter("pick_models").value:
            self._models[str(name)] = "pick"
        self._sizes = {
            "work_table": list(self.get_parameter("table_size").value),
            "grasp_box": list(self.get_parameter("grasp_box_size").value),
        }
        self._last_update = self.get_clock().now()
        self._period = 1.0 / max(float(self.get_parameter("update_rate").value), 0.1)
        self._in_flight = False
        self._seen = set()

        self._apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        # gazebo_ros_state publishes ModelStates with BEST_EFFORT QoS.
        model_states_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self._on_models, model_states_qos
        )
        #保存当前已经附着到机器人上的物体ID，避免重复添加
        self._attached_objects = set()

        #监听moveit中的planning sence变化
        self.create_subscription(
            PlanningScene,
            "/planning_scene", 
            self._on_planning_scene,
            10
        )

    def _on_models(self, message: ModelStates) -> None:
        now = self.get_clock().now()
        if self._in_flight or (now - self._last_update).nanoseconds * 1e-9 < self._period:
            return
        index = {name: i for i, name in enumerate(message.name)}
        objects = []
        for model_name, kind in self._models.items():
            if model_name not in index:
                continue
            size = self._sizes.get(model_name)
            if not size or len(size) != 3:
                self.get_logger().error(f"No 3D size configured for model '{model_name}'")
                continue
            object_id = f"gazebo_{model_name}"
            #如果目标被moveit附着到机器人上了，就不再添加到planning scene中
            if object_id in self._attached_objects:
                continue
            objects.append(self._make_box(object_id, message.pose[index[model_name]], size))
            if model_name not in self._seen:
                self.get_logger().info(
                    f"Syncing {kind} model '{model_name}' as CollisionObject '{object_id}'"
                )
                self._seen.add(model_name)
        if not objects or not self._apply.wait_for_service(timeout_sec=0.0):
            return
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = objects
        request = ApplyPlanningScene.Request(scene=scene)
        self._in_flight = True
        self._last_update = now
        future = self._apply.call_async(request)
        future.add_done_callback(self._done)

    def _make_box(self, object_id: str, pose: Pose, size) -> CollisionObject:
        primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])
        header = Header(frame_id=self._frame)
        return CollisionObject(
            header=header,
            id=object_id,
            primitives=[primitive],
            primitive_poses=[pose],
            operation=CollisionObject.ADD,
        )

    def _done(self, future) -> None:
        self._in_flight = False
        try:
            if not future.result().success:
                self.get_logger().warning("MoveIt rejected the planning-scene update")
        except Exception as error:  # service may disappear during shutdown
            self.get_logger().debug(f"Planning-scene update failed: {error}")

    def _on_planning_scene(self, message: PlanningScene) -> None:
        if not message.is_diff:
            self._attached_ids.clear()

        for attached in (
            message.robot_state.attached_collision_objects
        ):
            object_id = attached.object.id
            if (
                attached.object.operation == CollisionObject.REMOVE
            ):
                self._attached_ids.discard(object_id)
            else:
                self._attached_ids.add(object_id)    


def main() -> None:
    rclpy.init()
    node = GazeboPlanningSceneSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
