#!/usr/bin/env python3
"""将指定的 Gazebo 模型与 MoveIt 的规划场景同步。

Gazebo 在其世界坐标系中发布模型位姿。移动底盘固定时，规划场景使用 ``world``；恢复
差速驱动后，启动文件会改用插件发布的 ``odom``。静态障碍物和抓取候选对象作为独立的
CollisionObject 保存；抓取执行器稍后可以将候选对象替换为 AttachedCollisionObject。
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
    """Gazebo 模型与 MoveIt 规划场景的同步节点。

    订阅 Gazebo 的模型状态，将指定的模型作为碰撞对象添加到 MoveIt 的规划场景中。
    同时处理模型被机械臂抓取（附着）时的状态同步。
    """

    def __init__(self) -> None:
        """初始化节点，配置参数、订阅器和客户端。"""
        super().__init__("gazebo_planning_scene_sync")
        # 声明 ROS 参数
        # 固定底盘模式的默认根坐标系；联合启动文件会按底盘模式显式传入此参数。
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("update_rate", 2.0)     # 更新频率（Hz）
        self.declare_parameter("obstacle_models", ["work_table", "obstacle_box"])  # 作为障碍物的模型列表
        # 抓取目标以后由深度相机识别，
        # Gazebo只继续同步环境障碍物。  
        self.declare_parameter("pick_models", [])       # 作为抓取对象的模型列表
        self.declare_parameter("table_size", [2.2, 0.8, 0.8])      # 工作台尺寸 [长, 宽, 高]
        self.declare_parameter("obstacle_box_size", [0.03, 0.55, 0.25])  # 障碍物尺寸
        self.declare_parameter("grasp_box_size", [0.05, 0.05, 0.05])  # 抓取盒子尺寸

        # 读取并存储参数
        self._frame = str(self.get_parameter("world_frame").value)
        # 构建模型名称到类型的映射（"obstacle" 或 "pick"）
        self._models: Dict[str, str] = {}
        for name in self.get_parameter("obstacle_models").value:
            self._models[str(name)] = "obstacle"
        for name in self.get_parameter("pick_models").value:
            self._models[str(name)] = "pick"
        # 存储各模型的尺寸配置
        self._sizes = {
            "work_table": list(self.get_parameter("table_size").value),
            "obstacle_box": list(self.get_parameter("obstacle_box_size").value),
            "grasp_box": list(self.get_parameter("grasp_box_size").value),
        }
        self._last_update = self.get_clock().now()  # 上次更新时间
        self._period = 1.0 / max(float(self.get_parameter("update_rate").value), 0.1)  # 更新周期
        self._in_flight = False  # 是否有正在进行的异步请求
        self._in_flight_object_ids = set()  # 正在请求中的对象 ID 集合
        self._pending_removals = set()  # 待移除的对象 ID 集合（因附着而需移除世界副本）
        self._seen = set()  # 已经处理过的模型名称（避免重复日志）

        # 创建 ApplyPlanningScene 服务的客户端
        self._apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        # gazebo_ros_state 使用 BEST_EFFORT QoS 发布 ModelStates
        model_states_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # 订阅 Gazebo 的模型状态
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self._on_models, model_states_qos
        )

        # 保存当前附着在机器人上的对象 ID，用于从世界同步中排除
        self._attached_objects = set()

        # /monitored_planning_scene 是 move_group 维护的权威规划场景，
        # 包含 MTC 的 AttachedCollisionObject 更新。同时也订阅 /planning_scene
        # 以接收外部发布的差异更新。
        self.create_subscription(
            PlanningScene,
            "/monitored_planning_scene",
            self._on_planning_scene,
            10
        )
        self.create_subscription(
            PlanningScene,
            "/planning_scene",
            self._on_planning_scene,
            10,
        )

    def _on_models(self, message: ModelStates) -> None:
        """处理 Gazebo 模型状态回调。

        根据配置的模型列表，将指定模型的位姿转换为 CollisionObject 并发送给 MoveIt。
        会进行频率控制以避免过高的更新频率，并跳过已经附着到机器人上的对象。

        参数:
            message: Gazebo 发布的 ModelStates 消息
        """
        now = self.get_clock().now()
        # 如果有正在进行的请求，或者未达到更新周期，则跳过
        if self._in_flight or (now - self._last_update).nanoseconds * 1e-9 < self._period:
            return

        # 构建模型名称到索引的映射
        index = {name: i for i, name in enumerate(message.name)}
        objects = []

        for model_name, kind in self._models.items():
            if model_name not in index:
                continue
            size = self._sizes.get(model_name)
            if not size or len(size) != 3:
                self.get_logger().error(f"未为模型 '{model_name}' 配置三维尺寸")
                continue
            object_id = f"gazebo_{model_name}"
            # 如果目标已经被 MoveIt 附着到机器人上，就不再添加到规划场景中
            if object_id in self._attached_objects:
                continue
            objects.append(self._make_box(object_id, message.pose[index[model_name]], size))
            if model_name not in self._seen:
                self.get_logger().info(
                    f"正在同步 {kind} 模型 '{model_name}' 作为 CollisionObject '{object_id}'"
                )
                self._seen.add(model_name)

        if not objects or not self._apply.wait_for_service(timeout_sec=0.0):
            return

        # 构建差异场景并发送请求
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = objects
        request = ApplyPlanningScene.Request(scene=scene)
        self._in_flight = True
        self._in_flight_object_ids = {collision_object.id for collision_object in objects}
        self._last_update = now
        future = self._apply.call_async(request)
        future.add_done_callback(self._done)

    def _make_box(self, object_id: str, pose: Pose, size) -> CollisionObject:
        """创建一个长方体碰撞对象。

        根据给定的 ID、位姿和尺寸，生成一个 CollisionObject 消息，
        操作为 ADD（添加）。

        参数:
            object_id: 碰撞对象的唯一标识符
            pose: 物体的位姿
            size: 物体的尺寸 [长, 宽, 高]

        返回:
            CollisionObject: 构造好的碰撞对象消息
        """
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
        """处理 ApplyPlanningScene 服务请求完成的回调。

        在 ADD 请求完成后，检查是否有需要移除的对象（因附着而需要移除世界副本），
        如果有则执行移除操作。

        参数:
            future: 异步请求的 Future 对象
        """
        self._in_flight = False
        in_flight_object_ids = self._in_flight_object_ids
        self._in_flight_object_ids = set()
        try:
            if not future.result().success:
                self.get_logger().warning("MoveIt 拒绝了规划场景更新")
                self._pending_removals.difference_update(in_flight_object_ids)
                return
        except Exception as error:  # 服务可能在关闭期间消失
            self.get_logger().debug(f"规划场景更新失败: {error}")
            self._pending_removals.difference_update(in_flight_object_ids)
            return

        # 在处理 ADD 请求期间可能报告了附着状态。
        # 在 ADD 完成后，精确移除那些需要移除的世界副本。
        # 对于普通的附着更新，不进行移除：MoveIt 在附着时已经移除了世界对象。
        removals = self._pending_removals.intersection(in_flight_object_ids)
        self._pending_removals.difference_update(in_flight_object_ids)
        for object_id in removals:
            self._remove_world_object(object_id)

    def _remove_world_object(self, object_id: str) -> None:
        """移除陈旧的世界副本（在 MTC 附着更新之后）。

        当 MTC 附着某个对象时，可能已经有 Gazebo 位姿更新正在传输中。
        显式移除世界副本可以关闭这个竞态条件，而无需等待下一个模型状态回调
        来感知附着状态。

        参数:
            object_id: 需要从世界中移除的对象 ID
        """
        if not self._apply.wait_for_service(timeout_sec=0.0):
            return
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [
            CollisionObject(id=object_id, operation=CollisionObject.REMOVE)
        ]
        future = self._apply.call_async(ApplyPlanningScene.Request(scene=scene))
        future.add_done_callback(self._remove_done)

    def _remove_done(self, future) -> None:
        """处理世界对象移除请求完成的回调。

        参数:
            future: 异步请求的 Future 对象
        """
        try:
            if not future.result().success:
                self.get_logger().warning("MoveIt 拒绝了陈旧世界对象的移除")
        except Exception as error:  # 服务可能在关闭期间消失
            self.get_logger().debug(f"世界对象移除失败: {error}")

    def _on_planning_scene(self, message: PlanningScene) -> None:
        """处理 PlanningScene 消息回调。

        监听规划场景更新，特别是附着碰撞对象的变化。
        当发现本节点同步的对象被附着到机器人上时，记录其 ID，
        以便在后续的世界同步中跳过该对象，并在必要时移除其世界副本。

        参数:
            message: PlanningScene 消息
        """
        tracked_ids = {f"gazebo_{model_name}" for model_name in self._models}

        if not message.is_diff:
            # 完整场景是快照，不是连续的附着更新序列。
            # 保持之前的集合，避免从两个规划场景主题收到相同快照时触发重复的 REMOVE 请求。
            previous = self._attached_objects
            current = {
                attached.object.id
                for attached in message.robot_state.attached_collision_objects
                if (
                    attached.object.id in tracked_ids
                    and attached.object.operation != CollisionObject.REMOVE
                )
            }
            newly_attached = current.difference(previous)
            self._attached_objects = current
            for object_id in newly_attached:
                if object_id in self._in_flight_object_ids:
                    self._pending_removals.add(object_id)
            return

        # 处理差异场景中的附着对象更新
        for attached in message.robot_state.attached_collision_objects:
            object_id = attached.object.id
            if object_id not in tracked_ids:
                continue
            if (
                attached.object.operation == CollisionObject.REMOVE
            ):
                # 对象被移除附着
                self._attached_objects.discard(object_id)
            elif object_id not in self._attached_objects:
                # 新对象被附着
                self._attached_objects.add(object_id)
                if object_id in self._in_flight_object_ids:
                    self._pending_removals.add(object_id)


def main() -> None:
    """主函数：初始化 ROS 2，创建节点并进入事件循环。"""
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
