"""放置在 vendor_robot_simulation/launch 下，启动 Gazebo、ros2_control、MoveIt 2 和 RViz2 联合仿真。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


# 等待 Gazebo 中的机械臂和夹爪控制器完成加载和激活。
# 这样可避免 move_group 启动过早，未发现对应动作服务器。
_WAIT_FOR_CONTROLLER_SCRIPT = r"""
manager="$1"
timeout="$2"
deadline=$(( $(date +%s) + timeout ))

while [ "$(date +%s)" -lt "$deadline" ]; do
  line=$(
    ros2 control list_controllers       --controller-manager "$manager" 2>/dev/null |
    grep -E '^arm_controller[[:space:]].*[[:space:]]active$' || true
  )

  action=$(
    ros2 action list 2>/dev/null |
    grep -E '^/arm_controller/follow_joint_trajectory$' || true
  )

  gripper_line=$(
    ros2 control list_controllers       --controller-manager "$manager" 2>/dev/null |
    grep -E '^gripper_controller[[:space:]].*[[:space:]]active$' || true
  )

  gripper_action=$(
    ros2 action list 2>/dev/null |
    grep -E '^/gripper_controller/gripper_cmd$' || true
  )

  if [ -n "$line" ] && [ -n "$action" ] && [ -n "$gripper_line" ] && [ -n "$gripper_action" ]; then
    echo "[wait_for_gazebo_controller] 机械臂控制器已激活: $line"
    echo "[wait_for_gazebo_controller] 机械臂动作服务器已就绪: $action"
    echo "[wait_for_gazebo_controller] 夹爪控制器已激活: $gripper_line"
    echo "[wait_for_gazebo_controller] 夹爪动作服务器已就绪: $gripper_action"
    exit 0
  fi

  sleep 0.5
done

echo "[wait_for_gazebo_controller] 等待 ${timeout}s 超时。" >&2
echo "[wait_for_gazebo_controller] 仍将启动 MoveIt，" >&2
echo "[wait_for_gazebo_controller] 但执行前请确认 arm_controller 和 gripper_controller 均为 active。" >&2
exit 0
"""


def _as_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in ("true", "false", "1", "0"):
        raise RuntimeError(f"{name} 必须为 true 或 false")
    return normalized in ("true", "1")


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _setup(context):
    prefix = _value(context, "prefix")
    use_gravity = _value(context, "use_gravity")
    world = _value(context, "world")
    start_rviz = _as_bool(_value(context, "start_rviz"), "start_rviz")
    start_move_l_server = _as_bool(
        _value(context, "start_move_l_server"),
        "start_move_l_server",
    )
    allow_trajectory_execution = _as_bool(
        _value(context, "allow_trajectory_execution"),
        "allow_trajectory_execution",
    )
    wait_timeout = _value(context, "wait_for_controller_timeout")

    if prefix:
        raise RuntimeError(
            "当前 Gazebo controllers.yaml 和 MoveIt 控制器配置未使用关节前缀，"
            "请保持 prefix 为空。"
        )

    if not wait_timeout.isdigit() or int(wait_timeout) <= 0:
        raise RuntimeError("wait_for_controller_timeout 必须是正整数秒数")

    simulation_share = Path(
        get_package_share_directory("vendor_robot_simulation")
    )
    moveit_share = Path(
        get_package_share_directory("vendor_robot_moveit_config")
    )

    # 复用已有的 Gazebo + ros2_control 独立启动文件。
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / "launch" / "simulation.launch.py")
        ),
        launch_arguments={
            "prefix": prefix,
            "use_gravity": use_gravity,
            "world": world,
        }.items(),
    )

    # MoveIt 与 Gazebo 必须使用同一组合模型（移动底盘、机械臂和深度相机）。
    simulation_urdf = simulation_share / "urdf" / "g4_mobile_depth.gazebo.urdf.xacro"
    mobile_robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", str(simulation_urdf),
        " prefix:=", prefix, " use_gravity:=", use_gravity,
    ]), value_type=str)
    moveit_config = (
        MoveItConfigsBuilder(
            "G4",
            package_name="vendor_robot_moveit_config",
        )
        .robot_description(file_path="config/G4.urdf.xacro", mappings={"prefix": prefix, "robot_name": "g4_mobile_depth"})
        .robot_description_semantic(
            file_path="config/G4.srdf.xacro",
            # The combined Gazebo URDF is named g4_mobile_depth; SRDF must
            # use the same robot name and explicitly receives this xacro arg.
            mappings={"prefix": prefix, "robot_name": "g4_mobile_depth"},
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .joint_limits(
            file_path="config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .sensors_3d(
            file_path="config/sensors_3d.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    runtime_parameters = {
        # Gazebo、robot_state_publisher、move_group 和 RViz 必须使用同一仿真时钟。
        "use_sim_time": True,
        "allow_trajectory_execution": allow_trajectory_execution,
        # 控制器由 Gazebo 中的 gazebo_ros2_control 管理，MoveIt 不负责启停。
        "moveit_manage_controllers": False,
        #加载MTC执行能力
        "capabilities": "move_group/ExecuteTrajectoryCapability",
        "trajectory_execution.execution_duration_monitoring": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": float(
            _value(context, "allowed_start_tolerance")
        ),
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    common_parameters = [
        moveit_config.to_dict(),
        {"robot_description": mobile_robot_description},
        runtime_parameters,
    ]

    moveit_nodes = [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="vendor_robot_simulation",
            executable="gazebo_planning_scene_sync.py",
            name="gazebo_planning_scene_sync",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"world_frame": "odom"},
                {"obstacle_models": ["work_table"]},
                {"pick_models": ["grasp_box"]},
                {"table_size": [1.2, 0.8, 0.8]},
                {"grasp_box_size": [0.05, 0.05, 0.05]},
            ],
        ),
        # Node(
        #     package="vendor_robot_simulation",
        #     executable="mtc_task_demo",
        #     name="mtc_task_demo",
        #     output="screen",
        #     parameters=common_parameters,
        # )
    ]

    if start_move_l_server:
        moveit_nodes.append(
            Node(
                package="vendor_robot_moveit_config",
                executable="move_l_action_server",
                output="screen",
                parameters=common_parameters
                + [
                    {
                        "group_name": "arm",
                        "min_cartesian_fraction": 0.995,
                        "eef_step": 0.005,
                        "workspace_radius": 1.5,
                    }
                ],
            )
        )

    if start_rviz:
        moveit_nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=[
                    "-d",
                    str(moveit_share / "config" / "moveit.rviz"),
                ],
                parameters=common_parameters,
            )
        )

    waiter = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            _WAIT_FOR_CONTROLLER_SCRIPT,
            "wait_for_gazebo_controller",
            "/controller_manager",
            wait_timeout,
        ],
        output="screen",
    )

    return [
        LogInfo(
            msg=(
                "启动 G4 Gazebo + MoveIt 联合仿真："
                f"use_gravity={use_gravity}, "
                f"trajectory_execution={allow_trajectory_execution}, "
                f"rviz={start_rviz}"
            )
        ),
        simulation_launch,
        waiter,
        RegisterEventHandler(
            OnProcessExit(
                target_action=waiter,
                on_exit=moveit_nodes,
            )
        ),
    ]


def generate_launch_description():
    default_world = Path(
        get_package_share_directory("vendor_robot_simulation"),
        "world",
        "pick_scene.world",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="关节和 Link 前缀；当前单机器人配置应保持为空",
            ),
            DeclareLaunchArgument(
                "use_gravity",
                default_value="false",
                description="是否在 Gazebo 中启用各 Link 的重力",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=str(default_world),
                description="Gazebo world 文件；默认加载带桌子和箱子的 pick_scene.world",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="是否启动 MoveIt RViz2",
            ),
            DeclareLaunchArgument(
                "start_move_l_server",
                default_value="true",
                description="是否启动自定义 MoveL 动作服务器",
            ),
            DeclareLaunchArgument(
                "allow_trajectory_execution",
                default_value="true",
                description="是否允许 MoveIt 向 Gazebo 控制器执行轨迹",
            ),
            DeclareLaunchArgument(
                "wait_for_controller_timeout",
                default_value="60",
                description="等待 Gazebo 的机械臂与夹爪控制器就绪的最长秒数",
            ),
            DeclareLaunchArgument(
                "allowed_start_tolerance",
                default_value="0.05",
                description="MoveIt 执行轨迹时允许的起始关节误差",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
