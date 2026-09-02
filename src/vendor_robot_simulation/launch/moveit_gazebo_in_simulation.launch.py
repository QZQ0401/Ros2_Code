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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# 等待 Gazebo 中的 arm_controller 完成加载和激活。
# 这样可以避免 move_group 启动过早，未发现 FollowJointTrajectory 动作服务器。
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

  if [ -n "$line" ] && [ -n "$action" ]; then
    echo "[wait_for_gazebo_controller] 控制器已激活: $line"
    echo "[wait_for_gazebo_controller] 动作服务器已就绪: $action"
    exit 0
  fi

  sleep 0.5
done

echo "[wait_for_gazebo_controller] 等待 ${timeout}s 超时。" >&2
echo "[wait_for_gazebo_controller] 仍将启动 MoveIt，" >&2
echo "[wait_for_gazebo_controller] 但执行轨迹前请确认 arm_controller 为 active。" >&2
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
    robot_type = _value(context, "robot_type").strip().lower()
    model_configs = {
        "g3": ("G3", "config/g3/G3.urdf.xacro", "config/g3/G3.srdf.xacro"),
        "g4": ("G4", f"config/g4/G4.urdf.xacro", f"config/g4/G4.srdf.xacro"),
        "g6": ("G6", f"config/g6/G6.urdf.xacro", f"config/g6/G6.srdf.xacro"),
        "g6a": ("G6a", "config/g6a/G6a.urdf.xacro", "config/g6a/G6a.srdf.xacro"),
        "g6l": ("G6l", "config/g6l/G6l.urdf.xacro", "config/g6l/G6l.srdf.xacro"),
        "g9": ("G9", "config/g9/G9.urdf.xacro", "config/g9/G9.srdf.xacro"),
        "g12": ("G12", "config/g12/G12.urdf.xacro", "config/g12/G12.srdf.xacro"),
        "g18": ("G18", "config/g18/G18.urdf.xacro", "config/g18/G18.srdf.xacro"),
        "g20": ("G20", "config/g20/G20.urdf.xacro", "config/g20/G20.srdf.xacro"),
        "g25": ("G25", "config/g25/G25.urdf.xacro", "config/g25/G25.srdf.xacro"),
        "g30": ("G30", "config/g30/G30.urdf.xacro", "config/g30/G30.srdf.xacro"),
    }
    if robot_type not in model_configs:
        raise RuntimeError("robot_type must be one of g3, g4, g6, g6a, g6l, g9, g12, g18, g20, g25, g30")
    robot_name, urdf_file, srdf_file = model_configs[robot_type]
    prefix = _value(context, "prefix")
    use_gravity = _value(context, "use_gravity")
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
            "robot_type": robot_type,
            "use_gravity": use_gravity,
        }.items(),
    )

    # MoveIt 使用通用机器人运动学模型。
    # Gazebo 节点仍由 simulation.launch.py 使用 Gazebo 专用 URDF。
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name,
            package_name="vendor_robot_moveit_config",
        )
        .robot_description(
            file_path=urdf_file,
            mappings={
                "prefix": prefix,
                "use_fake_hardware": "true",
                "use_simulation": "false",
            },
        )
        .robot_description_semantic(
            file_path=srdf_file,
            mappings={"prefix": prefix},
        )
        .robot_description_kinematics(
            file_path=f"config/{robot_type}/kinematics.yaml"
        )
        .joint_limits(
            file_path=f"config/{robot_type}/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=f"config/{robot_type}/moveit_controllers.yaml"
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
        runtime_parameters,
    ]

    moveit_nodes = [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=common_parameters,
        ),
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
                    str(moveit_share / "config" / robot_type / "moveit.rviz"),
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
                f"启动 {robot_name} Gazebo + MoveIt 联合仿真："
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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="g4",
                description="机器人型号：g4 或 g6",
            ),
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
                description="等待 Gazebo arm_controller 就绪的最长秒数",
            ),
            DeclareLaunchArgument(
                "allowed_start_tolerance",
                default_value="0.05",
                description="MoveIt 执行轨迹时允许的起始关节误差",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
