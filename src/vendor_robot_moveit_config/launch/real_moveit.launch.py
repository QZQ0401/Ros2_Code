"""Start the real G4 driver and a matching MoveIt 2 move_group."""

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


# ---------------------------------------------------------------------------
# move_group 启动闸门
#
# MoveItSimpleControllerManager 只在 move_group **启动那一刻** 建立
# FollowJointTrajectory 动作客户端。真机模式下 controller_manager 要先完成
# RTDE 连接（initial_state_timeout_ms 默认 10 s）才能加载 arm_controller，
# 如果 move_group 抢先起来并且没等到动作服务器，它就会永久性地认为
# "没有任何控制器能驱动这些关节"，之后每次 execute 都在收到 goal 后
# 1 ms 内 abort —— 现象与控制器被停用完全一样，但重启 move_group 才能恢复。
#
# 这里在启动 move_group 之前先等 arm_controller 出现（可选：等到 active）。
# 超时后仍然继续启动，只是打印告警，不会把整个 launch 卡死。
# ---------------------------------------------------------------------------
_WAIT_FOR_CONTROLLER_SCRIPT = r"""
manager="$1"; require_active="$2"; timeout="$3"
deadline=$(( $(date +%s) + timeout ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  line=$(ros2 control list_controllers --controller-manager "$manager" 2>/dev/null \
         | grep -E '^arm_controller' || true)
  if [ -n "$line" ]; then
    if [ "$require_active" != "true" ]; then
      echo "[wait_for_arm_controller] loaded: $line"; exit 0
    fi
    # 状态是行尾最后一个字段。不能用 *active* 通配，"inactive" 也会命中。
    state=$(echo "$line" | awk '{print $NF}')
    if [ "$state" = "active" ]; then
      echo "[wait_for_arm_controller] active: $line"; exit 0
    fi
  fi
  sleep 0.5
done
echo "[wait_for_arm_controller] TIMEOUT after ${timeout}s." >&2
echo "[wait_for_arm_controller] move_group will start anyway, but if" >&2
echo "[wait_for_arm_controller] arm_controller shows up later, MoveIt execute()" >&2
echo "[wait_for_arm_controller] may abort instantly. Restart move_group in that case." >&2
exit 0
"""


def _as_bool(value, name):
    normalized = value.strip().lower()
    if normalized not in ("true", "false", "1", "0"):
        raise RuntimeError(f"{name} must be true or false")
    return normalized in ("true", "1")


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _setup(context):
    robot_ip = _value(context, "robot_ip")
    network_interface = _value(context, "network_interface")
    rtde_frequency = _value(context, "rtde_frequency")
    rtde_protocol_version = _value(context, "rtde_protocol_version")
    initial_state_timeout_ms = _value(context, "initial_state_timeout_ms")
    sdk_port = _value(context, "sdk_port")
    sdk_password = _value(context, "sdk_password")
    namespace = _value(context, "namespace").strip("/")
    prefix = _value(context, "prefix")
    start_arm_controller = _as_bool(
        _value(context, "start_arm_controller"), "start_arm_controller")
    enable_controller_stopper = _as_bool(
        _value(context, "enable_controller_stopper"),
        "enable_controller_stopper")
    start_rviz = _as_bool(_value(context, "start_rviz"), "start_rviz")
    start_move_l_server = _as_bool(
        _value(context, "start_move_l_server"), "start_move_l_server")
    allow_trajectory_execution = _as_bool(
        _value(context, "allow_trajectory_execution"),
        "allow_trajectory_execution")
    wait_for_arm_controller = _as_bool(
        _value(context, "wait_for_arm_controller"), "wait_for_arm_controller")
    wait_timeout = _value(context, "wait_for_arm_controller_timeout")
    if not wait_timeout.isdigit() or int(wait_timeout) <= 0:
        raise RuntimeError(
            "wait_for_arm_controller_timeout must be a positive integer "
            "number of seconds")

    description_mappings = {
        "prefix": prefix,
        "use_fake_hardware": "false",
        "use_simulation": "false",
        "robot_ip": robot_ip,
        "network_interface": network_interface,
        "rtde_frequency": rtde_frequency,
        "rtde_protocol_version": rtde_protocol_version,
        "initial_state_timeout_ms": initial_state_timeout_ms,
    }
    moveit_config = (
        MoveItConfigsBuilder(
            "G4", package_name="vendor_robot_moveit_config")
        .robot_description(
            file_path="config/G4.urdf.xacro",
            mappings=description_mappings)
        .robot_description_semantic(
            file_path="config/G4.srdf.xacro",
            mappings={"prefix": prefix})
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    joints = [f"{prefix}joint{index}" for index in range(1, 7)]
    controller_parameters = {
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["arm_controller"],
            "arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": joints,
            },
        },
    }
    runtime_parameters = {
        "use_sim_time": False,
        "allow_trajectory_execution": allow_trajectory_execution,
        "moveit_manage_controllers": False,
        "monitor_dynamics": False,
        # 执行时长监控放宽：透传伺服到位后还有一段稳定时间，
        # 原来的 1.2 倍 + 0.5 s 余量容易被判执行超时
        "trajectory_execution.execution_duration_monitoring": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        # 起始点容差。若日志出现 "start point deviates from current robot state"，
        # 说明规划起点与当前状态对不上，把这个值调大或改用当前状态重新规划。
        "trajectory_execution.allowed_start_tolerance": float(
            _value(context, "allowed_start_tolerance")),
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    bringup_share = Path(
        get_package_share_directory("vendor_robot_bringup"))
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "robot.launch.py")),
        launch_arguments={
            "mode": "real",
            "robot_ip": robot_ip,
            "network_interface": network_interface,
            "rtde_frequency": rtde_frequency,
            "rtde_protocol_version": rtde_protocol_version,
            "initial_state_timeout_ms": initial_state_timeout_ms,
            "sdk_port": sdk_port,
            "sdk_password": sdk_password,
            "enable_controller_stopper":
                "true" if enable_controller_stopper else "false",
            "start_arm_controller":
                "true" if start_arm_controller else "false",
            "namespace": namespace,
            "prefix": prefix,
            # JTC 容差透传给 bringup，方便真机现场统一调整
            "trajectory_tolerance": _value(context, "trajectory_tolerance"),
            "goal_tolerance": _value(context, "goal_tolerance"),
            "goal_time": _value(context, "goal_time"),
            "stopped_velocity_tolerance": _value(
                context, "stopped_velocity_tolerance"),
        }.items(),
    )

    common_parameters = [
        moveit_config.to_dict(),
        controller_parameters,
        runtime_parameters,
    ]
    manager = f"/{namespace}/controller_manager" if namespace \
        else "/controller_manager"
    gated_nodes = [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=namespace,
            output="screen",
            parameters=common_parameters,
        ),
    ]

    if start_move_l_server:
        gated_nodes.append(Node(
            package="vendor_robot_moveit_config",
            executable="move_l_action_server",
            namespace=namespace,
            output="screen",
            parameters=common_parameters + [{
                "group_name": "arm",
                "min_cartesian_fraction": 0.995,
                "eef_step": 0.005,
                "workspace_radius": 1.5,
            }],
        ))

    if start_rviz:
        rviz_config = Path(
            get_package_share_directory("vendor_robot_moveit_config"),
            "config", "moveit.rviz")
        gated_nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            namespace=namespace,
            output="log",
            arguments=["-d", str(rviz_config)],
            parameters=common_parameters,
        ))

    actions = [
        LogInfo(msg=(
            "Starting real G4 + MoveIt: "
            f"robot_ip={robot_ip}, namespace=/{namespace}, prefix={prefix!r}, "
            f"arm_active={start_arm_controller}, "
            f"execution={allow_trajectory_execution}, "
            f"wait_for_arm_controller={wait_for_arm_controller}")),
        bringup,
    ]
    if wait_for_arm_controller:
        waiter = ExecuteProcess(
            cmd=[
                "bash", "-c", _WAIT_FOR_CONTROLLER_SCRIPT, "wait_for_arm_controller",
                manager, "true" if start_arm_controller else "false", wait_timeout,
            ],
            output="screen",
        )
        actions.append(waiter)
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=waiter, on_exit=gated_nodes)))
    else:
        actions.extend(gated_nodes)

    if not start_arm_controller and allow_trajectory_execution:
        actions.append(LogInfo(msg=(
            "arm_controller is inactive: planning works, but execution will "
            "fail until the controller is activated after robot/safety checks.")))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.6.6"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("rtde_frequency", default_value="250"),
        DeclareLaunchArgument("rtde_protocol_version", default_value="3"),
        DeclareLaunchArgument(
            "initial_state_timeout_ms", default_value="10000"),
        DeclareLaunchArgument("sdk_port", default_value="2323"),
        DeclareLaunchArgument("sdk_password", default_value=""),
        DeclareLaunchArgument(
            "enable_controller_stopper", default_value="true"),
        DeclareLaunchArgument(
            "start_arm_controller", default_value="true",
            description="真机默认自动激活 arm_controller；异常停用后需人工恢复"),
        DeclareLaunchArgument(
            "allow_trajectory_execution", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_move_l_server", default_value="true"),
        DeclareLaunchArgument(
            "wait_for_arm_controller", default_value="false",
            description=(
                "Start move_group only after arm_controller shows up in "
                "controller_manager. Prevents MoveItSimpleControllerManager "
                "from permanently registering zero controllers.")),
        DeclareLaunchArgument(
            "wait_for_arm_controller_timeout", default_value="60"),
        DeclareLaunchArgument("trajectory_tolerance", default_value="0.35"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.05"),
        DeclareLaunchArgument("goal_time", default_value="2.0"),
        DeclareLaunchArgument(
            "stopped_velocity_tolerance", default_value="0.05"),
        DeclareLaunchArgument("allowed_start_tolerance", default_value="0.05"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("prefix", default_value=""),
        OpaqueFunction(function=_setup),
    ])
