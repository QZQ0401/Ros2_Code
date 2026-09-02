import atexit
from pathlib import Path
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _remove_file(path):
    try:
        Path(path).unlink(missing_ok=True)
    except OSError:
        pass


def _write_controller_parameters(
        namespace, update_rate, joints, trajectory_constraints):
    node_prefix = f"/{namespace}" if namespace else ""
    manager_name = f"{node_prefix}/controller_manager"
    controller_types = {
        "arm_controller":
            "joint_trajectory_controller/JointTrajectoryController",
        "joint_state_broadcaster":
            "joint_state_broadcaster/JointStateBroadcaster",
        "io_and_status_controller":
            "vendor_robot_controllers/IOAndStatusController",
        "driver_status_broadcaster":
            "vendor_robot_controllers/DriverStatusBroadcaster",
        "speed_scaling_broadcaster":
            "vendor_robot_controllers/SpeedScalingBroadcaster",
    }
    parameters = {
        manager_name: {
            "ros__parameters": {
                "update_rate": update_rate,
                **{
                    name: {"type": controller_type}
                    for name, controller_type in controller_types.items()
                },
            },
        },
        f"{node_prefix}/arm_controller": {
            "ros__parameters": {
                "joints": joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "state_publish_rate": 50.0,
                "action_monitor_rate": 20.0,
                "allow_partial_joints_goal": False,
                "open_loop_control": False,
                "allow_nonzero_velocity_at_trajectory_end": False,
                # JTC 容差。servoj 透传的命令永远领先实测，跟随滞后约
                # v/omega_cutoff；原来 trajectory=0.10 rad 在 2 rad/s 下就已经
                # 顶到边界，稍快一点就 PATH_TOLERANCE_VIOLATED、轨迹中途 abort。
                "constraints": {
                    "stopped_velocity_tolerance":
                        trajectory_constraints["stopped_velocity_tolerance"],
                    "goal_time": trajectory_constraints["goal_time"],
                    **{
                        joint: {
                            "trajectory": trajectory_constraints["trajectory"],
                            "goal": trajectory_constraints["goal"],
                        }
                        for joint in joints
                    },
                },
            },
        },
    }
    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix="vendor_robot_controllers_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    ) as parameter_file:
        yaml.safe_dump(parameters, parameter_file, sort_keys=False)
        path = parameter_file.name
    atexit.register(_remove_file, path)
    return path


def _setup(context):
    robot_type = LaunchConfiguration("robot_type").perform(context).strip().lower()
    model_files = {
        "g3": "g3.urdf.xacro",
        "g4": "g4.urdf.xacro",
        "g6": "g6.urdf.xacro",
        "g9": "g9.urdf.xacro", 
        "g12": "g12.urdf.xacro",
        "g18": "g18.urdf.xacro",
        "g20": "g20.urdf.xacro",
        "g25": "g25.urdf.xacro", 
        "g30": "g30.urdf.xacro",
        "g6a": "g6a.urdf.xacro",
        "g6l": "g6l.urdf.xacro",
    }
    if robot_type not in model_files:
        raise RuntimeError(
            f"unsupported robot_type '{robot_type}'; available models: "
            f"{', '.join(sorted(model_files))}")
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("real", "fake"):
        raise RuntimeError(
            "robot.launch.py supports mode:=real or mode:=fake; "
            "use simulation.launch.py for Gazebo")
    prefix = LaunchConfiguration("prefix").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    network_interface = LaunchConfiguration("network_interface").perform(context)
    rtde_frequency = LaunchConfiguration("rtde_frequency").perform(context)
    rtde_protocol_version = LaunchConfiguration(
        "rtde_protocol_version").perform(context)
    initial_state_timeout_ms = LaunchConfiguration(
        "initial_state_timeout_ms").perform(context)
    start_arm_controller_text = LaunchConfiguration(
        "start_arm_controller").perform(context).strip().lower()
    if start_arm_controller_text not in ("true", "false", "1", "0"):
        raise RuntimeError("start_arm_controller must be true or false")
    start_arm_controller = start_arm_controller_text in ("true", "1")
    sdk_port = LaunchConfiguration("sdk_port").perform(context)
    description_share = FindPackageShare("vendor_robot_description")
    initial_positions = Path(
        description_share.perform(context), "config", robot_type,
        "initial_positions.yaml")
    xacro_file = Path(
        description_share.perform(context), "urdf", robot_type,
        model_files[robot_type])
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", str(xacro_file),
        " use_fake_hardware:=", "true" if mode == "fake" else "false",
        " use_simulation:=false",
        " robot_ip:=", robot_ip,
        " network_interface:=", network_interface,
        " rtde_frequency:=", rtde_frequency,
        " rtde_protocol_version:=", rtde_protocol_version,
        " initial_state_timeout_ms:=", initial_state_timeout_ms,
        " prefix:=", prefix,
        " initial_positions_file:=", str(initial_positions),
    ]), value_type=str)
    joints = [f"{prefix}joint{i}" for i in range(1, 7)]
    try:
        update_rate = int(rtde_frequency)
    except ValueError as error:
        raise RuntimeError("rtde_frequency must be an integer") from error
    if update_rate <= 0:
        raise RuntimeError("rtde_frequency must be greater than zero")
    try:
        protocol_version = int(rtde_protocol_version)
        initial_timeout = int(initial_state_timeout_ms)
        sdk_port_value = int(sdk_port)
    except ValueError as error:
        raise RuntimeError(
            "rtde_protocol_version, initial_state_timeout_ms and sdk_port "
            "must be integers") from error
    if protocol_version <= 0:
        raise RuntimeError("rtde_protocol_version must be greater than zero")
    if initial_timeout <= 0:
        raise RuntimeError("initial_state_timeout_ms must be greater than zero")
    if not 1 <= sdk_port_value <= 65535:
        raise RuntimeError("sdk_port must be in the range 1..65535")
    # JTC 容差可通过启动参数覆盖，便于真机现场调整
    trajectory_constraints = {
        "trajectory": float(LaunchConfiguration(
            "trajectory_tolerance").perform(context)),
        "goal": float(LaunchConfiguration("goal_tolerance").perform(context)),
        "goal_time": float(LaunchConfiguration("goal_time").perform(context)),
        "stopped_velocity_tolerance": float(LaunchConfiguration(
            "stopped_velocity_tolerance").perform(context)),
    }
    for name, value in trajectory_constraints.items():
        if value < 0.0:
            raise RuntimeError(f"{name} must not be negative")
    controller_parameters = _write_controller_parameters(
        namespace, update_rate, joints, trajectory_constraints)
    node_prefix = f"/{namespace}" if namespace else ""
    manager = f"{node_prefix}/controller_manager"
    nodes = [
        LogInfo(msg=(
            "vendor_robot_bringup 0.1.13: "
            f"robot_type={robot_type}, mode={mode}, robot_ip={robot_ip}, "
            f"network_interface={network_interface or '<default>'}, "
            f"rtde_frequency={update_rate}, "
            f"rtde_protocol_version={protocol_version}, "
            f"initial_state_timeout_ms={initial_timeout}, "
            f"sdk_port={sdk_port_value}, "
            f"start_arm_controller={start_arm_controller}")),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=namespace, output="screen",
            parameters=[{"robot_description": robot_description}]),
        Node(
            package="controller_manager", executable="ros2_control_node",
            namespace=namespace, output="screen",
            parameters=[controller_parameters],
            respawn=True, respawn_delay=5.0,
            remappings=[("~/robot_description", "robot_description")]),
    ]
    for controller in (
        "joint_state_broadcaster", "driver_status_broadcaster",
        "speed_scaling_broadcaster",
        "io_and_status_controller", "arm_controller"):
        spawner_arguments = [
            controller, "--controller-manager", manager,
            "--controller-manager-timeout", "20"]
        if controller == "arm_controller":
            spawner_arguments.extend(["--param-file", controller_parameters])
            if not start_arm_controller:
                spawner_arguments.append("--inactive")
        nodes.append(Node(
            package="controller_manager", executable="spawner",
            arguments=spawner_arguments,
            output="screen"))

    # controller_manager 配置了 respawn。一次性 spawner 在 manager 重启后不会再次
    # 执行，因此增加轻量 bootstrapper：仅在控制器缺失/manager 重建时重新加载；
    # ControllerStopper 正常停用但控制器仍存在时，绝不会自动重新激活 arm_controller。
    nodes.append(Node(
        package="vendor_robot_controllers",
        executable="controller_bootstrapper_node",
        namespace=namespace,
        output="screen",
        parameters=[{
            "controllers": [
                "joint_state_broadcaster", "driver_status_broadcaster",
                "speed_scaling_broadcaster", "io_and_status_controller",
                "arm_controller"],
            "always_active_controllers": [
                "joint_state_broadcaster", "driver_status_broadcaster",
                "speed_scaling_broadcaster", "io_and_status_controller"],
            "arm_controller": "arm_controller",
            "start_arm_controller": start_arm_controller,
            "poll_period_ms": 2000,
        }]))
    if mode == "real":
        nodes.append(Node(
            package="vendor_robot_controllers", executable="sdk_manager_node",
            namespace=namespace, output="screen",
            parameters=[{
                "robot_ip": robot_ip,
                "sdk_port": sdk_port_value,
                "sdk_password": LaunchConfiguration("sdk_password"),
            }]))
        nodes.append(Node(
            package="vendor_robot_controllers", executable="controller_stopper_node",
            namespace=namespace, output="screen",
            remappings=[
                ("driver_status",
                 f"{node_prefix}/driver_status_broadcaster/driver_status"),
            ],
            parameters=[{
                "enabled": ParameterValue(
                    LaunchConfiguration("enable_controller_stopper"), value_type=bool),
                "auto_restart": ParameterValue(
                    LaunchConfiguration("stopper_auto_restart"), value_type=bool),
                "treat_sdk_motion_as_external": ParameterValue(
                    LaunchConfiguration("stopper_treat_sdk_motion_as_external"),
                    value_type=bool),
                "enforce_period_ms": int(LaunchConfiguration(
                    "stopper_enforce_period_ms").perform(context)),
                "servo_command_timeout_ms": int(LaunchConfiguration(
                    "stopper_servo_command_timeout_ms").perform(context)),
                "switch_timeout_ms": int(LaunchConfiguration(
                    "stopper_switch_timeout_ms").perform(context)),
                "verify_timeout_ms": int(LaunchConfiguration(
                    "stopper_verify_timeout_ms").perform(context)),
                "mode_timeout_ms": int(LaunchConfiguration(
                    "stopper_mode_timeout_ms").perform(context)),
                "startup_grace_ms": int(LaunchConfiguration(
                    "stopper_startup_grace_ms").perform(context)),
                "stop_on_watchdog": ParameterValue(
                    LaunchConfiguration("stopper_stop_on_watchdog"),
                    value_type=bool),
                # 受管控制器。若将来新增其它会驱动关节的控制器，必须一并列入，
                # 否则外部运动源出现时它不会被停用。
                "controllers": ["arm_controller"],
            }]))
    nodes.append(Node(
        package="vendor_robot_controllers", executable="diagnostics_bridge_node",
        namespace=namespace, output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type", default_value="g4",
            description="Robot model to load "),
        DeclareLaunchArgument("mode", default_value="real",
                              description="real or fake"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.6.6"),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("rtde_frequency", default_value="250"),
        DeclareLaunchArgument("rtde_protocol_version", default_value="3"),
        DeclareLaunchArgument("initial_state_timeout_ms", default_value="10000"),
        DeclareLaunchArgument(
            "sdk_port", default_value="2323",
            description="SDK TCP port: 2323 real controller, 2325 virtual controller"),
        DeclareLaunchArgument("sdk_password", default_value=""),
        DeclareLaunchArgument(
            "enable_controller_stopper", default_value="true",
            description=(
                "Automatically deactivate arm_controller when SDK robot/safety "
                "state forbids ROS motion or external motion owns the robot.")),
        # ---- ControllerStopper 参数 (stopper_*) ----
        DeclareLaunchArgument(
            "stopper_auto_restart", default_value="false",
            description=(
                "Ignored by design. Motion controllers are never re-activated "
                "automatically; the operator must activate them manually.")),
        DeclareLaunchArgument(
            "stopper_stop_on_watchdog", default_value="true",
            description=(
                "Deactivate the motion controllers when /robot_mode or "
                "/safety_mode stops being published (SDK link lost).")),
        DeclareLaunchArgument(
            "stopper_treat_sdk_motion_as_external", default_value="true",
            description="Treat SDK_Moving(102) as external motion unless own servoj active"),
        DeclareLaunchArgument(
            "stopper_enforce_period_ms", default_value="100",
            description="Enforcement tick period: watchdog + manual-activation scan"),
        DeclareLaunchArgument(
            "stopper_servo_command_timeout_ms", default_value="300",
            description="Max age of command_fresh to claim 'we are driving' (ms)"),
        DeclareLaunchArgument(
            "stopper_switch_timeout_ms", default_value="500",
            description="Max wait for switch_controller response before retry (ms)"),
        DeclareLaunchArgument(
            "stopper_verify_timeout_ms", default_value="500",
            description="Max wait for list_controllers confirmation before retry (ms)"),
        DeclareLaunchArgument(
            "stopper_mode_timeout_ms", default_value="200",
            description="Watchdog: max age of robot_mode/safety_mode before stop (ms)"),
        DeclareLaunchArgument(
            "stopper_startup_grace_ms", default_value="3000",
            description="Startup grace period before any stops (ms)"),
        # -------------------------------------------------------
        DeclareLaunchArgument(
            "start_arm_controller", default_value="true",
            description=(
                "真机默认自动激活 arm_controller；ControllerStopper 在检测到"
                "外部运动源或通信/安全异常时负责停用，并要求人工恢复。")),
        DeclareLaunchArgument(
            "trajectory_tolerance", default_value="0.05",
            description=(
                "JointTrajectoryController per-joint path tolerance in rad. "
                "servoj pass-through lags behind the command, so this must be "
                "larger than the expected following error.")),
        DeclareLaunchArgument(
            "goal_tolerance", default_value="0.05",
            description="JointTrajectoryController per-joint goal tolerance in rad"),
        DeclareLaunchArgument(
            "goal_time", default_value="2.0",
            description="Extra time allowed for the robot to settle at the goal"),
        DeclareLaunchArgument(
            "stopped_velocity_tolerance", default_value="0.05",
            description="Velocity below which the robot counts as stopped (rad/s)"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("prefix", default_value=""),
        OpaqueFunction(function=_setup),
    ])
