from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def _setup(context):
    robot_type = LaunchConfiguration("robot_type").perform(context).strip().lower()
    supported_models = ("g3", "g4", "g6", "g6a", "g6l", "g9", "g12", "g18", "g20", "g25", "g30")
    if robot_type not in supported_models:
        raise RuntimeError(f"robot_type must be one of {supported_models}")
    prefix = LaunchConfiguration("prefix").perform(context)
    use_gravity = LaunchConfiguration("use_gravity").perform(context)
    if prefix:
        raise RuntimeError("Gazebo controller YAML is unprefixed; use a separate Gazebo instance per robot")
    description_share = Path(
        FindPackageShare("vendor_robot_description").perform(context))
    initial_positions = description_share / "config" / robot_type / "initial_positions.yaml"
    controllers = Path(
        FindPackageShare("vendor_robot_simulation").perform(context),
        "config", robot_type, "controllers.yaml")
    if robot_type in supported_models:
        description = Path(FindPackageShare("vendor_robot_simulation").perform(context), "urdf", "gazebo.urdf.xacro")
    else:
        description = description_share / "urdf" / robot_type / f"{robot_type}.urdf.xacro"
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", str(description),
        " ", "prefix:=", prefix, " ", "robot_type:=", robot_type,
        " ", "use_gravity:=", use_gravity,
        " initial_positions_file:=", str(initial_positions),
        " controllers_file:=", str(controllers),
        " use_simulation:=true", " use_fake_hardware:=true"]), value_type=str)
    # controller_yaml = Path(
    #     FindPackageShare("vendor_robot_simulation").perform(context),
    #     "config", "controllers.yaml")
    gazebo_launch = Path(
        FindPackageShare("gazebo_ros").perform(context), "launch", "gazebo.launch.py")
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_launch))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, "use_sim_time": True,}], output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-topic", "robot_description", "-entity", robot_type, "-x", "0.0", "-y", "0.0", "-z", "0.0",], output="screen"),
        # Node(package="controller_manager", executable="ros2_control_node",
        #      parameters=[{"robot_description": robot_description},
        #                  str(controller_yaml)],
        #      output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller", "--controller-manager", "/controller_manager"]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="g4"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("use_gravity",  default_value="false", description="是否在 Gazebo 中启用机器人各 Link 的重力"),
        OpaqueFunction(function=_setup),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=["pkill", "-9", "gzserver"],
                        shell=False,
                    ),
                    ExecuteProcess(
                        cmd=["pkill", "-9", "gzclient"],
                        shell=False,
                    ),
                ]
            )
        ),
    ])
