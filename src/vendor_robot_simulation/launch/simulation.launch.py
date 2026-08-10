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
    prefix = LaunchConfiguration("prefix").perform(context)
    use_gravity = LaunchConfiguration("use_gravity").perform(context)
    if prefix:
        raise RuntimeError("Gazebo controller YAML is unprefixed; use a separate Gazebo instance per robot")
    simulation_share = Path(
        FindPackageShare("vendor_robot_simulation").perform(context)
    )
    description = Path(
        simulation_share
        /"urdf"
        /"g4.gazebo.urdf.xacro"
        )
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", str(description),
        " ", "prefix:=", prefix, " ", "use_gravity:=", use_gravity,]), value_type=str)
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
             arguments=["-topic", "robot_description", "-entity", "g4", "-x", "0.0", "-y", "0.0", "-z", "0.0",], output="screen"),
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
