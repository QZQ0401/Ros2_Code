from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import re
import xacro


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
        /"g4_mobile_depth.gazebo.urdf.xacro"
        )
    # Humble gazebo_ros2_control forwards this text as
    # `--param robot_description:=<text>`. XML declarations/comments can make
    # its rcl argument parser reject the override, so pass only the URDF root.
    robot_description = xacro.process_file(
        str(description), mappings={"prefix": prefix, "use_gravity": use_gravity}).toxml()
    robot_description = re.sub(r"<\?xml[^>]*\?>", "", robot_description)
    robot_description = re.sub(r"<!--.*?-->", "", robot_description, flags=re.DOTALL).strip()
    # controller_yaml = Path(
    #     FindPackageShare("vendor_robot_simulation").perform(context),
    #     "config", "controllers.yaml")
    gazebo_launch = Path(
        FindPackageShare("gazebo_ros").perform(context), "launch", "gazebo.launch.py")
    spawn_entity = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "g4", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen")
    joint_state_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen")
    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen")
    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen")
    joint_state_filter = Node(
        package="vendor_robot_simulation",
        executable="joint_state_filter.py",
        name="joint_state_filter",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={
                "world": LaunchConfiguration("world"),
            }.items(),
        ),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, "use_sim_time": True,}], output="screen"),
        joint_state_filter,
        spawn_entity,
        # gazebo_ros2_control is a model plugin: /controller_manager does not
        # exist until spawn_entity has inserted the URDF into gzserver.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[arm_spawner, gripper_spawner])),
    ]


def generate_launch_description():
    default_world = Path(
        get_package_share_directory("vendor_robot_simulation"),
        "world",
        "pick_scene.world",
    )
    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("use_gravity",  default_value="false", description="是否在 Gazebo 中启用机器人各 Link 的重力"),
        DeclareLaunchArgument(
            "world",
            default_value=str(default_world),
            description="Gazebo world 文件；默认加载带桌子和箱子的 pick_scene.world",
        ),
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
