"""单独启动 MTC 抓取节点，并加载完整 MoveIt 2 配置。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def _setup(context):
    """根据启动参数构建 MoveItConfigs 并启动 MTC 节点。"""

    prefix = LaunchConfiguration("prefix").perform(context)
    use_gravity = LaunchConfiguration("use_gravity").perform(context)
    post_grasp_settle_time = LaunchConfiguration(
        "post_grasp_settle_time"
    )

    simulation_share = Path(
        get_package_share_directory(
            "vendor_robot_simulation"
        )
    )

    # ============================================================
    # 1. 构建 MoveIt 配置
    #
    # 这里主要加载：
    # - SRDF
    # - kinematics.yaml
    # - joint_limits.yaml
    # - OMPL
    # - MoveIt controller 配置
    # ============================================================

    moveit_config = (
        MoveItConfigsBuilder(
            "G4",
            package_name="vendor_robot_moveit_config",
        )

        # MoveItConfig包自身的URDF配置
        # 后面会被Gazebo实际使用的组合URDF覆盖
        .robot_description(
            file_path="config/G4.urdf.xacro",
            mappings={
                "prefix": prefix,
            },
        )

        # SRDF
        #
        # 如果你已经将SRDF改成：
        #
        # <xacro:arg name="robot_name" default="G4"/>
        # <robot name="$(arg robot_name)">
        #
        # 则这里传入g4_mobile_depth
        .robot_description_semantic(
            file_path="config/G4.srdf.xacro",
            mappings={
                "prefix": prefix,
                "robot_name": "g4_mobile_depth",
            },
        )

        # IK配置
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )

        # 关节限制
        .joint_limits(
            file_path="config/joint_limits.yaml"
        )

        # MoveIt控制器配置
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )

        # 明确加载OMPL
        .planning_pipelines(
            pipelines=["ompl"]
        )

        .to_moveit_configs()
    )

    # ============================================================
    # 2. 加载Gazebo实际运行的完整机器人URDF
    #
    # 你的Gazebo不是单独G4机械臂，而是：
    #
    # g4_mobile_depth
    #
    # 所以MTC必须和move_group使用完全相同的robot_description。
    # ============================================================

    simulation_urdf = (
        simulation_share
        / "urdf"
        / "g4_mobile_depth.gazebo.urdf.xacro"
    )

    mobile_robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(simulation_urdf),
                " prefix:=",
                prefix,
                " use_gravity:=",
                use_gravity,
            ]
        ),
        value_type=str,
    )

    # ============================================================
    # 3. 给MTC节点传递额外运行参数
    # ============================================================

    runtime_parameters = {
        # 与Gazebo使用相同的仿真时钟
        "use_sim_time": True,

        # 允许节点发布robot_description相关信息
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        # Keep the arm still after the gripper closes so Gazebo's contact
        # solver can settle before the lift trajectory starts.
        "post_grasp_settle_time": ParameterValue(
            post_grasp_settle_time,
            value_type=float,
        ),
    }

    # ============================================================
    # 4. 参数顺序非常重要
    #
    # moveit_config.to_dict() 中会带一个robot_description，
    # 后面的mobile_robot_description会覆盖它。
    #
    # 最终MTC获得的是Gazebo正在使用的完整机器人模型。
    # ============================================================

    mtc_parameters = [
        moveit_config.to_dict(),

        {
            "robot_description":
                mobile_robot_description
        },

        runtime_parameters,
    ]

    # ============================================================
    # 5. 启动MTC
    # ============================================================

    mtc_node = Node(
        package="vendor_robot_simulation",
        executable="mtc_task_demo",
        name="mtc_pick_demo",
        output="screen",
        parameters=mtc_parameters,
    )

    return [mtc_node]


def generate_launch_description():
    """生成ROS 2 LaunchDescription。"""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="机器人关节和Link前缀",
            ),

            DeclareLaunchArgument(
                "use_gravity",
                default_value="false",
                description="是否启用Gazebo机器人Link重力",
            ),

            DeclareLaunchArgument(
                "post_grasp_settle_time",
                default_value="1.0",
                description="闭爪后、抬升前的Gazebo接触稳定等待时间（秒）",
            ),

            OpaqueFunction(
                function=_setup
            ),
        ]
    )
