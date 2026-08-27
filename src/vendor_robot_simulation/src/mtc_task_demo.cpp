#include <cmath>
#include <memory>
#include <thread>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/move_it_error_codes.hpp>


namespace mtc = moveit::task_constructor;


// ============================================================
// 当前工程中的名称
// ============================================================

static constexpr char ARM_GROUP[] =
    "arm";

static constexpr char HAND_GROUP[] =
    "gripper";

static constexpr char HAND_FRAME[] =
    "gripper_base_link";

static constexpr char OBJECT_ID[] =
    "gazebo_grasp_box";

static constexpr char WORLD_FRAME[] =
    "odom";


class MtcPickNode
{
public:

    explicit MtcPickNode(
        const rclcpp::NodeOptions & options)
    {
        node_ =
            std::make_shared<rclcpp::Node>(
                "mtc_pick_demo",
                options);
    }


    rclcpp::node_interfaces::
        NodeBaseInterface::SharedPtr
    getNodeBaseInterface()
    {
        return node_->
            get_node_base_interface();
    }


    bool waitForObject()
    {
        moveit::planning_interface::
            PlanningSceneInterface scene;

        RCLCPP_INFO(
            node_->get_logger(),
            "等待PlanningScene中的目标物体: %s",
            OBJECT_ID);

        for (int i = 0; i < 100; ++i)
        {
            auto objects =
                scene.getObjects(
                    {OBJECT_ID});

            if (
                objects.find(OBJECT_ID)
                != objects.end())
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "已找到目标物体: %s",
                    OBJECT_ID);

                return true;
            }

            rclcpp::sleep_for(
                std::chrono::milliseconds(
                    100));
        }

        RCLCPP_ERROR(
            node_->get_logger(),
            "PlanningScene中没有找到%s",
            OBJECT_ID);

        return false;
    }


    void run()
    {
        if (!waitForObject())
        {
            return;
        }

        task_ = createTask();

        try
        {
            task_.init();
        }
        catch (
            mtc::InitStageException & error)
        {
            RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                error);

            return;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "开始MTC抓取规划");


        // 最多搜索10个完整可行解
        if (!task_.plan(10))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MTC没有找到完整抓取方案");

            return;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "MTC规划成功，共找到%zu个方案",
            task_.solutions().size());


        // 将最优解发送到RViz的MTC可视化插件
        task_.introspection().
            publishSolution(
                *task_.solutions().front());


        RCLCPP_INFO(
            node_->get_logger(),
            "开始执行MTC抓取任务");


        auto result =
            task_.execute(
                *task_.solutions().front());


        if (
            result.val
            != moveit_msgs::msg::
                MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MTC任务执行失败");

            return;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "抓取任务执行完成");
    }


private:

    mtc::Task createTask()
    {
        mtc::Task task;


        task.stages()->
            setName(
                "G4 pick grasp_box");


        // 从robot_description和SRDF加载机器人模型
        task.loadRobotModel(
            node_);


        // ====================================================
        // 检查Planning Group
        // ====================================================

        const auto * arm_group =
            task.getRobotModel()->
                getJointModelGroup(
                    ARM_GROUP);

        if (!arm_group)
        {
            throw std::runtime_error(
                "SRDF中不存在arm group");
        }


        const auto * hand_group =
            task.getRobotModel()->
                getJointModelGroup(
                    HAND_GROUP);

        if (!hand_group)
        {
            throw std::runtime_error(
                "SRDF中不存在gripper group，"
                "请先增加gripper planning group");
        }


        // ====================================================
        // Task全局属性
        // ====================================================

        task.setProperty(
            "group",
            std::string(ARM_GROUP));

        task.setProperty(
            "eef",
            std::string(HAND_GROUP));

        task.setProperty(
            "ik_frame",
            std::string(HAND_FRAME));


        // ====================================================
        // 当前机器人状态
        // ====================================================

        mtc::Stage *
            current_state_ptr =
                nullptr;


        auto current_state =
            std::make_unique<
                mtc::stages::CurrentState>(
                    "current state");


        current_state_ptr =
            current_state.get();


        task.add(
            std::move(
                current_state));


        // ====================================================
        // 创建Planner
        // ====================================================

        // OMPL负责远距离自由空间规划
        auto sampling_planner =
            std::make_shared<
                mtc::solvers::
                    PipelinePlanner>(
                        node_,
                        "ompl");
        // 默认使用RRTConnectkConfigDefault
        sampling_planner->
            setPlannerId(
                "RRTConnectkConfigDefault");

        // 夹爪开合使用关节插值
        auto interpolation_planner =
            std::make_shared<
                mtc::solvers::
                    JointInterpolationPlanner>();


        // 接近和抬起使用笛卡尔直线
        auto cartesian_planner =
            std::make_shared<
                mtc::solvers::
                    CartesianPath>();


        cartesian_planner->
            setMaxVelocityScalingFactor(
                0.2);

        cartesian_planner->
            setMaxAccelerationScalingFactor(
                0.2);

        cartesian_planner->
            setStepSize(
                0.005);


        // ====================================================
        // 打开夹爪
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::MoveTo>(
                        "open gripper",
                        interpolation_planner);


            stage->setGroup(
                HAND_GROUP);

            stage->setGoal(
                "open");


            task.add(
                std::move(stage));
        }


        // ====================================================
        // Connect：
        // 从当前机械臂状态规划到抓取候选状态附近
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::Connect>(
                        "move to pick",
                        mtc::stages::
                            Connect::
                                GroupPlannerVector{
                                    {
                                        ARM_GROUP,
                                        sampling_planner
                                    }
                                });


            stage->setTimeout(
                10.0);


            stage->properties().
                configureInitFrom(
                    mtc::Stage::PARENT);


            task.add(
                std::move(stage));
        }


        // ====================================================
        // Pick Container
        // ====================================================

        auto pick =
            std::make_unique<
                mtc::SerialContainer>(
                    "pick gazebo_grasp_box");


        task.properties().
            exposeTo(
                pick->properties(),
                {
                    "eef",
                    "group",
                    "ik_frame"
                });


        pick->properties().
            configureInitFrom(
                mtc::Stage::PARENT,
                {
                    "eef",
                    "group",
                    "ik_frame"
                });


        // ====================================================
        // 直线接近物体
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        MoveRelative>(
                            "approach object",
                            cartesian_planner);


            stage->properties().
                configureInitFrom(
                    mtc::Stage::PARENT,
                    {"group"});


            stage->setIKFrame(
                HAND_FRAME);


            // 在方块上方8~15cm处形成pre-grasp
            stage->setMinMaxDistance(
                0.08,
                0.15);


            geometry_msgs::msg::
                Vector3Stamped direction;


            // 当前场景是桌面顶部抓取，
            // 因此直接使用odom的-Z方向向下接近
            direction.header.frame_id =
                WORLD_FRAME;

            direction.vector.z =
                -1.0;


            stage->setDirection(
                direction);


            pick->insert(
                std::move(stage));
        }


        // ====================================================
        // 自动生成抓取姿态
        // ====================================================

        {
            auto grasp_generator =
                std::make_unique<
                    mtc::stages::
                        GenerateGraspPose>(
                            "generate grasp poses");


            grasp_generator->
                properties().
                configureInitFrom(
                    mtc::Stage::PARENT);


            grasp_generator->
                properties().
                set(
                    "marker_ns",
                    "grasp_pose");


            // SRDF中的open状态
            grasp_generator->
                setPreGraspPose(
                    "open");


            // 注意：
            // 使用的是PlanningScene中的ID，
            // 不是Gazebo模型名称
            grasp_generator->
                setObject(
                    OBJECT_ID);


            // 每15°产生一个抓取候选姿态
            grasp_generator->
                setAngleDelta(
                    M_PI / 12.0);


            // 从CurrentState获取当前PlanningScene，
            // 因此能够取得gazebo_grasp_box的位置和姿态
            grasp_generator->
                setMonitoredStage(
                    current_state_ptr);


            // =================================================
            // 抓取中心相对于gripper_base_link的变换
            //
            // 这里是当前工程唯一需要根据真实夹爪模型校准的部分。
            // =================================================

            Eigen::Isometry3d
                grasp_frame_transform =
                    Eigen::Isometry3d::
                        Identity();


            // 示例：
            // 假设夹爪闭合中心距离gripper_base_link约10cm
            grasp_frame_transform.
                translation().z() =
                    0.10;


            // 示例初始姿态：
            // 顶部向下抓取
            //
            // 由于当前上传的zip里没有
            // vendor_robot_description/gripper.xacro，
            // 这里必须按照你的真实夹爪轴方向调整。
            grasp_frame_transform.
                linear() =
                    Eigen::AngleAxisd(
                        M_PI,
                        Eigen::Vector3d::
                            UnitX())
                        .toRotationMatrix();


            auto ik =
                std::make_unique<
                    mtc::stages::ComputeIK>(
                        "compute grasp IK",
                        std::move(
                            grasp_generator));


            // 每个抓取姿态最多找8组IK
            ik->setMaxIKSolutions(
                8);


            ik->setMinSolutionDistance(
                0.5);


            ik->setIKFrame(
                grasp_frame_transform,
                HAND_FRAME);


            ik->properties().
                configureInitFrom(
                    mtc::Stage::PARENT,
                    {
                        "eef",
                        "group"
                    });


            ik->properties().
                configureInitFrom(
                    mtc::Stage::INTERFACE,
                    {
                        "target_pose"
                    });


            pick->insert(
                std::move(ik));
        }


        // ====================================================
        // 允许夹爪与目标方块接触
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        ModifyPlanningScene>(
                            "allow gripper-object collision");


            stage->allowCollisions(
                OBJECT_ID,

                hand_group->
                    getLinkModelNamesWithCollisionGeometry(),

                true);


            pick->insert(
                std::move(stage));
        }


        // ====================================================
        // 闭合夹爪
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::MoveTo>(
                        "close gripper",
                        interpolation_planner);


            stage->setGroup(
                HAND_GROUP);


            // SRDF中的closed状态
            stage->setGoal(
                "closed");


            pick->insert(
                std::move(stage));
        }


        // ====================================================
        // 在MoveIt PlanningScene中附着物体
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        ModifyPlanningScene>(
                            "attach object");


            stage->attachObject(
                OBJECT_ID,
                HAND_FRAME);


            pick->insert(
                std::move(stage));
        }


        // ====================================================
        // 抬起方块
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        MoveRelative>(
                            "lift object",
                            cartesian_planner);


            stage->properties().
                configureInitFrom(
                    mtc::Stage::PARENT,
                    {"group"});


            stage->setIKFrame(
                HAND_FRAME);


            // 抬高10~20cm
            stage->setMinMaxDistance(
                0.10,
                0.20);


            geometry_msgs::msg::
                Vector3Stamped direction;


            // 沿世界坐标Z正方向向上
            direction.header.frame_id =
                WORLD_FRAME;

            direction.vector.z =
                1.0;


            stage->setDirection(
                direction);


            pick->insert(
                std::move(stage));
        }


        task.add(
            std::move(pick));


        return task;
    }


private:

    rclcpp::Node::SharedPtr
        node_;

    mtc::Task
        task_;
};


int main(
    int argc,
    char ** argv)
{
    rclcpp::init(
        argc,
        argv);


    rclcpp::NodeOptions
        options;


    // MTC节点需要接收robot_description、
    // robot_description_semantic、
    // kinematics、OMPL等MoveIt参数
    options.
        automatically_declare_parameters_from_overrides(
            true);


    auto mtc_node =
        std::make_shared<
            MtcPickNode>(
                options);


    rclcpp::executors::
        MultiThreadedExecutor
        executor;


    auto spin_thread =
        std::thread(
            [&]()
            {
                executor.add_node(
                    mtc_node->
                        getNodeBaseInterface());

                executor.spin();
            });


    mtc_node->run();


    // 保持节点运行，
    // 这样RViz仍然可以查看MTC生成的所有候选解
    spin_thread.join();


    rclcpp::shutdown();

    return 0;
}
