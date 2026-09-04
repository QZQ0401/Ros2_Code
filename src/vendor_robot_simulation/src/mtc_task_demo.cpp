#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
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
    "gripper_tcp";

static constexpr char OBJECT_ID[] =
    "vision_target";

static constexpr char WORK_TABLE_ID[] =
    "gazebo_work_table";

// 移动底盘固定时，URDF 根节点为 world，且差速插件不会再发布 odom。
// 抓取接近方向和物体位姿必须使用同一坐标系，避免出现 odom -> world
// 变换缺失并被 MoveIt 回退为单位变换的错误。
static constexpr char WORLD_FRAME[] =
    "world";


static constexpr char PLACE_POSE_TOPIC[] =
    "/perception/place_pose";


class MtcPickNode
{
public:

    explicit MtcPickNode(
        const rclcpp::NodeOptions & options)
    {
        node_ =
            std::make_shared<rclcpp::Node>(
                "mtc_pick_place_demo",
                options);


        // bin_estimator 使用 transient_local 发布 place_pose，
        // 这里使用相同 QoS，MTC 即使稍晚启动也能收到最后一次有效位姿。
        const auto place_pose_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();


        place_pose_sub_ =
            node_->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                    PLACE_POSE_TOPIC,
                    place_pose_qos,
                    [this](
                        const geometry_msgs::msg::
                            PoseStamped::SharedPtr message)
                    {
                        std::lock_guard<std::mutex>
                            lock(
                                place_pose_mutex_);

                        place_pose_ =
                            *message;

                        has_place_pose_ =
                            true;
                    });
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


    bool waitForPlacePose()
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "等待放置位姿: %s",
            PLACE_POSE_TOPIC);


        for (int i = 0; i < 100; ++i)
        {
            geometry_msgs::msg::PoseStamped
                pose;


            bool available =
                false;


            {
                std::lock_guard<std::mutex>
                    lock(
                        place_pose_mutex_);


                if (has_place_pose_)
                {
                    pose =
                        place_pose_;

                    available =
                        true;
                }
            }


            if (available)
            {
                const auto & q =
                    pose.pose.orientation;


                const double q_norm =
                    std::sqrt(
                        q.x * q.x +
                        q.y * q.y +
                        q.z * q.z +
                        q.w * q.w);


                if (
                    !pose.header.frame_id.empty() &&
                    std::isfinite(
                        pose.pose.position.x) &&
                    std::isfinite(
                        pose.pose.position.y) &&
                    std::isfinite(
                        pose.pose.position.z) &&
                    std::isfinite(q_norm) &&
                    q_norm > 1e-6)
                {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "已收到放置位姿: frame=%s, "
                        "xyz=(%.3f, %.3f, %.3f)",
                        pose.header.frame_id.c_str(),
                        pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z);

                    return true;
                }
            }


            rclcpp::sleep_for(
                std::chrono::milliseconds(
                    100));
        }


        RCLCPP_ERROR(
            node_->get_logger(),
            "没有收到有效的%s",
            PLACE_POSE_TOPIC);


        return false;
    }


    void run()
    {
        if (!waitForObject())
        {
            return;
        }


        if (!waitForPlacePose())
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
            "开始MTC抓取 + 放置规划");


        // 最多搜索10个完整可行解
        if (!task_.plan(10))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "MTC没有找到完整抓取 + 放置方案");

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
            "开始执行MTC抓取 + 放置任务");


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
            "抓取 + 放置任务执行完成");
    }


private:

    mtc::Task createTask()
    {
        mtc::Task task;


        task.stages()->
            setName(
                "G4 pick and place vision_target");


        // place_pose 表示“物体最终中心位姿”，不是 gripper_tcp 位姿。
        // GeneratePlacePose + attach stage 会自动利用抓取后已经建立的
        // object <-> gripper attachment relation 求对应的机械臂IK。
        geometry_msgs::msg::PoseStamped
            place_pose;


        {
            std::lock_guard<std::mutex>
                lock(
                    place_pose_mutex_);


            if (!has_place_pose_)
            {
                throw std::runtime_error(
                    "createTask时没有有效place_pose");
            }


            place_pose =
                place_pose_;
        }


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


        // GeneratePlacePose需要监视attach stage，
        // 以知道抓取完成后 OBJECT_ID 相对于 gripper_tcp 的实际变换。
        mtc::Stage *
            attach_object_stage_ptr =
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

        sampling_planner->
            setMaxVelocityScalingFactor(
                0.2);

        sampling_planner->
            setMaxAccelerationScalingFactor(
                0.2);

        // 夹爪开合使用关节插值
        auto interpolation_planner =
            std::make_shared<
                mtc::solvers::
                    JointInterpolationPlanner>();

        interpolation_planner->
            setMaxVelocityScalingFactor(
                0.2);
        interpolation_planner->
            setMaxAccelerationScalingFactor(
                0.2);

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


            // 在方块上方20-25cm处形成pre-grasp
            stage->setMinMaxDistance(
                0.15,
                0.25);


            geometry_msgs::msg::
                Vector3Stamped direction;


            // 当前场景是桌面顶部抓取，固定底盘模式下直接使用 world 的
            // -Z 方向向下接近，和规划场景碰撞物体的坐标系保持一致。
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
            // 相对于 gripper_tcp 的抓取变换
            //
            // gripper_tcp 已在 gripper.xacro 中校准为手指夹持中心，故这里
            // 必须保持零平移；若再次填入 gripper_base_link 的偏置，会使 TCP
            // 偏移两次，造成真实夹爪停在物块上方或下方。
            // =================================================

            Eigen::Isometry3d
                grasp_frame_transform =
                    Eigen::Isometry3d::
                        Identity();


            // 物块中心与 gripper_tcp 重合，实际手指位置由 TCP 固定关节给出。
            grasp_frame_transform.
                translation().z() =
                    0.0;


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

            // SRDF 的 gripper 组只包含左侧的驱动关节。左右手指都是固定
            // 子 Link 或 mimic Link，不能依赖 hand_group 一定收集完整；
            // 因此下面显式列出两侧导轨和手指，使闭合时允许它们接触目标。
            auto gripper_collision_links =
                hand_group->
                    getLinkModelNamesWithCollisionGeometry();

            gripper_collision_links.emplace_back(
                "gripper_left_link");

            gripper_collision_links.emplace_back(
                "gripper_left_finger");

            gripper_collision_links.emplace_back(
                "gripper_right_link");

            gripper_collision_links.emplace_back(
                "gripper_right_finger");

            stage->allowCollisions(
                OBJECT_ID,

                gripper_collision_links,

                true);

            // The cube starts in physical contact with the table. Gazebo and
            // FCL both treat this zero-clearance support contact as a
            // collision; keep it allowed until the following lift stage.
            stage->allowCollisions(
                OBJECT_ID,
                WORK_TABLE_ID,
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


            // 对 5 cm 方块使用 SRDF 的 grasp 状态。该状态允许手指与目标
            // 轻微接触以模拟夹持；closed 是空载时两指几乎相合的状态，不能使用。
            stage->setGoal(
                "grasp");

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


            attach_object_stage_ptr =
                stage.get();


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


        // ====================================================
        // 物体已经离开桌面后，恢复目标物体与桌面的碰撞检测
        //
        // 前面的 grasp 阶段为了允许零间隙支撑接触临时允许了
        // OBJECT_ID <-> WORK_TABLE_ID 碰撞。如果不恢复，后续
        // move-to-place 可能规划出穿过桌面的轨迹。
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        ModifyPlanningScene>(
                            "forbid object-table collision");


            stage->allowCollisions(
                OBJECT_ID,
                WORK_TABLE_ID,
                false);


            pick->insert(
                std::move(stage));
        }


        task.add(
            std::move(pick));


        // ====================================================
        // Connect：从抬起后的抓取状态移动到筐子上方的pre-place
        //
        // 后面的 lower object 是一个反向可传播的 MoveRelative，
        // 因此 Connect 会连接到“最终place pose上方”的状态，而不是
        // 直接从侧面规划进筐子。
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::Connect>(
                        "move to place",
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
        // Place Container
        // ====================================================

        auto place =
            std::make_unique<
                mtc::SerialContainer>(
                    "place object into bin");


        task.properties().
            exposeTo(
                place->properties(),
                {
                    "eef",
                    "group",
                    "ik_frame"
                });


        place->properties().
            configureInitFrom(
                mtc::Stage::PARENT,
                {
                    "eef",
                    "group",
                    "ik_frame"
                });


        // ====================================================
        // 从筐子上方向下直线进入
        //
        // 该stage位于GeneratePlacePose之前，因此MTC从最终放置状态
        // 反向传播得到pre-place状态：最终执行时就是从上方向下运动。
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        MoveRelative>(
                            "lower object into bin",
                            cartesian_planner);


            stage->properties().
                configureInitFrom(
                    mtc::Stage::PARENT,
                    {"group"});


            stage->properties().
                set(
                    "marker_ns",
                    "lower_object");


            stage->setIKFrame(
                HAND_FRAME);


            // 筐壁高约0.16m。pre-place保持在开口上方，
            // 再沿-Z进入。根据实际筐高和机器人可达性可继续调节。
            stage->setMinMaxDistance(
                0.10,
                0.18);


            geometry_msgs::msg::
                Vector3Stamped direction;


            direction.header.frame_id =
                WORLD_FRAME;

            direction.vector.z =
                -1.0;


            stage->setDirection(
                direction);


            place->insert(
                std::move(stage));
        }


        // ====================================================
        // 根据/perception/place_pose生成最终物体放置状态 + IK
        //
        // place_pose是OBJECT_ID最终期望位姿。
        // GeneratePlacePose监视attach stage，因此知道当前物体与夹爪
        // 的固定抓取变换，不需要把place_pose手工换算成TCP位姿。
        // ====================================================

        {
            auto place_generator =
                std::make_unique<
                    mtc::stages::
                        GeneratePlacePose>(
                            "generate place pose");


            place_generator->
                properties().
                configureInitFrom(
                    mtc::Stage::PARENT);


            place_generator->
                properties().
                set(
                    "marker_ns",
                    "place_pose");


            place_generator->
                setObject(
                    OBJECT_ID);


            place_generator->
                setPose(
                    place_pose);


            place_generator->
                setMonitoredStage(
                    attach_object_stage_ptr);


            auto ik =
                std::make_unique<
                    mtc::stages::ComputeIK>(
                        "compute place IK",
                        std::move(
                            place_generator));


            ik->setMaxIKSolutions(
                8);


            ik->setMinSolutionDistance(
                0.5);


            // 这里使用已附着物体自身作为IK frame。
            // 目标是把 OBJECT_ID 放到 /perception/place_pose，
            // MTC会利用attachment relation得到gripper_tcp的对应姿态。
            ik->setIKFrame(
                OBJECT_ID);


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


            place->insert(
                std::move(ik));
        }


        // ====================================================
        // 到达place pose后打开夹爪
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::MoveTo>(
                        "open gripper at place",
                        interpolation_planner);


            stage->setGroup(
                HAND_GROUP);


            stage->setGoal(
                "open");


            place->insert(
                std::move(stage));
        }


        // ====================================================
        // 恢复夹爪与目标的正常碰撞关系
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        ModifyPlanningScene>(
                            "forbid gripper-object collision");


            auto gripper_collision_links =
                hand_group->
                    getLinkModelNamesWithCollisionGeometry();


            gripper_collision_links.emplace_back(
                "gripper_left_link");

            gripper_collision_links.emplace_back(
                "gripper_left_finger");

            gripper_collision_links.emplace_back(
                "gripper_right_link");

            gripper_collision_links.emplace_back(
                "gripper_right_finger");


            stage->allowCollisions(
                OBJECT_ID,
                gripper_collision_links,
                false);


            place->insert(
                std::move(stage));
        }


        // ====================================================
        // 从夹爪解除附着
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        ModifyPlanningScene>(
                            "detach object");


            stage->detachObject(
                OBJECT_ID,
                HAND_FRAME);


            place->insert(
                std::move(stage));
        }


        // ====================================================
        // 放开后沿+Z退出筐子
        // ====================================================

        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                        MoveRelative>(
                            "retreat above bin",
                            cartesian_planner);


            stage->properties().
                configureInitFrom(
                    mtc::Stage::PARENT,
                    {"group"});


            stage->properties().
                set(
                    "marker_ns",
                    "retreat_after_place");


            stage->setIKFrame(
                HAND_FRAME);


            stage->setMinMaxDistance(
                0.10,
                0.18);


            geometry_msgs::msg::
                Vector3Stamped direction;


            direction.header.frame_id =
                WORLD_FRAME;

            direction.vector.z =
                1.0;


            stage->setDirection(
                direction);


            place->insert(
                std::move(stage));
        }


        task.add(
            std::move(place));


        return task;
    }


private:

    rclcpp::Node::SharedPtr
        node_;


    rclcpp::Subscription<
        geometry_msgs::msg::
            PoseStamped>::SharedPtr
        place_pose_sub_;


    std::mutex
        place_pose_mutex_;


    geometry_msgs::msg::PoseStamped
        place_pose_;


    bool
        has_place_pose_{false};


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