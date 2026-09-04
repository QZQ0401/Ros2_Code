#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <rclcpp/rclcpp.hpp>


#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

#include <visualization_msgs/msg/marker.hpp>


#include <tf2/exceptions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>


#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>


class TargetEstimator : public rclcpp::Node
{
public:

  using PointT = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = Cloud::Ptr;


  struct Candidate
  {
    // 当前cluster点云
    CloudPtr cloud;

    // output_frame_/world坐标系中的物体中心
    Eigen::Vector3f center;

    // 物体坐标系到output_frame_/world的旋转矩阵
    //
    // column 0 -> object X
    // column 1 -> object Y
    // column 2 -> object Z / 桌面法向
    Eigen::Matrix3f rotation;

    // 对应物体XYZ方向上的尺寸
    Eigen::Vector3f size;

    // 用于选择目标的代价
    double score;
  };


  struct WorldCloudFrame
  {
    rclcpp::Time stamp;
    CloudPtr cloud;
  };


  TargetEstimator()
  : Node("target_estimator")
  {
    // ========================================================
    // 参数声明
    // ========================================================

    declare_parameter<std::string>(
      "input_cloud_topic",
      "/workspace/merged_points");

    declare_parameter<std::string>(
      "output_frame",
      "world");

    // 用于“从哪个方向挑选目标”的参考 optical frame。
    // 几何估计始终在 output_frame/world 中进行；
    // 这里只保留原来的“更靠近相机光轴的 cluster 优先”策略。
    declare_parameter<std::string>(
      "selection_frame",
      "workspace_camera_optical_frame");

    // true 时输入已经是多相机融合点云。
    // 这时不再按“相机 z 深度”裁剪，而是在 world/output_frame 中
    // 使用下面的 workspace XYZ 范围裁剪。
    declare_parameter<bool>(
      "input_is_fused_cloud",
      true);

    declare_parameter<double>("workspace_min_x", -1.05);
    declare_parameter<double>("workspace_max_x",  1.05);
    declare_parameter<double>("workspace_min_y",  0.30);
    declare_parameter<double>("workspace_max_y",  1.10);
    declare_parameter<double>("workspace_min_z",  0.79);
    declare_parameter<double>("workspace_max_z",  1.30);

    declare_parameter<std::string>(
      "target_id",
      "vision_target");


    declare_parameter<double>(
      "processing_rate",
      10.0);

    declare_parameter<double>(
      "min_depth",
      0.12);

    declare_parameter<double>(
      "max_depth",
      2.0);

    declare_parameter<double>(
      "voxel_leaf_size",
      0.002);


    declare_parameter<double>(
      "plane_distance_threshold",
      0.003);

    declare_parameter<int>(
      "plane_max_iterations",
      150);

    declare_parameter<int>(
      "plane_min_inliers",
      300);

    declare_parameter<double>(
      "plane_max_tilt_deg",
      10.0);


    declare_parameter<double>(
      "cluster_tolerance",
      0.008);

    declare_parameter<int>(
      "min_cluster_size",
      60);

    declare_parameter<int>(
      "max_cluster_size",
      30000);


    declare_parameter<double>(
      "min_planar_dimension",
      0.015);

    declare_parameter<double>(
      "max_planar_dimension",
      0.25);

    declare_parameter<double>(
      "min_object_height",
      0.015);

    declare_parameter<double>(
      "max_object_height",
      0.25);


    declare_parameter<double>(
      "max_axis_ratio",
      0.8);


    declare_parameter<double>(
      "pca_anisotropy_threshold",
      0.15);

    declare_parameter<double>(
      "planar_percentile_low",
      0.02);

    declare_parameter<double>(
      "planar_percentile_high",
      0.98);


    // 先融合world点云，再进行几何估计。
    // 固定工作区相机建议10~20帧。
    declare_parameter<int>(
      "fusion_frames",
      10);

    declare_parameter<double>(
      "fusion_max_age",
      2.0);


    declare_parameter<double>(
      "smoothing_alpha",
      1.0);

    declare_parameter<double>(
      "stable_position_threshold",
      0.005);

    declare_parameter<double>(
      "stable_size_threshold",
      0.008);

    declare_parameter<int>(
      "stable_frames",
      5);

    declare_parameter<double>(
      "target_lost_timeout",
      0.5);


    declare_parameter<bool>(
      "publish_to_planning_scene",
      true);

    declare_parameter<double>(
      "scene_update_rate",
      2.0);

    declare_parameter<double>(
      "collision_padding",
      0.0);


    declare_parameter<double>(
      "tf_timeout",
      0.1);


    // ========================================================
    // 读取参数
    // ========================================================

    input_cloud_topic_ =
      get_parameter(
        "input_cloud_topic").as_string();

    output_frame_ =
      get_parameter(
        "output_frame").as_string();

    selection_frame_ =
      get_parameter(
        "selection_frame").as_string();

    input_is_fused_cloud_ =
      get_parameter(
        "input_is_fused_cloud").as_bool();

    workspace_min_x_ = get_parameter("workspace_min_x").as_double();
    workspace_max_x_ = get_parameter("workspace_max_x").as_double();
    workspace_min_y_ = get_parameter("workspace_min_y").as_double();
    workspace_max_y_ = get_parameter("workspace_max_y").as_double();
    workspace_min_z_ = get_parameter("workspace_min_z").as_double();
    workspace_max_z_ = get_parameter("workspace_max_z").as_double();

    if (workspace_min_x_ >= workspace_max_x_ ||
        workspace_min_y_ >= workspace_max_y_ ||
        workspace_min_z_ >= workspace_max_z_)
    {
      throw std::runtime_error(
        "workspace min bounds must be smaller than max bounds");
    }

    target_id_ =
      get_parameter(
        "target_id").as_string();


    processing_rate_ =
      get_parameter(
        "processing_rate").as_double();

    min_depth_ =
      get_parameter(
        "min_depth").as_double();

    max_depth_ =
      get_parameter(
        "max_depth").as_double();

    voxel_leaf_size_ =
      get_parameter(
        "voxel_leaf_size").as_double();


    plane_distance_threshold_ =
      get_parameter(
        "plane_distance_threshold").as_double();

    plane_max_iterations_ =
      get_parameter(
        "plane_max_iterations").as_int();

    plane_min_inliers_ =
      get_parameter(
        "plane_min_inliers").as_int();

    plane_max_tilt_deg_ =
      get_parameter(
        "plane_max_tilt_deg").as_double();


    cluster_tolerance_ =
      get_parameter(
        "cluster_tolerance").as_double();

    min_cluster_size_ =
      get_parameter(
        "min_cluster_size").as_int();

    max_cluster_size_ =
      get_parameter(
        "max_cluster_size").as_int();


    min_planar_dimension_ =
      get_parameter(
        "min_planar_dimension").as_double();

    max_planar_dimension_ =
      get_parameter(
        "max_planar_dimension").as_double();

    min_object_height_ =
      get_parameter(
        "min_object_height").as_double();

    max_object_height_ =
      get_parameter(
        "max_object_height").as_double();


    max_axis_ratio_ =
      get_parameter(
        "max_axis_ratio").as_double();


    pca_anisotropy_threshold_ =
      get_parameter(
        "pca_anisotropy_threshold").as_double();

    planar_percentile_low_ =
      get_parameter(
        "planar_percentile_low").as_double();

    planar_percentile_high_ =
      get_parameter(
        "planar_percentile_high").as_double();


    fusion_frames_ =
      std::max(
        1,
        static_cast<int>(
          get_parameter(
            "fusion_frames").as_int()));

    fusion_max_age_ =
      std::max(
        0.0,
        get_parameter(
          "fusion_max_age").as_double());


    smoothing_alpha_ =
      get_parameter(
        "smoothing_alpha").as_double();

    stable_position_threshold_ =
      get_parameter(
        "stable_position_threshold").as_double();

    stable_size_threshold_ =
      get_parameter(
        "stable_size_threshold").as_double();

    stable_frames_required_ =
      get_parameter(
        "stable_frames").as_int();

    target_lost_timeout_ =
      get_parameter(
        "target_lost_timeout").as_double();


    publish_to_planning_scene_ =
      get_parameter(
        "publish_to_planning_scene").as_bool();

    scene_update_rate_ =
      get_parameter(
        "scene_update_rate").as_double();

    collision_padding_ =
      get_parameter(
        "collision_padding").as_double();


    tf_timeout_ =
      get_parameter(
        "tf_timeout").as_double();


    // ========================================================
    // TF
    // ========================================================

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(
        this->get_clock());

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer_);


    // ========================================================
    // 发布器
    // ========================================================

    auto latched_qos =
      rclcpp::QoS(1)
        .reliable()
        .transient_local();


    target_pose_pub_ =
      create_publisher<
        geometry_msgs::msg::PoseStamped>(
          "/perception/target_pose",
          latched_qos);


    target_size_pub_ =
      create_publisher<
        geometry_msgs::msg::Vector3Stamped>(
          "/perception/target_size",
          latched_qos);


    target_stable_pub_ =
      create_publisher<
        std_msgs::msg::Bool>(
          "/perception/target_stable",
          latched_qos);


    marker_pub_ =
      create_publisher<
        visualization_msgs::msg::Marker>(
          "/perception/target_marker",
          10);


    // Debug点云使用SensorDataQoS
    filtered_cloud_pub_ =
      create_publisher<
        sensor_msgs::msg::PointCloud2>(
          "/perception/filtered_cloud",
          rclcpp::SensorDataQoS());


    plane_cloud_pub_ =
      create_publisher<
        sensor_msgs::msg::PointCloud2>(
          "/perception/table_plane",
          rclcpp::SensorDataQoS());


    no_plane_cloud_pub_ =
      create_publisher<
        sensor_msgs::msg::PointCloud2>(
          "/perception/no_plane_cloud",
          rclcpp::SensorDataQoS());


    target_cloud_pub_ =
      create_publisher<
        sensor_msgs::msg::PointCloud2>(
          "/perception/target_cloud",
          rclcpp::SensorDataQoS());


    // ========================================================
    // 点云订阅
    // ========================================================

    cloud_sub_ =
      create_subscription<
        sensor_msgs::msg::PointCloud2>(
          input_cloud_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(
            &TargetEstimator::cloudCallback,
            this,
            std::placeholders::_1));


    // ========================================================
    // MoveIt PlanningScene
    // ========================================================

    if (publish_to_planning_scene_)
    {
      try
      {
        planning_scene_interface_ =
          std::make_unique<
            moveit::planning_interface::
              PlanningSceneInterface>(
                "",
                true);
      }
      catch (
        const std::exception & exception)
      {
        RCLCPP_ERROR(
          get_logger(),
          "无法连接MoveIt PlanningScene: %s",
          exception.what());

        throw;
      }


      // 监听目标是否已经被MTC attach。
      //
      // 一旦vision_target附着到机器人，
      // 感知节点不能再把它重新加入World。
      planning_scene_sub_ =
        create_subscription<
          moveit_msgs::msg::PlanningScene>(
            "/monitored_planning_scene",
            10,
            std::bind(
              &TargetEstimator::
                planningSceneCallback,
              this,
              std::placeholders::_1));
    }


    last_process_time_ =
      rclcpp::Time(
        0,
        0,
        get_clock()->get_clock_type());

    last_scene_update_time_ =
      last_process_time_;

    last_valid_detection_time_ =
      get_clock()->now();


    RCLCPP_INFO(
      get_logger(),
      "target_estimator已启动");

    RCLCPP_INFO(
      get_logger(),
      "输入点云: %s",
      input_cloud_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "输出坐标系: %s",
      output_frame_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "融合点云模式: %s, 目标选择参考坐标系: %s",
      input_is_fused_cloud_ ? "true" : "false",
      selection_frame_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "PlanningScene目标ID: %s",
      target_id_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "World点云融合: %d帧, 最大历史%.2fs",
      fusion_frames_,
      fusion_max_age_);
  }


private:

  // ==========================================================
  // 2D minimum-area rectangle
  //
  // projected cluster -> convex hull -> enumerate hull-edge
  // orientations -> minimum-area rectangle.
  //
  // Compared with PCA:
  // - the orientation is determined by the object boundary;
  // - incomplete / asymmetric sampling does not rotate the box merely
  //   because the point covariance changes;
  // - the resulting XY box is the tightest oriented rectangle of the
  //   projected cluster (subject to the actual point-cloud boundary).
  // ==========================================================

  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };


  struct MinAreaRect2D
  {
    bool valid{false};

    // Rectangle center in the chosen 2D table basis.
    double center_x{0.0};
    double center_y{0.0};

    // Unit rectangle axes in the same 2D table basis.
    double ux{1.0};
    double uy{0.0};
    double vx{0.0};
    double vy{1.0};

    double size_u{0.0};
    double size_v{0.0};
    double area{
      std::numeric_limits<double>::infinity()};
  };


  static double cross2D(
    const Point2D & origin,
    const Point2D & a,
    const Point2D & b)
  {
    return
      (a.x - origin.x) *
        (b.y - origin.y) -
      (a.y - origin.y) *
        (b.x - origin.x);
  }


  static std::vector<Point2D>
  convexHull2D(
    std::vector<Point2D> points)
  {
    constexpr double epsilon =
      1e-9;


    if (points.size() <= 2)
    {
      return points;
    }


    std::sort(
      points.begin(),
      points.end(),
      [](
        const Point2D & a,
        const Point2D & b)
      {
        if (a.x < b.x)
        {
          return true;
        }

        if (a.x > b.x)
        {
          return false;
        }

        return a.y < b.y;
      });


    std::vector<Point2D>
      unique_points;

    unique_points.reserve(
      points.size());


    for (
      const auto & point :
      points)
    {
      if (
        unique_points.empty() ||
        std::abs(
          point.x -
          unique_points.back().x) >
          epsilon ||
        std::abs(
          point.y -
          unique_points.back().y) >
          epsilon)
      {
        unique_points.push_back(
          point);
      }
    }


    if (unique_points.size() <= 2)
    {
      return unique_points;
    }


    std::vector<Point2D>
      hull(
        2 *
        unique_points.size());


    std::size_t k =
      0;


    // Lower hull.
    for (
      const auto & point :
      unique_points)
    {
      while (
        k >= 2 &&
        cross2D(
          hull[k - 2],
          hull[k - 1],
          point) <= 0.0)
      {
        --k;
      }

      hull[k++] =
        point;
    }


    // Upper hull.
    const std::size_t lower_size =
      k;


    for (
      std::size_t i =
        unique_points.size() - 1;
      i > 0;
      --i)
    {
      const auto & point =
        unique_points[i - 1];

      while (
        k > lower_size &&
        cross2D(
          hull[k - 2],
          hull[k - 1],
          point) <= 0.0)
      {
        --k;
      }

      hull[k++] =
        point;
    }


    if (k > 1)
    {
      --k;
    }


    hull.resize(
      k);


    return hull;
  }


  static MinAreaRect2D
  minimumAreaRectangle2D(
    const std::vector<Point2D> & points)
  {
    MinAreaRect2D
      best;


    const auto hull =
      convexHull2D(
        points);


    if (hull.size() < 2)
    {
      return best;
    }


    for (
      std::size_t i = 0;
      i < hull.size();
      ++i)
    {
      const auto & a =
        hull[i];

      const auto & b =
        hull[
          (i + 1) %
          hull.size()];


      const double edge_x =
        b.x - a.x;

      const double edge_y =
        b.y - a.y;


      const double edge_length =
        std::hypot(
          edge_x,
          edge_y);


      if (edge_length < 1e-9)
      {
        continue;
      }


      // u follows a hull edge; v is its +90 deg perpendicular.
      const double ux =
        edge_x /
        edge_length;

      const double uy =
        edge_y /
        edge_length;

      const double vx =
        -uy;

      const double vy =
        ux;


      double min_u =
        std::numeric_limits<double>::infinity();

      double max_u =
        -std::numeric_limits<double>::infinity();

      double min_v =
        std::numeric_limits<double>::infinity();

      double max_v =
        -std::numeric_limits<double>::infinity();


      for (
        const auto & point :
        hull)
      {
        const double u =
          point.x * ux +
          point.y * uy;

        const double v =
          point.x * vx +
          point.y * vy;


        min_u =
          std::min(
            min_u,
            u);

        max_u =
          std::max(
            max_u,
            u);

        min_v =
          std::min(
            min_v,
            v);

        max_v =
          std::max(
            max_v,
            v);
      }


      const double size_u =
        max_u -
        min_u;

      const double size_v =
        max_v -
        min_v;

      const double area =
        size_u *
        size_v;


      if (
        !std::isfinite(area) ||
        area >=
          best.area)
      {
        continue;
      }


      const double center_u =
        0.5 *
        (min_u + max_u);

      const double center_v =
        0.5 *
        (min_v + max_v);


      best.valid =
        true;

      best.ux =
        ux;

      best.uy =
        uy;

      best.vx =
        vx;

      best.vy =
        vy;

      best.size_u =
        size_u;

      best.size_v =
        size_v;

      best.center_x =
        center_u * ux +
        center_v * vx;

      best.center_y =
        center_u * uy +
        center_v * vy;

      best.area =
        area;
    }


    return best;
  }


  // ==========================================================
  // 主点云回调
  // ==========================================================

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const rclcpp::Time message_time(
      message->header.stamp,
      get_clock()->get_clock_type());


    // ========================================================
    // 限制处理频率
    // ========================================================

    if (
      processing_rate_ > 0.0 &&
      last_process_time_.nanoseconds() != 0)
    {
      const double dt =
        (message_time -
         last_process_time_).seconds();

      if (
        dt >= 0.0 &&
        dt < 1.0 / processing_rate_)
      {
        return;
      }
    }

    last_process_time_ =
      message_time;


    // ========================================================
    // ROS PointCloud2 -> PCL
    // ========================================================

    CloudPtr raw_cloud(
      new Cloud);

    pcl::fromROSMsg(
      *message,
      *raw_cloud);


    if (raw_cloud->empty())
    {
      handleDetectionMiss();

      return;
    }


    // ========================================================
    // 删除NaN
    // ========================================================

    CloudPtr clean_cloud(
      new Cloud);

    std::vector<int>
      valid_indices;


    pcl::removeNaNFromPointCloud(
      *raw_cloud,
      *clean_cloud,
      valid_indices);


    // ========================================================
    // 输入预处理
    //
    // 单相机模式：
    //   min_depth/max_depth 仍然表示 optical frame 的 z 深度。
    //
    // 多相机融合模式：
    //   /workspace/merged_points 已经位于 world/output_frame，
    //   不能再把 world Z 当作相机深度；因此这里先跳过深度裁剪，
    //   在转换到 output_frame 后使用 workspace XYZ 范围裁剪。
    // ========================================================

    CloudPtr pre_transform_cloud(
      new Cloud);


    if (input_is_fused_cloud_)
    {
      *pre_transform_cloud = *clean_cloud;
    }
    else
    {
      pcl::PassThrough<PointT>
        depth_pass;

      depth_pass.setInputCloud(
        clean_cloud);

      depth_pass.setFilterFieldName(
        "z");

      depth_pass.setFilterLimits(
        static_cast<float>(min_depth_),
        static_cast<float>(max_depth_));

      depth_pass.filter(
        *pre_transform_cloud);
    }


    if (pre_transform_cloud->empty())
    {
      handleDetectionMiss();

      return;
    }


    // ========================================================
    // 使用点云自身timestamp查询 Camera -> World TF
    //
    // 从这里开始：
    //   桌面分割 / 聚类 / minimum-area rectangle / Pose / Size
    // 全部在output_frame_中进行。
    // ========================================================

    geometry_msgs::msg::TransformStamped
      camera_to_world_message;


    try
    {
      camera_to_world_message =
        tf_buffer_->lookupTransform(
          output_frame_,
          message->header.frame_id,
          message_time,
          rclcpp::Duration::from_seconds(
            tf_timeout_));
    }
    catch (
      const tf2::TransformException &
        exception)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "无法将点云转换到%s: %s",
        output_frame_.c_str(),
        exception.what());

      handleDetectionMiss();

      return;
    }


    Eigen::Quaternionf
      camera_to_world_q(
        static_cast<float>(
          camera_to_world_message.
            transform.rotation.w),
        static_cast<float>(
          camera_to_world_message.
            transform.rotation.x),
        static_cast<float>(
          camera_to_world_message.
            transform.rotation.y),
        static_cast<float>(
          camera_to_world_message.
            transform.rotation.z));


    if (
      camera_to_world_q.norm() <
      1e-6f)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Camera -> %s TF四元数无效",
        output_frame_.c_str());

      handleDetectionMiss();

      return;
    }


    camera_to_world_q.normalize();


    Eigen::Affine3f
      camera_to_world =
        Eigen::Affine3f::Identity();


    camera_to_world.linear() =
      camera_to_world_q.
        toRotationMatrix();

    camera_to_world.translation() <<
      static_cast<float>(
        camera_to_world_message.
          transform.translation.x),
      static_cast<float>(
        camera_to_world_message.
          transform.translation.y),
      static_cast<float>(
        camera_to_world_message.
          transform.translation.z);


    // ========================================================
    // 整帧点云先转换到World
    // ========================================================

    CloudPtr current_world_cloud(
      new Cloud);


    pcl::transformPointCloud(
      *pre_transform_cloud,
      *current_world_cloud,
      camera_to_world);


    // ========================================================
    // 融合点云模式下，在 output_frame/world 中裁剪工作区。
    //
    // 这一步非常重要：
    //   1. 两个相机已经从不同方向观察，不能再使用某一个相机的深度轴；
    //   2. 直接裁出桌面工作区可显著减少机器人、地面和远处模型干扰；
    //   3. 后面的 RANSAC / clustering / OBB 都在同一个 world frame 中。
    // ========================================================

    if (input_is_fused_cloud_)
    {
      auto crop_axis =
        [](const CloudPtr & input,
           const std::string & field,
           double min_value,
           double max_value) -> CloudPtr
        {
          CloudPtr output(new Cloud);

          pcl::PassThrough<PointT> crop;
          crop.setInputCloud(input);
          crop.setFilterFieldName(field);
          crop.setFilterLimits(
            static_cast<float>(min_value),
            static_cast<float>(max_value));
          crop.filter(*output);

          return output;
        };


      current_world_cloud =
        crop_axis(
          current_world_cloud,
          "x",
          workspace_min_x_,
          workspace_max_x_);

      current_world_cloud =
        crop_axis(
          current_world_cloud,
          "y",
          workspace_min_y_,
          workspace_max_y_);

      current_world_cloud =
        crop_axis(
          current_world_cloud,
          "z",
          workspace_min_z_,
          workspace_max_z_);


      if (current_world_cloud->empty())
      {
        handleDetectionMiss();
        return;
      }
    }


    // ========================================================
    // 为目标选择准备一个独立的 optical reference frame。
    //
    // 输入 fused cloud 的 frame_id 是 world，因此不能再用
    // input frame 来代表“相机光轴”。几何估计仍在 world 中，
    // 这里只把候选中心临时变换到 selection_frame_ 来计算 score。
    // ========================================================

    Eigen::Affine3f
      world_to_selection =
        Eigen::Affine3f::Identity();


    if (!selection_frame_.empty())
    {
      try
      {
        const auto selection_from_world_message =
          tf_buffer_->lookupTransform(
            selection_frame_,
            output_frame_,
            message_time,
            rclcpp::Duration::from_seconds(
              tf_timeout_));


        Eigen::Quaternionf selection_q(
          static_cast<float>(
            selection_from_world_message.transform.rotation.w),
          static_cast<float>(
            selection_from_world_message.transform.rotation.x),
          static_cast<float>(
            selection_from_world_message.transform.rotation.y),
          static_cast<float>(
            selection_from_world_message.transform.rotation.z));


        if (selection_q.norm() < 1e-6f)
        {
          throw std::runtime_error(
            "selection_frame TF quaternion is invalid");
        }


        selection_q.normalize();

        world_to_selection.linear() =
          selection_q.toRotationMatrix();

        world_to_selection.translation() <<
          static_cast<float>(
            selection_from_world_message.transform.translation.x),
          static_cast<float>(
            selection_from_world_message.transform.translation.y),
          static_cast<float>(
            selection_from_world_message.transform.translation.z);
      }
      catch (const std::exception & exception)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "无法获取目标选择参考坐标系 %s <- %s: %s",
          selection_frame_.c_str(),
          output_frame_.c_str(),
          exception.what());

        handleDetectionMiss();
        return;
      }
    }


    // ========================================================
    // 10~20帧World点云融合
    //
    // 先融合，再VoxelGrid，再做RANSAC/Cluster/minimum-area rectangle。
    // 这样多帧能补足单帧缺失表面，而不是只对最终Pose/Size做EMA。
    // ========================================================

    if (
      !world_cloud_history_.empty() &&
      message_time <
        world_cloud_history_.back().stamp)
    {
      // Gazebo /clock复位或时间跳变时，旧点云不能继续融合。
      world_cloud_history_.clear();
    }


    world_cloud_history_.push_back(
      WorldCloudFrame{
        message_time,
        current_world_cloud});


    while (
      world_cloud_history_.size() >
      static_cast<std::size_t>(
        fusion_frames_))
    {
      world_cloud_history_.pop_front();
    }


    if (fusion_max_age_ > 0.0)
    {
      while (
        !world_cloud_history_.empty())
      {
        const double age =
          (
            message_time -
            world_cloud_history_.front().stamp
          ).seconds();

        if (
          age >= 0.0 &&
          age <= fusion_max_age_)
        {
          break;
        }

        world_cloud_history_.pop_front();
      }
    }


    CloudPtr fused_world_cloud(
      new Cloud);


    std::size_t total_points =
      0;

    for (
      const auto & frame :
      world_cloud_history_)
    {
      if (frame.cloud)
      {
        total_points +=
          frame.cloud->size();
      }
    }


    fused_world_cloud->reserve(
      total_points);


    for (
      const auto & frame :
      world_cloud_history_)
    {
      if (!frame.cloud)
      {
        continue;
      }

      *fused_world_cloud +=
        *frame.cloud;
    }


    if (fused_world_cloud->empty())
    {
      handleDetectionMiss();

      return;
    }


    // ========================================================
    // 融合后的World点云再做VoxelGrid
    // ========================================================

    CloudPtr filtered_cloud(
      new Cloud);


    pcl::VoxelGrid<PointT>
      voxel;


    voxel.setInputCloud(
      fused_world_cloud);

    voxel.setLeafSize(
      static_cast<float>(
        voxel_leaf_size_),

      static_cast<float>(
        voxel_leaf_size_),

      static_cast<float>(
        voxel_leaf_size_));


    voxel.filter(
      *filtered_cloud);


    std_msgs::msg::Header
      world_header;

    world_header.stamp =
      message->header.stamp;

    world_header.frame_id =
      output_frame_;


    publishCloud(
      filtered_cloud_pub_,
      filtered_cloud,
      world_header);


    if (
      filtered_cloud->size() <
      static_cast<std::size_t>(
        plane_min_inliers_))
    {
      handleDetectionMiss();

      return;
    }


    // ========================================================
    // RANSAC检测桌面
    //
    // 只允许平面法向接近World +Z，避免把墙、箱体侧面、机器人结构
    // 误识别为桌面。
    // ========================================================

    pcl::ModelCoefficients::Ptr
      plane_coefficients(
        new pcl::ModelCoefficients);


    pcl::PointIndices::Ptr
      plane_inliers(
        new pcl::PointIndices);


    pcl::SACSegmentation<PointT>
      segmentation;


    segmentation.setOptimizeCoefficients(
      true);

    segmentation.setModelType(
      pcl::SACMODEL_PERPENDICULAR_PLANE);

    segmentation.setMethodType(
      pcl::SAC_RANSAC);

    segmentation.setAxis(
      Eigen::Vector3f(
        0.0f,
        0.0f,
        1.0f));

    const double plane_max_tilt_rad =
      std::clamp(
        plane_max_tilt_deg_,
        0.1,
        89.0) *
      std::acos(-1.0) /
      180.0;


    segmentation.setEpsAngle(
      plane_max_tilt_rad);

    segmentation.setMaxIterations(
      plane_max_iterations_);

    segmentation.setDistanceThreshold(
      plane_distance_threshold_);

    segmentation.setInputCloud(
      filtered_cloud);


    segmentation.segment(
      *plane_inliers,
      *plane_coefficients);


    if (
      plane_inliers->indices.size() <
      static_cast<std::size_t>(
        plane_min_inliers_) ||
      plane_coefficients->values.size() <
      4)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "没有找到足够稳定且接近水平的桌面平面");

      handleDetectionMiss();

      return;
    }


    // ========================================================
    // 提取桌面点云
    // ========================================================

    pcl::ExtractIndices<PointT>
      extractor;


    CloudPtr plane_cloud(
      new Cloud);


    extractor.setInputCloud(
      filtered_cloud);

    extractor.setIndices(
      plane_inliers);

    extractor.setNegative(
      false);

    extractor.filter(
      *plane_cloud);


    publishCloud(
      plane_cloud_pub_,
      plane_cloud,
      world_header);


    // ========================================================
    // 去除桌面
    // ========================================================

    CloudPtr object_cloud(
      new Cloud);


    extractor.setNegative(
      true);

    extractor.filter(
      *object_cloud);


    publishCloud(
      no_plane_cloud_pub_,
      object_cloud,
      world_header);


    if (
      object_cloud->size() <
      static_cast<std::size_t>(
        min_cluster_size_))
    {
      handleDetectionMiss();

      return;
    }


    // ========================================================
    // 归一化桌面平面
    //
    // ax + by + cz + d = 0
    // ========================================================

    Eigen::Vector3f plane_normal(
      plane_coefficients->values[0],
      plane_coefficients->values[1],
      plane_coefficients->values[2]);


    float plane_d =
      plane_coefficients->values[3];


    const float normal_norm =
      plane_normal.norm();


    if (normal_norm < 1e-6f)
    {
      handleDetectionMiss();

      return;
    }


    plane_normal /=
      normal_norm;

    plane_d /=
      normal_norm;


    // ========================================================
    // 强制桌面法向始终朝World +Z
    // ========================================================

    if (plane_normal.z() < 0.0f)
    {
      plane_normal =
        -plane_normal;

      plane_d =
        -plane_d;
    }


    // ========================================================
    // Euclidean Cluster
    // ========================================================

    pcl::search::KdTree<PointT>::Ptr
      tree(
        new pcl::search::KdTree<PointT>);


    tree->setInputCloud(
      object_cloud);


    std::vector<
      pcl::PointIndices>
      cluster_indices;


    pcl::EuclideanClusterExtraction<PointT>
      clustering;


    clustering.setClusterTolerance(
      cluster_tolerance_);

    clustering.setMinClusterSize(
      min_cluster_size_);

    clustering.setMaxClusterSize(
      max_cluster_size_);

    clustering.setSearchMethod(
      tree);

    clustering.setInputCloud(
      object_cloud);


    clustering.extract(
      cluster_indices);


    if (cluster_indices.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "去除桌面后没有找到合法cluster");

      handleDetectionMiss();

      return;
    }


    // ========================================================
    // 从所有cluster中选择目标
    //
    // 几何估计全部在World中完成。
    // 多相机输入时使用 selection_frame_（默认第一台相机 optical frame）
    // 作为独立的“目标选择观察方向”，而不是错误地把 fused cloud 的
    // world frame 当作相机 frame。
    // ========================================================

    Candidate best_candidate;

    bool found_candidate =
      false;

    best_candidate.score =
      std::numeric_limits<
        double>::infinity();


    for (
      const auto & indices :
      cluster_indices)
    {
      CloudPtr cluster(
        new Cloud);


      cluster->reserve(
        indices.indices.size());


      for (
        const int index :
        indices.indices)
      {
        cluster->push_back(
          (*object_cloud)[index]);
      }


      Candidate candidate;


      if (
        !estimateCandidate(
          cluster,
          plane_normal,
          plane_d,
          candidate))
      {
        continue;
      }


      const Eigen::Vector3f
        center_selection =
          world_to_selection *
          candidate.center;


      if (center_selection.z() <= 1e-4f)
      {
        continue;
      }


      const double axis_ratio =
        std::hypot(
          static_cast<double>(
            center_selection.x()),
          static_cast<double>(
            center_selection.y())) /
        static_cast<double>(
          center_selection.z());


      if (
        axis_ratio >
        max_axis_ratio_)
      {
        continue;
      }


      candidate.score =
        axis_ratio +
        0.02 *
        static_cast<double>(
          center_selection.z());


      if (
        candidate.score <
        best_candidate.score)
      {
        best_candidate =
          candidate;

        found_candidate =
          true;
      }
    }


    if (!found_candidate)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cluster存在，但没有一个满足目标几何约束");

      handleDetectionMiss();

      return;
    }


    // ========================================================
    // 发布最终目标cluster；其frame_id已经是World
    // ========================================================

    publishCloud(
      target_cloud_pub_,
      best_candidate.cloud,
      world_header);


    // ========================================================
    // World坐标系中的目标Pose
    // ========================================================

    geometry_msgs::msg::PoseStamped
      pose_output;


    pose_output.header =
      world_header;


    pose_output.pose.position.x =
      best_candidate.center.x();

    pose_output.pose.position.y =
      best_candidate.center.y();

    pose_output.pose.position.z =
      best_candidate.center.z();


    Eigen::Quaternionf
      quaternion_world(
        best_candidate.rotation);


    quaternion_world.normalize();


    pose_output.pose.orientation.x =
      quaternion_world.x();

    pose_output.pose.orientation.y =
      quaternion_world.y();

    pose_output.pose.orientation.z =
      quaternion_world.z();

    pose_output.pose.orientation.w =
      quaternion_world.w();


    // ========================================================
    // 稳定性判断
    // ========================================================

    Eigen::Vector3d raw_size(
      best_candidate.size.x(),
      best_candidate.size.y(),
      best_candidate.size.z());


    updateStability(
      pose_output,
      raw_size);


    // ========================================================
    // 可选EMA
    //
    // 默认smoothing_alpha=1.0，因此几何精度主要来自多帧点云融合，
    // 不再依靠“先估计Pose/Size再平均”来掩盖单帧误差。
    // ========================================================

    updateFilteredEstimate(
      pose_output,
      raw_size);


    last_valid_detection_time_ =
      get_clock()->now();


    // ========================================================
    // 发布结果
    // ========================================================

    publishEstimate();


    // ========================================================
    // 只有目标连续稳定后才写入PlanningScene。
    // ========================================================

    if (
      target_stable_ &&
      publish_to_planning_scene_ &&
      !target_attached_)
    {
      updatePlanningScene();
    }
  }


  // ==========================================================
  // 利用“桌面平面 + cluster”估计物体
  // ==========================================================

  bool estimateCandidate(
    const CloudPtr & cluster,
    const Eigen::Vector3f & plane_normal,
    const float plane_d,
    Candidate & candidate)
  {
    if (
      cluster->size() <
      static_cast<std::size_t>(
        min_cluster_size_))
    {
      return false;
    }


    // ========================================================
    // Build a deterministic 2D basis on the table plane.
    //
    // This basis is only used to express projected points in 2D.
    // The final object axes come from the minimum-area rectangle.
    // ========================================================

    Eigen::Vector3f basis_x(
      1.0f,
      0.0f,
      0.0f);


    basis_x -=
      plane_normal *
      basis_x.dot(
        plane_normal);


    if (basis_x.norm() < 1e-5f)
    {
      basis_x =
        Eigen::Vector3f(
          0.0f,
          1.0f,
          0.0f);

      basis_x -=
        plane_normal *
        basis_x.dot(
          plane_normal);
    }


    if (basis_x.norm() < 1e-5f)
    {
      return false;
    }


    basis_x.normalize();


    Eigen::Vector3f basis_y =
      plane_normal.cross(
        basis_x);


    if (basis_y.norm() < 1e-5f)
    {
      return false;
    }


    basis_y.normalize();


    const Eigen::Vector3f
      plane_origin =
        -plane_d *
        plane_normal;


    // ========================================================
    // Project cluster to the table and collect heights.
    // ========================================================

    std::vector<Point2D>
      projected_2d;

    std::vector<float>
      heights;


    projected_2d.reserve(
      cluster->size());

    heights.reserve(
      cluster->size());


    for (
      const auto & point :
      cluster->points)
    {
      const Eigen::Vector3f p(
        point.x,
        point.y,
        point.z);


      const float height =
        plane_normal.dot(p) +
        plane_d;


      // Small fitting noise may lie slightly below the table.
      if (height <= 0.0f)
      {
        continue;
      }


      const Eigen::Vector3f projected =
        p -
        height *
        plane_normal;


      const Eigen::Vector3f relative =
        projected -
        plane_origin;


      projected_2d.push_back(
        Point2D{
          static_cast<double>(
            basis_x.dot(relative)),
          static_cast<double>(
            basis_y.dot(relative))});

      heights.push_back(
        height);
    }


    if (
      projected_2d.size() <
      static_cast<std::size_t>(
        min_cluster_size_))
    {
      return false;
    }


    // ========================================================
    // Height remains a robust percentile of distance above table.
    // ========================================================

    const float object_height =
      percentile(
        heights,
        0.95);


    if (
      object_height <
        min_object_height_ ||
      object_height >
        max_object_height_)
    {
      return false;
    }


    // ========================================================
    // Minimum-area rectangle on the projected cluster.
    //
    // No covariance / PCA direction is used here.
    // ========================================================

    const MinAreaRect2D rectangle =
      minimumAreaRectangle2D(
        projected_2d);


    if (!rectangle.valid)
    {
      return false;
    }


    Eigen::Vector3f axis_u =
      static_cast<float>(
        rectangle.ux) *
        basis_x +
      static_cast<float>(
        rectangle.uy) *
        basis_y;


    Eigen::Vector3f axis_v =
      static_cast<float>(
        rectangle.vx) *
        basis_x +
      static_cast<float>(
        rectangle.vy) *
        basis_y;


    if (
      axis_u.norm() < 1e-5f ||
      axis_v.norm() < 1e-5f)
    {
      return false;
    }


    axis_u.normalize();

    axis_v.normalize();


    float size_x =
      static_cast<float>(
        rectangle.size_u);

    float size_y =
      static_cast<float>(
        rectangle.size_v);


    // Keep object X as the longer planar side.
    //
    // This makes size/orientation semantics stable for MTC and also
    // removes the 90 deg alternative representation of the same box.
    Eigen::Vector3f axis_x =
      axis_u;

    Eigen::Vector3f axis_y =
      axis_v;


    if (size_y > size_x)
    {
      std::swap(
        size_x,
        size_y);

      axis_x =
        axis_v;

      // Preserve a right-handed frame:
      // axis_x x axis_y = plane_normal.
      axis_y =
        -axis_u;
    }


    if (
      size_x <
        min_planar_dimension_ ||
      size_x >
        max_planar_dimension_ ||
      size_y <
        min_planar_dimension_ ||
      size_y >
        max_planar_dimension_)
    {
      return false;
    }


    // ========================================================
    // Stabilize the remaining 180 deg sign ambiguity.
    //
    // A rectangle has identical geometry after a 180 deg rotation.
    // Select the sign closest to projected World X / Y.
    // ========================================================

    Eigen::Vector3f world_x(
      1.0f,
      0.0f,
      0.0f);

    Eigen::Vector3f world_y(
      0.0f,
      1.0f,
      0.0f);


    world_x -=
      plane_normal *
      world_x.dot(
        plane_normal);

    world_y -=
      plane_normal *
      world_y.dot(
        plane_normal);


    if (world_x.norm() > 1e-5f)
    {
      world_x.normalize();
    }

    if (world_y.norm() > 1e-5f)
    {
      world_y.normalize();
    }


    const float dot_x =
      axis_x.dot(
        world_x);

    const float dot_y =
      axis_x.dot(
        world_y);


    bool flip_axes =
      false;


    if (
      std::abs(dot_x) >=
      std::abs(dot_y))
    {
      flip_axes =
        dot_x < 0.0f;
    }
    else
    {
      flip_axes =
        dot_y < 0.0f;
    }


    if (flip_axes)
    {
      axis_x =
        -axis_x;

      axis_y =
        -axis_y;
    }


    // Re-orthogonalize using the table normal.
    axis_y =
      plane_normal.cross(
        axis_x);


    if (axis_y.norm() < 1e-5f)
    {
      return false;
    }


    axis_y.normalize();

    axis_x =
      axis_y.cross(
        plane_normal);

    axis_x.normalize();


    // ========================================================
    // Rectangle center -> world.
    // ========================================================

    const Eigen::Vector3f
      bottom_center =
        plane_origin +
        static_cast<float>(
          rectangle.center_x) *
          basis_x +
        static_cast<float>(
          rectangle.center_y) *
          basis_y;


    const Eigen::Vector3f
      object_center =
        bottom_center +
        0.5f *
        object_height *
        plane_normal;


    // ========================================================
    // Output.
    // ========================================================

    candidate.cloud =
      cluster;


    candidate.center =
      object_center;


    candidate.size =
      Eigen::Vector3f(
        size_x,
        size_y,
        object_height);


    candidate.rotation.col(0) =
      axis_x;

    candidate.rotation.col(1) =
      axis_y;

    candidate.rotation.col(2) =
      plane_normal;


    candidate.score =
      std::numeric_limits<
        double>::infinity();


    return true;
  }

  // ==========================================================
  // 百分位
  // ==========================================================

  static float percentile(
    std::vector<float> values,
    const double ratio)
  {
    if (values.empty())
    {
      return 0.0f;
    }


    const double bounded_ratio =
      std::clamp(
        ratio,
        0.0,
        1.0);


    const std::size_t index =
      static_cast<std::size_t>(
        bounded_ratio *
        static_cast<double>(
          values.size() - 1));


    std::nth_element(
      values.begin(),
      values.begin() + index,
      values.end());


    return values[index];
  }


  // ==========================================================
  // 稳定性判断
  // ==========================================================

  void updateStability(
    const geometry_msgs::msg::PoseStamped &
      raw_pose,
    const Eigen::Vector3d &
      raw_size)
  {
    if (!has_raw_estimate_)
    {
      previous_raw_pose_ =
        raw_pose;

      previous_raw_size_ =
        raw_size;

      has_raw_estimate_ =
        true;

      stable_count_ =
        0;

      target_stable_ =
        false;

      return;
    }


    const Eigen::Vector3d current_position(
      raw_pose.pose.position.x,
      raw_pose.pose.position.y,
      raw_pose.pose.position.z);


    const Eigen::Vector3d previous_position(
      previous_raw_pose_.pose.position.x,
      previous_raw_pose_.pose.position.y,
      previous_raw_pose_.pose.position.z);


    const double position_change =
      (
        current_position -
        previous_position
      ).norm();


    const double size_change =
      (
        raw_size -
        previous_raw_size_
      ).norm();


    if (
      position_change <
        stable_position_threshold_ &&
      size_change <
        stable_size_threshold_)
    {
      ++stable_count_;
    }
    else
    {
      stable_count_ =
        0;
    }


    target_stable_ =
      stable_count_ >=
      stable_frames_required_;


    previous_raw_pose_ =
      raw_pose;

    previous_raw_size_ =
      raw_size;
  }


  // ==========================================================
  // EMA平滑
  // ==========================================================

  void updateFilteredEstimate(
    const geometry_msgs::msg::PoseStamped &
      raw_pose,
    const Eigen::Vector3d &
      raw_size)
  {
    if (!has_filtered_estimate_)
    {
      filtered_pose_ =
        raw_pose;

      filtered_size_ =
        raw_size;

      has_filtered_estimate_ =
        true;

      return;
    }


    const double alpha =
      std::clamp(
        smoothing_alpha_,
        0.0,
        1.0);


    // ========================================================
    // Position
    // ========================================================

    filtered_pose_.header =
      raw_pose.header;


    filtered_pose_.pose.position.x =
      (1.0 - alpha) *
        filtered_pose_.
          pose.position.x +
      alpha *
        raw_pose.pose.position.x;


    filtered_pose_.pose.position.y =
      (1.0 - alpha) *
        filtered_pose_.
          pose.position.y +
      alpha *
        raw_pose.pose.position.y;


    filtered_pose_.pose.position.z =
      (1.0 - alpha) *
        filtered_pose_.
          pose.position.z +
      alpha *
        raw_pose.pose.position.z;


    // ========================================================
    // Quaternion
    // ========================================================

    Eigen::Quaterniond old_q(
      filtered_pose_.pose.orientation.w,
      filtered_pose_.pose.orientation.x,
      filtered_pose_.pose.orientation.y,
      filtered_pose_.pose.orientation.z);


    Eigen::Quaterniond new_q(
      raw_pose.pose.orientation.w,
      raw_pose.pose.orientation.x,
      raw_pose.pose.orientation.y,
      raw_pose.pose.orientation.z);


    old_q.normalize();

    new_q.normalize();


    // q与-q表示相同旋转。
    //
    // 统一符号可避免slerp走远路。
    if (
      old_q.dot(new_q) <
      0.0)
    {
      new_q.coeffs() =
        -new_q.coeffs();
    }


    const Eigen::Quaterniond
      filtered_q =
        old_q.slerp(
          alpha,
          new_q)
        .normalized();


    filtered_pose_.pose.orientation.x =
      filtered_q.x();

    filtered_pose_.pose.orientation.y =
      filtered_q.y();

    filtered_pose_.pose.orientation.z =
      filtered_q.z();

    filtered_pose_.pose.orientation.w =
      filtered_q.w();


    // ========================================================
    // Size
    // ========================================================

    filtered_size_ =
      (1.0 - alpha) *
        filtered_size_ +
      alpha *
        raw_size;
  }


  // ==========================================================
  // 发布检测结果
  // ==========================================================

  void publishEstimate()
  {
    if (!has_filtered_estimate_)
    {
      return;
    }


    // ========================================================
    // Pose
    // ========================================================

    target_pose_pub_->
      publish(
        filtered_pose_);


    // ========================================================
    // Size
    // ========================================================

    geometry_msgs::msg::Vector3Stamped
      size_message;


    size_message.header =
      filtered_pose_.header;


    size_message.vector.x =
      filtered_size_.x();

    size_message.vector.y =
      filtered_size_.y();

    size_message.vector.z =
      filtered_size_.z();


    target_size_pub_->
      publish(
        size_message);


    // ========================================================
    // Stable
    // ========================================================

    std_msgs::msg::Bool
      stable_message;


    stable_message.data =
      target_stable_;


    target_stable_pub_->
      publish(
        stable_message);


    // ========================================================
    // RViz Marker
    // ========================================================

    visualization_msgs::msg::Marker
      marker;


    marker.header =
      filtered_pose_.header;

    marker.ns =
      "target_estimator";

    marker.id =
      0;

    marker.type =
      visualization_msgs::msg::
        Marker::CUBE;

    marker.action =
      visualization_msgs::msg::
        Marker::ADD;


    marker.pose =
      filtered_pose_.pose;


    marker.scale.x =
      std::max(
        filtered_size_.x(),
        0.001);

    marker.scale.y =
      std::max(
        filtered_size_.y(),
        0.001);

    marker.scale.z =
      std::max(
        filtered_size_.z(),
        0.001);


    // 稳定后显示绿色，否则黄色
    if (target_stable_)
    {
      marker.color.r =
        0.1f;

      marker.color.g =
        1.0f;

      marker.color.b =
        0.1f;
    }
    else
    {
      marker.color.r =
        1.0f;

      marker.color.g =
        0.8f;

      marker.color.b =
        0.0f;
    }


    marker.color.a =
      0.45f;


    marker.lifetime =
      rclcpp::Duration::from_seconds(
        0.3);


    marker_pub_->
      publish(
        marker);
  }


  // ==========================================================
  // 更新MoveIt PlanningScene
  // ==========================================================

  void updatePlanningScene()
  {
    if (
      !planning_scene_interface_ ||
      !has_filtered_estimate_ ||
      target_attached_)
    {
      return;
    }


    const rclcpp::Time now =
      get_clock()->now();


    if (
      last_scene_update_time_.
        nanoseconds() != 0 &&
      scene_update_rate_ > 0.0)
    {
      const double dt =
        (
          now -
          last_scene_update_time_
        ).seconds();


      if (
        dt <
        1.0 /
        scene_update_rate_)
      {
        return;
      }
    }


    moveit_msgs::msg::CollisionObject
      object;


    object.header.frame_id =
      output_frame_;

    object.header.stamp =
      now;


    object.id =
      target_id_;


    // ========================================================
    // 使用视觉估计出的长方体作为碰撞模型
    // ========================================================

    shape_msgs::msg::SolidPrimitive
      primitive;


    primitive.type =
      shape_msgs::msg::
        SolidPrimitive::BOX;


    primitive.dimensions.resize(
      3);


    primitive.dimensions[
      shape_msgs::msg::
        SolidPrimitive::BOX_X] =
      filtered_size_.x() +
      2.0 *
      collision_padding_;


    primitive.dimensions[
      shape_msgs::msg::
        SolidPrimitive::BOX_Y] =
      filtered_size_.y() +
      2.0 *
      collision_padding_;


    primitive.dimensions[
      shape_msgs::msg::
        SolidPrimitive::BOX_Z] =
      filtered_size_.z() +
      2.0 *
      collision_padding_;


    object.primitives.push_back(
      primitive);


    object.primitive_poses.push_back(
      filtered_pose_.pose);


    object.operation =
      moveit_msgs::msg::
        CollisionObject::ADD;


    const bool success =
      planning_scene_interface_->
        applyCollisionObject(
          object);


    if (success)
    {
      target_in_scene_ =
        true;

      last_scene_update_time_ =
        now;


      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "vision_target: "
        "position=(%.3f %.3f %.3f), "
        "size=(%.3f %.3f %.3f), "
        "stable=%s",
        filtered_pose_.pose.position.x,
        filtered_pose_.pose.position.y,
        filtered_pose_.pose.position.z,
        filtered_size_.x(),
        filtered_size_.y(),
        filtered_size_.z(),
        target_stable_ ?
          "true" :
          "false");
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "MoveIt拒绝vision_target更新");
    }
  }


  // ==========================================================
  // 检测丢失
  // ==========================================================

  void handleDetectionMiss()
  {
    stable_count_ =
      0;

    target_stable_ =
      false;


    std_msgs::msg::Bool
      stable_message;


    stable_message.data =
      false;


    target_stable_pub_->
      publish(
        stable_message);


    const double lost_duration =
      (
        get_clock()->now() -
        last_valid_detection_time_
      ).seconds();


    if (
      lost_duration <
      target_lost_timeout_)
    {
      return;
    }


    has_raw_estimate_ =
      false;

    has_filtered_estimate_ =
      false;


    // ========================================================
    // 删除RViz Marker
    // ========================================================

    visualization_msgs::msg::Marker
      marker;


    marker.header.frame_id =
      output_frame_;

    marker.header.stamp =
      get_clock()->now();

    marker.ns =
      "target_estimator";

    marker.id =
      0;

    marker.action =
      visualization_msgs::msg::
        Marker::DELETE;


    marker_pub_->
      publish(
        marker);


    // ========================================================
    // 目标已经被MTC attach时，
    // 绝不能删除attached目标。
    // ========================================================

    if (
      publish_to_planning_scene_ &&
      planning_scene_interface_ &&
      target_in_scene_ &&
      !target_attached_)
    {
      moveit_msgs::msg::CollisionObject
        remove_object;


      remove_object.header.frame_id =
        output_frame_;

      remove_object.id =
        target_id_;

      remove_object.operation =
        moveit_msgs::msg::
          CollisionObject::REMOVE;


      planning_scene_interface_->
        applyCollisionObject(
          remove_object);


      target_in_scene_ =
        false;


      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "视觉目标丢失，已从PlanningScene删除%s",
        target_id_.c_str());
    }
  }


  // ==========================================================
  // 监听MTC attach / detach
  // ==========================================================

  void planningSceneCallback(
    const moveit_msgs::msg::
      PlanningScene::SharedPtr message)
  {
    if (!message->is_diff)
    {
      target_attached_ =
        false;
    }


    for (
      const auto & attached :
      message->
        robot_state.
        attached_collision_objects)
    {
      if (
        attached.object.id !=
        target_id_)
      {
        continue;
      }


      if (
        attached.object.operation ==
        moveit_msgs::msg::
          CollisionObject::REMOVE)
      {
        target_attached_ =
          false;

        // detach后重新从新鲜点云开始融合。
        world_cloud_history_.clear();

        RCLCPP_INFO(
          get_logger(),
          "%s已从机器人detach",
          target_id_.c_str());
      }
      else
      {
        target_attached_ =
          true;

        target_in_scene_ =
          false;

        // 目标已经随机器人运动，旧的静态World融合点云不能继续保留。
        world_cloud_history_.clear();

        RCLCPP_INFO(
          get_logger(),
          "%s已attach，停止World目标更新",
          target_id_.c_str());
      }
    }
  }


  // ==========================================================
  // Debug点云发布
  // ==========================================================

  void publishCloud(
    const rclcpp::Publisher<
      sensor_msgs::msg::PointCloud2>::
        SharedPtr & publisher,
    const CloudPtr & cloud,
    const std_msgs::msg::Header & header)
  {
    if (
      !publisher ||
      !cloud)
    {
      return;
    }


    sensor_msgs::msg::PointCloud2
      message;


    pcl::toROSMsg(
      *cloud,
      message);


    message.header =
      header;


    publisher->publish(
      message);
  }


private:

  // ==========================================================
  // Parameters
  // ==========================================================

  std::string
    input_cloud_topic_;

  std::string
    output_frame_;

  std::string
    selection_frame_;

  bool
    input_is_fused_cloud_;

  double workspace_min_x_;
  double workspace_max_x_;
  double workspace_min_y_;
  double workspace_max_y_;
  double workspace_min_z_;
  double workspace_max_z_;

  std::string
    target_id_;


  double
    processing_rate_;

  double
    min_depth_;

  double
    max_depth_;

  double
    voxel_leaf_size_;


  double
    plane_distance_threshold_;

  int
    plane_max_iterations_;

  int
    plane_min_inliers_;

  double
    plane_max_tilt_deg_;


  double
    cluster_tolerance_;

  int
    min_cluster_size_;

  int
    max_cluster_size_;


  double
    min_planar_dimension_;

  double
    max_planar_dimension_;

  double
    min_object_height_;

  double
    max_object_height_;


  double
    max_axis_ratio_;


  double
    pca_anisotropy_threshold_;

  double
    planar_percentile_low_;

  double
    planar_percentile_high_;


  int
    fusion_frames_;

  double
    fusion_max_age_;


  double
    smoothing_alpha_;

  double
    stable_position_threshold_;

  double
    stable_size_threshold_;

  int
    stable_frames_required_;

  double
    target_lost_timeout_;


  bool
    publish_to_planning_scene_;

  double
    scene_update_rate_;

  double
    collision_padding_;


  double
    tf_timeout_;


  // ==========================================================
  // ROS
  // ==========================================================

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::
      SharedPtr
    cloud_sub_;


  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::
      SharedPtr
    target_pose_pub_;


  rclcpp::Publisher<
    geometry_msgs::msg::Vector3Stamped>::
      SharedPtr
    target_size_pub_;


  rclcpp::Publisher<
    std_msgs::msg::Bool>::
      SharedPtr
    target_stable_pub_;


  rclcpp::Publisher<
    visualization_msgs::msg::Marker>::
      SharedPtr
    marker_pub_;


  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::
      SharedPtr
    filtered_cloud_pub_;


  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::
      SharedPtr
    plane_cloud_pub_;


  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::
      SharedPtr
    no_plane_cloud_pub_;


  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::
      SharedPtr
    target_cloud_pub_;


  // ==========================================================
  // TF
  // ==========================================================

  std::unique_ptr<
    tf2_ros::Buffer>
    tf_buffer_;


  std::shared_ptr<
    tf2_ros::TransformListener>
    tf_listener_;


  // ==========================================================
  // PlanningScene
  // ==========================================================

  std::unique_ptr<
    moveit::planning_interface::
      PlanningSceneInterface>
    planning_scene_interface_;


  rclcpp::Subscription<
    moveit_msgs::msg::PlanningScene>::
      SharedPtr
    planning_scene_sub_;


  bool
    target_attached_ =
      false;


  bool
    target_in_scene_ =
      false;


  // ==========================================================
  // World多帧融合状态
  // ==========================================================

  std::deque<WorldCloudFrame>
    world_cloud_history_;


  // ==========================================================
  // Estimate状态
  // ==========================================================

  bool
    has_raw_estimate_ =
      false;


  bool
    has_filtered_estimate_ =
      false;


  bool
    target_stable_ =
      false;


  int
    stable_count_ =
      0;


  geometry_msgs::msg::PoseStamped
    previous_raw_pose_;


  Eigen::Vector3d
    previous_raw_size_ =
      Eigen::Vector3d::Zero();


  geometry_msgs::msg::PoseStamped
    filtered_pose_;


  Eigen::Vector3d
    filtered_size_ =
      Eigen::Vector3d::Zero();


  rclcpp::Time
    last_process_time_;


  rclcpp::Time
    last_scene_update_time_;


  rclcpp::Time
    last_valid_detection_time_;
};


// ============================================================
// main
// ============================================================

int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);


  auto node =
    std::make_shared<
      TargetEstimator>();


  rclcpp::spin(
    node);


  rclcpp::shutdown();


  return 0;
}