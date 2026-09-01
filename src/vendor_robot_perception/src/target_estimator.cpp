#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>


#include <rclcpp/rclcpp.hpp>


#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <std_msgs/msg/bool.hpp>

#include <visualization_msgs/msg/marker.hpp>


#include <tf2/exceptions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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

    // 相机坐标系中的物体中心
    Eigen::Vector3f center;

    // 物体坐标系到相机坐标系的旋转矩阵
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


  TargetEstimator()
  : Node("target_estimator")
  {
    // ========================================================
    // 参数声明
    // ========================================================

    declare_parameter<std::string>(
      "input_cloud_topic",
      "/depth_camera/points");

    declare_parameter<std::string>(
      "output_frame",
      "world");

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
      0.004);


    declare_parameter<double>(
      "plane_distance_threshold",
      0.008);

    declare_parameter<int>(
      "plane_max_iterations",
      150);

    declare_parameter<int>(
      "plane_min_inliers",
      300);


    declare_parameter<double>(
      "cluster_tolerance",
      0.018);

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
      "smoothing_alpha",
      0.30);

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
      0.003);


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
      "PlanningScene目标ID: %s",
      target_id_.c_str());
  }


private:

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
    // 深度PassThrough
    //
    // ROS optical frame中：
    //
    // X -> 右
    // Y -> 下
    // Z -> 前
    //
    // 因此过滤Z即可限制相机观测距离。
    // ========================================================

    CloudPtr depth_cloud(
      new Cloud);


    pcl::PassThrough<PointT>
      pass;


    pass.setInputCloud(
      clean_cloud);

    pass.setFilterFieldName(
      "z");

    pass.setFilterLimits(
      static_cast<float>(
        min_depth_),

      static_cast<float>(
        max_depth_));


    pass.filter(
      *depth_cloud);


    if (depth_cloud->empty())
    {
      handleDetectionMiss();

      return;
    }


    // ========================================================
    // VoxelGrid
    // ========================================================

    CloudPtr filtered_cloud(
      new Cloud);


    pcl::VoxelGrid<PointT>
      voxel;


    voxel.setInputCloud(
      depth_cloud);

    voxel.setLeafSize(
      static_cast<float>(
        voxel_leaf_size_),

      static_cast<float>(
        voxel_leaf_size_),

      static_cast<float>(
        voxel_leaf_size_));


    voxel.filter(
      *filtered_cloud);


    publishCloud(
      filtered_cloud_pub_,
      filtered_cloud,
      message->header);


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
      pcl::SACMODEL_PLANE);

    segmentation.setMethodType(
      pcl::SAC_RANSAC);

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
        "没有找到足够稳定的桌面平面");

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
      message->header);


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
      message->header);


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
    // 让桌面法向朝向相机
    //
    // 相机原点为0。
    //
    // 平面上的最近点：
    //
    // p0 = -d * n
    //
    // 从平面指向相机：
    //
    // 0 - p0 = d*n
    //
    // 因此希望d为正。
    //
    // 对桌面上的物体而言：
    //
    // distance = n·p + d
    //
    // 会得到正的物体高度。
    // ========================================================

    if (plane_d < 0.0f)
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
    // 发布最终目标cluster
    // ========================================================

    publishCloud(
      target_cloud_pub_,
      best_candidate.cloud,
      message->header);


    // ========================================================
    // 相机坐标系中的目标Pose
    // ========================================================

    geometry_msgs::msg::PoseStamped
      pose_camera;


    pose_camera.header =
      message->header;


    pose_camera.pose.position.x =
      best_candidate.center.x();

    pose_camera.pose.position.y =
      best_candidate.center.y();

    pose_camera.pose.position.z =
      best_candidate.center.z();


    Eigen::Quaternionf
      quaternion_camera(
        best_candidate.rotation);


    quaternion_camera.normalize();


    pose_camera.pose.orientation.x =
      quaternion_camera.x();

    pose_camera.pose.orientation.y =
      quaternion_camera.y();

    pose_camera.pose.orientation.z =
      quaternion_camera.z();

    pose_camera.pose.orientation.w =
      quaternion_camera.w();


    // ========================================================
    // TF：
    //
    // depth_camera_optical_frame
    //            ↓
    //           world
    //
    // 这里必须使用点云自身的timestamp。
    // ========================================================

    geometry_msgs::msg::PoseStamped
      pose_output;


    try
    {
      const auto transform =
        tf_buffer_->lookupTransform(
          output_frame_,
          message->header.frame_id,
          message_time,
          rclcpp::Duration::from_seconds(
            tf_timeout_));


      tf2::doTransform(
        pose_camera,
        pose_output,
        transform);
    }
    catch (
      const tf2::TransformException &
        exception)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "TF转换失败 %s -> %s: %s",
        message->header.frame_id.c_str(),
        output_frame_.c_str(),
        exception.what());

      handleDetectionMiss();

      return;
    }


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
    // 平滑
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
    // 把cluster投影到桌面，并计算每个点高于桌面的高度
    // ========================================================

    std::vector<Eigen::Vector3f>
      projected_points;


    std::vector<float>
      heights;


    projected_points.reserve(
      cluster->size());

    heights.reserve(
      cluster->size());


    Eigen::Vector3f
      mean_projected =
        Eigen::Vector3f::Zero();


    for (
      const auto & point :
      cluster->points)
    {
      Eigen::Vector3f p(
        point.x,
        point.y,
        point.z);


      const float height =
        plane_normal.dot(p) +
        plane_d;


      // 只考虑桌面上方的点。
      //
      // 少量噪声可能落在平面下面。
      if (height <= 0.0f)
      {
        continue;
      }


      const Eigen::Vector3f
        projected =
          p -
          height *
          plane_normal;


      projected_points.push_back(
        projected);

      heights.push_back(
        height);


      mean_projected +=
        projected;
    }


    if (
      projected_points.size() <
      static_cast<std::size_t>(
        min_cluster_size_))
    {
      return false;
    }


    mean_projected /=
      static_cast<float>(
        projected_points.size());


    // ========================================================
    // 用95百分位估计物体高度
    //
    // 比直接使用max更不容易被离群点影响。
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
    // 计算投影点的协方差
    // ========================================================

    Eigen::Matrix3f covariance =
      Eigen::Matrix3f::Zero();


    for (
      const auto & point :
      projected_points)
    {
      const Eigen::Vector3f delta =
        point -
        mean_projected;

      covariance +=
        delta *
        delta.transpose();
    }


    covariance /=
      static_cast<float>(
        projected_points.size());


    Eigen::SelfAdjointEigenSolver<
      Eigen::Matrix3f>
      eigen_solver(
        covariance);


    if (
      eigen_solver.info() !=
      Eigen::Success)
    {
      return false;
    }


    // ========================================================
    // 最大特征值方向作为物体X轴
    // ========================================================

    Eigen::Vector3f axis_x =
      eigen_solver
        .eigenvectors()
        .col(2);


    // 再次投影到桌面上，避免数值误差产生法向分量
    axis_x -=
      plane_normal *
      axis_x.dot(
        plane_normal);


    if (
      axis_x.norm() <
      1e-5f)
    {
      return false;
    }


    axis_x.normalize();


    // ========================================================
    // 减少PCA方向180°随机翻转
    //
    // 使用相机X轴在桌面上的投影作为参考方向。
    // ========================================================

    Eigen::Vector3f reference_axis(
      1.0f,
      0.0f,
      0.0f);


    reference_axis -=
      plane_normal *
      reference_axis.dot(
        plane_normal);


    if (
      reference_axis.norm() <
      0.1f)
    {
      reference_axis =
        Eigen::Vector3f(
          0.0f,
          1.0f,
          0.0f);

      reference_axis -=
        plane_normal *
        reference_axis.dot(
          plane_normal);
    }


    if (
      reference_axis.norm() >
      1e-5f)
    {
      reference_axis.normalize();


      if (
        axis_x.dot(
          reference_axis) <
        0.0f)
      {
        axis_x =
          -axis_x;
      }
    }


    // ========================================================
    // 创建右手坐标系
    //
    // X × Y = Z
    // ========================================================

    Eigen::Vector3f axis_y =
      plane_normal.cross(
        axis_x);


    if (
      axis_y.norm() <
      1e-5f)
    {
      return false;
    }


    axis_y.normalize();


    // ========================================================
    // 计算物体在桌面上的二维包围盒
    // ========================================================

    float min_u =
      std::numeric_limits<float>::max();

    float max_u =
      std::numeric_limits<float>::lowest();

    float min_v =
      std::numeric_limits<float>::max();

    float max_v =
      std::numeric_limits<float>::lowest();


    for (
      const auto & point :
      projected_points)
    {
      const float u =
        axis_x.dot(point);

      const float v =
        axis_y.dot(point);


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


    const float size_x =
      max_u -
      min_u;

    const float size_y =
      max_v -
      min_v;


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
    // 计算物体体积中心
    //
    // 平面上距相机最近的原点：
    //
    // p0 = -d*n
    //
    // 物体底面中心：
    //
    // p_bottom =
    //   p0 + u_center*x + v_center*y
    //
    // 物体中心：
    //
    // p_center =
    //   p_bottom + height/2*n
    // ========================================================

    const float center_u =
      0.5f *
      (min_u + max_u);

    const float center_v =
      0.5f *
      (min_v + max_v);


    const Eigen::Vector3f
      plane_origin =
        -plane_d *
        plane_normal;


    const Eigen::Vector3f
      bottom_center =
        plane_origin +
        center_u *
        axis_x +
        center_v *
        axis_y;


    const Eigen::Vector3f
      object_center =
        bottom_center +
        0.5f *
        object_height *
        plane_normal;


    // ========================================================
    // 目标必须位于相机前方
    // ========================================================

    if (
      object_center.z() <=
      1e-4f)
    {
      return false;
    }


    // ========================================================
    // 计算目标距相机光轴的角度代价
    //
    // 越接近图像中心，代价越小。
    // ========================================================

    const double axis_ratio =
      std::hypot(
        object_center.x(),
        object_center.y()) /
      object_center.z();


    if (
      axis_ratio >
      max_axis_ratio_)
    {
      return false;
    }


    // ========================================================
    // 当前简单场景：
    //
    // 优先选择最接近相机光轴的目标。
    //
    // 深度只作为一个很弱的次级代价。
    // ========================================================

    const double score =
      axis_ratio +
      0.02 *
      object_center.z();


    // ========================================================
    // 输出
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
      score;


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
