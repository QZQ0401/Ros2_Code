#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>


class BinEstimator : public rclcpp::Node
{
public:

  using PointT = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = Cloud::Ptr;


  BinEstimator()
  : Node("bin_estimator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ========================================================
    // Input/output
    // ========================================================

    input_cloud_topic_ =
      declare_parameter<std::string>(
        "input_cloud_topic",
        "/workspace/merged_points");

    output_frame_ =
      declare_parameter<std::string>(
        "output_frame",
        "world");

    target_size_topic_ =
      declare_parameter<std::string>(
        "target_size_topic",
        "/perception/target_size");


    // ========================================================
    // Bin search ROI.
    //
    // Defaults match the left-side placement area used in the
    // current Gazebo scene. Adjust these when the basket moves.
    // ========================================================

    roi_min_x_ =
      declare_parameter<double>(
        "roi_min_x",
        -1.00);

    roi_max_x_ =
      declare_parameter<double>(
        "roi_max_x",
        -0.30);

    roi_min_y_ =
      declare_parameter<double>(
        "roi_min_y",
        0.35);

    roi_max_y_ =
      declare_parameter<double>(
        "roi_max_y",
        1.05);

    roi_min_z_ =
      declare_parameter<double>(
        "roi_min_z",
        0.75);

    roi_max_z_ =
      declare_parameter<double>(
        "roi_max_z",
        1.20);


    // ========================================================
    // Filtering / table plane
    // ========================================================

    processing_rate_ =
      declare_parameter<double>(
        "processing_rate",
        8.0);

    voxel_leaf_size_ =
      declare_parameter<double>(
        "voxel_leaf_size",
        0.004);

    plane_distance_threshold_ =
      declare_parameter<double>(
        "plane_distance_threshold",
        0.006);

    plane_max_iterations_ =
      declare_parameter<int>(
        "plane_max_iterations",
        150);

    plane_min_inliers_ =
      declare_parameter<int>(
        "plane_min_inliers",
        120);

    plane_max_tilt_deg_ =
      declare_parameter<double>(
        "plane_max_tilt_deg",
        10.0);


    // ========================================================
    // Bin wall extraction.
    //
    // The basket is open on top. We estimate its outer XY rectangle
    // from points above the table plane, while rejecting the table
    // and the thin bottom plate.
    // ========================================================

    wall_min_height_ =
      declare_parameter<double>(
        "wall_min_height",
        0.025);

    wall_max_height_ =
      declare_parameter<double>(
        "wall_max_height",
        0.35);

    min_bin_points_ =
      declare_parameter<int>(
        "min_bin_points",
        80);

    // ========================================================
    // Known basket geometry from pick_scene.world.
    //
    // placement_bin:
    //   outer footprint  = 0.40 x 0.32 m
    //   base thickness   = 0.03 m
    //   wall thickness   = 0.025 m
    //   wall height      = 0.16 m
    //   total height     = 0.19 m
    //   inner opening    = 0.35 x 0.27 m
    //
    // Point cloud is now used ONLY to estimate pose.
    // The measured rectangle size is only a sanity check.
    // ========================================================

    known_outer_x_ =
      declare_parameter<double>(
        "known_outer_x",
        0.40);

    known_outer_y_ =
      declare_parameter<double>(
        "known_outer_y",
        0.32);

    known_base_thickness_ =
      declare_parameter<double>(
        "known_base_thickness",
        0.03);

    known_wall_thickness_ =
      declare_parameter<double>(
        "known_wall_thickness",
        0.025);

    known_wall_height_ =
      declare_parameter<double>(
        "known_wall_height",
        0.16);

    // Because an open basket can be partially occluded, the observed
    // rectangle is allowed to be smaller than the true model.
    min_observed_fraction_ =
      declare_parameter<double>(
        "min_observed_fraction",
        0.55);

    max_observed_oversize_ =
      declare_parameter<double>(
        "max_observed_oversize",
        0.08);


    // ========================================================
    // Final object place pose.
    //
    // /perception/place_pose represents the desired FINAL OBJECT pose,
    // not the end-effector pose.
    // ========================================================

    place_clearance_ =
      declare_parameter<double>(
        "place_clearance",
        0.005);

    fit_margin_ =
      declare_parameter<double>(
        "fit_margin",
        0.015);

    default_object_height_ =
      declare_parameter<double>(
        "default_object_height",
        0.05);

    tf_timeout_ =
      declare_parameter<double>(
        "tf_timeout",
        0.20);


    // Optional: publish the perceived basket pose using the exact
    // five-box geometry from pick_scene.world.
    //
    // Default false preserves the original point-cloud / OctoMap logic.
    publish_to_planning_scene_ =
      declare_parameter<bool>(
        "publish_to_planning_scene",
        false);

    collision_object_id_ =
      declare_parameter<std::string>(
        "collision_object_id",
        "placement_bin_vision");


    if (
      roi_min_x_ >= roi_max_x_ ||
      roi_min_y_ >= roi_max_y_ ||
      roi_min_z_ >= roi_max_z_)
    {
      throw std::runtime_error(
        "bin ROI min values must be smaller than max values");
    }


    if (
      known_outer_x_ <= 0.0 ||
      known_outer_y_ <= 0.0 ||
      known_base_thickness_ <= 0.0 ||
      known_wall_thickness_ <= 0.0 ||
      known_wall_height_ <= 0.0)
    {
      throw std::runtime_error(
        "known bin dimensions must be positive");
    }


    if (
      2.0 * known_wall_thickness_ >= known_outer_x_ ||
      2.0 * known_wall_thickness_ >= known_outer_y_)
    {
      throw std::runtime_error(
        "known wall thickness leaves no basket opening");
    }


    // ========================================================
    // Publishers
    // ========================================================

    const auto latched_qos =
      rclcpp::QoS(1)
        .reliable()
        .transient_local();


    place_pose_pub_ =
      create_publisher<
        geometry_msgs::msg::PoseStamped>(
          "/perception/place_pose",
          latched_qos);


    bin_pose_pub_ =
      create_publisher<
        geometry_msgs::msg::PoseStamped>(
          "/perception/bin_pose",
          latched_qos);


    bin_size_pub_ =
      create_publisher<
        geometry_msgs::msg::Vector3Stamped>(
          "/perception/bin_size",
          latched_qos);


    bin_cloud_pub_ =
      create_publisher<
        sensor_msgs::msg::PointCloud2>(
          "/perception/bin_cloud",
          rclcpp::SensorDataQoS());


    marker_pub_ =
      create_publisher<
        visualization_msgs::msg::Marker>(
          "/perception/bin_marker",
          10);


    // ========================================================
    // Subscribers
    // ========================================================

    cloud_sub_ =
      create_subscription<
        sensor_msgs::msg::PointCloud2>(
          input_cloud_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(
            &BinEstimator::cloudCallback,
            this,
            std::placeholders::_1));


    target_size_sub_ =
      create_subscription<
        geometry_msgs::msg::Vector3Stamped>(
          target_size_topic_,
          latched_qos,
          std::bind(
            &BinEstimator::targetSizeCallback,
            this,
            std::placeholders::_1));


    if (publish_to_planning_scene_)
    {
      planning_scene_interface_ =
        std::make_unique<
          moveit::planning_interface::
            PlanningSceneInterface>(
              "",
              true);
    }


    RCLCPP_INFO(
      get_logger(),
      "bin_estimator: cloud='%s', place_pose='/perception/place_pose'",
      input_cloud_topic_.c_str());
  }


private:

  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };


  struct MinAreaRect2D
  {
    bool valid{false};

    double center_x{0.0};
    double center_y{0.0};

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
        area >= best.area)
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


  void targetSizeCallback(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
  {
    target_size_ =
      Eigen::Vector3d(
        message->vector.x,
        message->vector.y,
        message->vector.z);

    has_target_size_ =
      true;
  }


  CloudPtr cropAxis(
    const CloudPtr & input,
    const std::string & field,
    const double min_value,
    const double max_value) const
  {
    CloudPtr output(
      new Cloud);


    pcl::PassThrough<PointT>
      pass;


    pass.setInputCloud(
      input);

    pass.setFilterFieldName(
      field);

    pass.setFilterLimits(
      static_cast<float>(
        min_value),
      static_cast<float>(
        max_value));

    pass.filter(
      *output);


    return output;
  }


  bool transformCloudToOutput(
    const sensor_msgs::msg::PointCloud2 & message,
    CloudPtr & output)
  {
    CloudPtr input(
      new Cloud);


    pcl::fromROSMsg(
      message,
      *input);


    CloudPtr clean(
      new Cloud);


    std::vector<int>
      valid_indices;


    pcl::removeNaNFromPointCloud(
      *input,
      *clean,
      valid_indices);


    if (clean->empty())
    {
      return false;
    }


    if (
      message.header.frame_id ==
      output_frame_)
    {
      output =
        clean;

      return true;
    }


    geometry_msgs::msg::TransformStamped
      transform;


    try
    {
      transform =
        tf_buffer_.lookupTransform(
          output_frame_,
          message.header.frame_id,
          rclcpp::Time(
            message.header.stamp),
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
        "bin TF %s <- %s failed: %s",
        output_frame_.c_str(),
        message.header.frame_id.c_str(),
        exception.what());

      return false;
    }


    const auto & t =
      transform.transform.translation;

    const auto & q_message =
      transform.transform.rotation;


    Eigen::Quaternionf q(
      static_cast<float>(
        q_message.w),
      static_cast<float>(
        q_message.x),
      static_cast<float>(
        q_message.y),
      static_cast<float>(
        q_message.z));


    if (q.norm() < 1e-6f)
    {
      return false;
    }


    q.normalize();


    Eigen::Affine3f tf =
      Eigen::Affine3f::Identity();


    tf.linear() =
      q.toRotationMatrix();

    tf.translation() <<
      static_cast<float>(t.x),
      static_cast<float>(t.y),
      static_cast<float>(t.z);


    output.reset(
      new Cloud);


    pcl::transformPointCloud(
      *clean,
      *output,
      tf);


    return !output->empty();
  }


  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const rclcpp::Time stamp(
      message->header.stamp,
      get_clock()->get_clock_type());


    if (
      processing_rate_ > 0.0 &&
      last_process_time_.nanoseconds() != 0)
    {
      const double dt =
        (
          stamp -
          last_process_time_
        ).seconds();


      if (
        dt >= 0.0 &&
        dt <
          1.0 /
          processing_rate_)
      {
        return;
      }
    }


    last_process_time_ =
      stamp;


    CloudPtr world_cloud;


    if (
      !transformCloudToOutput(
        *message,
        world_cloud))
    {
      return;
    }


    // ========================================================
    // Bin ROI
    // ========================================================

    CloudPtr roi =
      cropAxis(
        world_cloud,
        "x",
        roi_min_x_,
        roi_max_x_);


    roi =
      cropAxis(
        roi,
        "y",
        roi_min_y_,
        roi_max_y_);


    roi =
      cropAxis(
        roi,
        "z",
        roi_min_z_,
        roi_max_z_);


    if (
      roi->size() <
      static_cast<std::size_t>(
        plane_min_inliers_))
    {
      return;
    }


    CloudPtr filtered(
      new Cloud);


    pcl::VoxelGrid<PointT>
      voxel;


    voxel.setInputCloud(
      roi);


    const float leaf =
      static_cast<float>(
        voxel_leaf_size_);


    voxel.setLeafSize(
      leaf,
      leaf,
      leaf);


    voxel.filter(
      *filtered);


    if (
      filtered->size() <
      static_cast<std::size_t>(
        plane_min_inliers_))
    {
      return;
    }


    // ========================================================
    // Detect table plane near +Z.
    // ========================================================

    pcl::PointIndices::Ptr
      plane_inliers(
        new pcl::PointIndices);


    pcl::ModelCoefficients::Ptr
      plane_coefficients(
        new pcl::ModelCoefficients);


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


    const double max_tilt_rad =
      std::clamp(
        plane_max_tilt_deg_,
        0.1,
        89.0) *
      std::acos(-1.0) /
      180.0;


    segmentation.setEpsAngle(
      max_tilt_rad);

    segmentation.setMaxIterations(
      plane_max_iterations_);

    segmentation.setDistanceThreshold(
      plane_distance_threshold_);

    segmentation.setInputCloud(
      filtered);


    segmentation.segment(
      *plane_inliers,
      *plane_coefficients);


    if (
      plane_inliers->indices.size() <
        static_cast<std::size_t>(
          plane_min_inliers_) ||
      plane_coefficients->values.size() < 4)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "bin_estimator: no stable table plane in bin ROI");

      return;
    }


    Eigen::Vector3f plane_normal(
      plane_coefficients->values[0],
      plane_coefficients->values[1],
      plane_coefficients->values[2]);


    const float normal_norm =
      plane_normal.norm();


    if (normal_norm < 1e-6f)
    {
      return;
    }


    plane_normal /=
      normal_norm;


    float plane_d =
      plane_coefficients->values[3] /
      normal_norm;


    if (plane_normal.z() < 0.0f)
    {
      plane_normal =
        -plane_normal;

      plane_d =
        -plane_d;
    }


    // ========================================================
    // Deterministic table basis.
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
      return;
    }


    basis_x.normalize();


    Eigen::Vector3f basis_y =
      plane_normal.cross(
        basis_x);


    if (basis_y.norm() < 1e-5f)
    {
      return;
    }


    basis_y.normalize();


    const Eigen::Vector3f plane_origin =
      -plane_d *
      plane_normal;


    // ========================================================
    // Extract wall/rim points:
    // remove table and bottom plate by positive height threshold.
    // ========================================================

    CloudPtr bin_cloud(
      new Cloud);


    std::vector<Point2D>
      projected_2d;


    std::vector<float>
      wall_heights;


    projected_2d.reserve(
      filtered->size());

    wall_heights.reserve(
      filtered->size());


    for (
      const auto & point :
      filtered->points)
    {
      const Eigen::Vector3f p(
        point.x,
        point.y,
        point.z);


      const float height =
        plane_normal.dot(p) +
        plane_d;


      if (
        height <
          wall_min_height_ ||
        height >
          wall_max_height_)
      {
        continue;
      }


      bin_cloud->push_back(
        point);


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


      wall_heights.push_back(
        height);
    }


    if (
      projected_2d.size() <
      static_cast<std::size_t>(
        min_bin_points_))
    {
      return;
    }


    // ========================================================
    // Pose from minimum-area rectangle; size comes from known XML geometry.
    // ========================================================

    const MinAreaRect2D rectangle =
      minimumAreaRectangle2D(
        projected_2d);


    if (!rectangle.valid)
    {
      return;
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


    axis_u.normalize();

    axis_v.normalize();


    // Measured rectangle is used only to identify / orient the basket.
    // Final basket size comes from the XML model.
    double measured_x =
      rectangle.size_u;

    double measured_y =
      rectangle.size_v;


    Eigen::Vector3f axis_x =
      axis_u;

    Eigen::Vector3f axis_y =
      axis_v;


    // The XML basket is longer in X (0.40 > 0.32).
    // Make the detected longer rectangle side the local basket X axis.
    if (measured_y > measured_x)
    {
      std::swap(
        measured_x,
        measured_y);

      axis_x =
        axis_v;

      axis_y =
        -axis_u;
    }


    // Sanity check only. Partial walls may make the observed footprint
    // smaller than the real basket, so under-observation is tolerated.
    const bool observed_size_plausible =
      measured_x >=
        min_observed_fraction_ *
        known_outer_x_ &&
      measured_y >=
        min_observed_fraction_ *
        known_outer_y_ &&
      measured_x <=
        known_outer_x_ +
        max_observed_oversize_ &&
      measured_y <=
        known_outer_y_ +
        max_observed_oversize_;


    if (!observed_size_plausible)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "bin footprint inconsistent with known model: "
        "measured=(%.3f %.3f), known=(%.3f %.3f)",
        measured_x,
        measured_y,
        known_outer_x_,
        known_outer_y_);

      return;
    }


    // Final geometry: exact values from pick_scene.world.
    const double outer_x =
      known_outer_x_;

    const double outer_y =
      known_outer_y_;

    const double total_height =
      known_base_thickness_ +
      known_wall_height_;


    // Resolve 180-degree sign using world X/Y.
    if (
      std::abs(axis_x.x()) >=
      std::abs(axis_x.y()))
    {
      if (axis_x.x() < 0.0f)
      {
        axis_x =
          -axis_x;

        axis_y =
          -axis_y;
      }
    }
    else if (axis_x.y() < 0.0f)
    {
      axis_x =
        -axis_x;

      axis_y =
        -axis_y;
    }


    axis_y =
      plane_normal.cross(
        axis_x);

    axis_y.normalize();

    axis_x =
      axis_y.cross(
        plane_normal);

    axis_x.normalize();


    const Eigen::Vector3f bottom_center =
      plane_origin +
      static_cast<float>(
        rectangle.center_x) *
        basis_x +
      static_cast<float>(
        rectangle.center_y) *
        basis_y;


    const float observed_rim_height =
      percentile(
        wall_heights,
        0.95);


    // ========================================================
    // Opening size and object-fit check.
    // ========================================================

    const double opening_x =
      std::max(
        0.0,
        outer_x -
        2.0 *
        known_wall_thickness_);


    const double opening_y =
      std::max(
        0.0,
        outer_y -
        2.0 *
        known_wall_thickness_);


    double object_x =
      has_target_size_ ?
        target_size_.x() :
        0.0;

    double object_y =
      has_target_size_ ?
        target_size_.y() :
        0.0;

    double object_z =
      has_target_size_ ?
        target_size_.z() :
        default_object_height_;


    // target_estimator minimum-area rectangle defines target X as
    // its longer planar side. Keep the same convention here.
    if (object_y > object_x)
    {
      std::swap(
        object_x,
        object_y);
    }


    if (has_target_size_)
    {
      const bool fits =
        object_x +
          2.0 *
          fit_margin_ <=
            opening_x &&
        object_y +
          2.0 *
          fit_margin_ <=
            opening_y;


      if (!fits)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "target does not fit bin opening: object=(%.3f %.3f), opening=(%.3f %.3f)",
          object_x,
          object_y,
          opening_x,
          opening_y);

        return;
      }
    }


    // ========================================================
    // /perception/place_pose
    //
    // This is the desired FINAL OBJECT CENTER pose inside the basket.
    // The bottom reference uses the detected table plane plus a known
    // basket-bottom offset/thickness.
    // ========================================================

    const double center_height =
      known_base_thickness_ +
      0.5 *
      object_z +
      place_clearance_;


    const Eigen::Vector3f place_center =
      bottom_center +
      static_cast<float>(
        center_height) *
      plane_normal;


    Eigen::Matrix3f rotation;

    rotation.col(0) =
      axis_x;

    rotation.col(1) =
      axis_y;

    rotation.col(2) =
      plane_normal;


    Eigen::Quaternionf q(
      rotation);

    q.normalize();


    std_msgs::msg::Header header;

    header.stamp =
      message->header.stamp;

    header.frame_id =
      output_frame_;


    geometry_msgs::msg::PoseStamped place_pose;

    place_pose.header =
      header;

    place_pose.pose.position.x =
      place_center.x();

    place_pose.pose.position.y =
      place_center.y();

    place_pose.pose.position.z =
      place_center.z();

    place_pose.pose.orientation.x =
      q.x();

    place_pose.pose.orientation.y =
      q.y();

    place_pose.pose.orientation.z =
      q.z();

    place_pose.pose.orientation.w =
      q.w();


    place_pose_pub_->publish(
      place_pose);


    // ========================================================
    // Additional bin diagnostics.
    // ========================================================

    geometry_msgs::msg::PoseStamped bin_pose;

    bin_pose.header =
      header;

    // /perception/bin_pose matches the SDF <model pose> semantic:
    // footprint center at the table / basket-bottom origin.
    bin_pose.pose.position.x =
      bottom_center.x();

    bin_pose.pose.position.y =
      bottom_center.y();

    bin_pose.pose.position.z =
      bottom_center.z();

    bin_pose.pose.orientation =
      place_pose.pose.orientation;


    bin_pose_pub_->publish(
      bin_pose);


    geometry_msgs::msg::Vector3Stamped bin_size;

    bin_size.header =
      header;

    bin_size.vector.x =
      outer_x;

    bin_size.vector.y =
      outer_y;

    bin_size.vector.z =
      total_height;


    bin_size_pub_->publish(
      bin_size);


    sensor_msgs::msg::PointCloud2 bin_cloud_message;

    pcl::toROSMsg(
      *bin_cloud,
      bin_cloud_message);

    bin_cloud_message.header =
      header;

    bin_cloud_pub_->publish(
      bin_cloud_message);


    publishMarkers(
      header,
      bin_pose,
      outer_x,
      outer_y,
      total_height,
      place_pose);


    if (publish_to_planning_scene_)
    {
      publishKnownBinCollisionObject(
        header,
        bottom_center,
        q);
    }


    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "bin pose=(%.3f %.3f %.3f), "
      "known=(%.3f %.3f %.3f), measured_xy=(%.3f %.3f), "
      "observed_rim=%.3f, opening=(%.3f %.3f), "
      "place=(%.3f %.3f %.3f)",
      bin_pose.pose.position.x,
      bin_pose.pose.position.y,
      bin_pose.pose.position.z,
      outer_x,
      outer_y,
      total_height,
      measured_x,
      measured_y,
      observed_rim_height,
      opening_x,
      opening_y,
      place_pose.pose.position.x,
      place_pose.pose.position.y,
      place_pose.pose.position.z);
  }


  void publishMarkers(
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::PoseStamped & bin_pose,
    const double outer_x,
    const double outer_y,
    const double rim_height,
    const geometry_msgs::msg::PoseStamped & place_pose)
  {
    visualization_msgs::msg::Marker bin_marker;

    bin_marker.header =
      header;

    bin_marker.ns =
      "bin_estimator";

    bin_marker.id =
      0;

    bin_marker.type =
      visualization_msgs::msg::Marker::CUBE;

    bin_marker.action =
      visualization_msgs::msg::Marker::ADD;

    bin_marker.pose =
      bin_pose.pose;

    // Marker center is half the known total height above model origin.
    {
      Eigen::Quaterniond marker_q(
        bin_pose.pose.orientation.w,
        bin_pose.pose.orientation.x,
        bin_pose.pose.orientation.y,
        bin_pose.pose.orientation.z);

      marker_q.normalize();

      const Eigen::Vector3d marker_offset =
        marker_q *
        Eigen::Vector3d(
          0.0,
          0.0,
          0.5 * rim_height);

      bin_marker.pose.position.x +=
        marker_offset.x();

      bin_marker.pose.position.y +=
        marker_offset.y();

      bin_marker.pose.position.z +=
        marker_offset.z();
    }

    bin_marker.scale.x =
      outer_x;

    bin_marker.scale.y =
      outer_y;

    bin_marker.scale.z =
      std::max(
        rim_height,
        0.001);

    bin_marker.color.r =
      0.1f;

    bin_marker.color.g =
      0.4f;

    bin_marker.color.b =
      1.0f;

    bin_marker.color.a =
      0.18f;

    marker_pub_->publish(
      bin_marker);


    visualization_msgs::msg::Marker place_marker;

    place_marker.header =
      header;

    place_marker.ns =
      "bin_estimator";

    place_marker.id =
      1;

    place_marker.type =
      visualization_msgs::msg::Marker::ARROW;

    place_marker.action =
      visualization_msgs::msg::Marker::ADD;

    place_marker.pose =
      place_pose.pose;

    place_marker.scale.x =
      0.12;

    place_marker.scale.y =
      0.02;

    place_marker.scale.z =
      0.02;

    place_marker.color.r =
      0.2f;

    place_marker.color.g =
      1.0f;

    place_marker.color.b =
      0.2f;

    place_marker.color.a =
      0.9f;

    marker_pub_->publish(
      place_marker);
  }


  void publishKnownBinCollisionObject(
    const std_msgs::msg::Header & header,
    const Eigen::Vector3f & bottom_center,
    const Eigen::Quaternionf & orientation)
  {
    if (!planning_scene_interface_)
    {
      return;
    }


    moveit_msgs::msg::CollisionObject object;

    object.header =
      header;

    object.id =
      collision_object_id_;

    object.operation =
      moveit_msgs::msg::CollisionObject::ADD;


    const Eigen::Quaterniond q =
      orientation.cast<double>();


    const auto add_box =
      [&](
        const double size_x,
        const double size_y,
        const double size_z,
        const Eigen::Vector3d & local_center)
      {
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type =
          shape_msgs::msg::SolidPrimitive::BOX;

        primitive.dimensions.resize(3);

        primitive.dimensions[
          shape_msgs::msg::SolidPrimitive::BOX_X] =
          size_x;

        primitive.dimensions[
          shape_msgs::msg::SolidPrimitive::BOX_Y] =
          size_y;

        primitive.dimensions[
          shape_msgs::msg::SolidPrimitive::BOX_Z] =
          size_z;


        const Eigen::Vector3d world_center =
          bottom_center.cast<double>() +
          q * local_center;


        geometry_msgs::msg::Pose pose;

        pose.position.x =
          world_center.x();

        pose.position.y =
          world_center.y();

        pose.position.z =
          world_center.z();

        pose.orientation.x =
          q.x();

        pose.orientation.y =
          q.y();

        pose.orientation.z =
          q.z();

        pose.orientation.w =
          q.w();


        object.primitives.push_back(
          primitive);

        object.primitive_poses.push_back(
          pose);
      };


    // Exact geometry from pick_scene.world.
    add_box(
      known_outer_x_,
      known_outer_y_,
      known_base_thickness_,
      Eigen::Vector3d(
        0.0,
        0.0,
        0.5 * known_base_thickness_));


    const double wall_center_z =
      known_base_thickness_ +
      0.5 * known_wall_height_;


    const double wall_y =
      0.5 *
      (known_outer_y_ -
       known_wall_thickness_);


    const double wall_x =
      0.5 *
      (known_outer_x_ -
       known_wall_thickness_);


    // +Y / -Y walls.
    add_box(
      known_outer_x_,
      known_wall_thickness_,
      known_wall_height_,
      Eigen::Vector3d(
        0.0,
        wall_y,
        wall_center_z));

    add_box(
      known_outer_x_,
      known_wall_thickness_,
      known_wall_height_,
      Eigen::Vector3d(
        0.0,
        -wall_y,
        wall_center_z));


    // +X / -X walls.
    add_box(
      known_wall_thickness_,
      known_outer_y_,
      known_wall_height_,
      Eigen::Vector3d(
        wall_x,
        0.0,
        wall_center_z));

    add_box(
      known_wall_thickness_,
      known_outer_y_,
      known_wall_height_,
      Eigen::Vector3d(
        -wall_x,
        0.0,
        wall_center_z));


    const bool success =
      planning_scene_interface_->
        applyCollisionObject(
          object);


    if (!success)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "failed to apply perceived basket CollisionObject '%s'",
        collision_object_id_.c_str());
    }
  }


private:

  std::string input_cloud_topic_;
  std::string output_frame_;
  std::string target_size_topic_;

  double roi_min_x_;
  double roi_max_x_;
  double roi_min_y_;
  double roi_max_y_;
  double roi_min_z_;
  double roi_max_z_;

  double processing_rate_;
  double voxel_leaf_size_;

  double plane_distance_threshold_;
  int plane_max_iterations_;
  int plane_min_inliers_;
  double plane_max_tilt_deg_;

  double wall_min_height_;
  double wall_max_height_;
  int min_bin_points_;

  double known_outer_x_;
  double known_outer_y_;
  double known_base_thickness_;
  double known_wall_thickness_;
  double known_wall_height_;

  double min_observed_fraction_;
  double max_observed_oversize_;

  double place_clearance_;
  double fit_margin_;
  double default_object_height_;

  double tf_timeout_;

  bool publish_to_planning_scene_{false};
  std::string collision_object_id_;

  std::unique_ptr<
    moveit::planning_interface::
      PlanningSceneInterface>
    planning_scene_interface_;

  bool has_target_size_{false};
  Eigen::Vector3d target_size_{
    Eigen::Vector3d::Zero()};

  rclcpp::Time last_process_time_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::Vector3Stamped>::SharedPtr
    target_size_sub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    place_pose_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    bin_pose_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::Vector3Stamped>::SharedPtr
    bin_size_pub_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    bin_cloud_pub_;

  rclcpp::Publisher<
    visualization_msgs::msg::Marker>::SharedPtr
    marker_pub_;
};


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);


  rclcpp::spin(
    std::make_shared<
      BinEstimator>());


  rclcpp::shutdown();


  return 0;
}