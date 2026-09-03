#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MultiCameraCloudFusion : public rclcpp::Node
{
public:
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using PointT = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = Cloud::Ptr;
  using ApproximatePolicy =
    message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;

  MultiCameraCloudFusion()
  : Node("multi_camera_cloud_fusion"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    topic_1_ = declare_parameter<std::string>(
      "input_topic_1", "/workspace_camera/points");
    topic_2_ = declare_parameter<std::string>(
      "input_topic_2", "/workspace_camera_2/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/workspace/merged_points");

    debug_topic_1_ = declare_parameter<std::string>(
      "debug_topic_1", "/workspace/camera_1_world");
    debug_topic_2_ = declare_parameter<std::string>(
      "debug_topic_2", "/workspace/camera_2_world");
    publish_debug_clouds_ =
      declare_parameter<bool>("publish_debug_clouds", true);

    voxel_leaf_size_ =
      declare_parameter<double>("voxel_leaf_size", 0.003);
    sync_slop_ =
      declare_parameter<double>("sync_slop", 0.08);
    tf_timeout_ =
      declare_parameter<double>("tf_timeout", 0.20);
    sync_queue_size_ =
      declare_parameter<int>("sync_queue_size", 10);

    crop_to_workspace_ =
      declare_parameter<bool>("crop_to_workspace", true);
    workspace_min_x_ =
      declare_parameter<double>("workspace_min_x", -1.10);
    workspace_max_x_ =
      declare_parameter<double>("workspace_max_x", 1.10);
    workspace_min_y_ =
      declare_parameter<double>("workspace_min_y", 0.25);
    workspace_max_y_ =
      declare_parameter<double>("workspace_max_y", 1.15);
    workspace_min_z_ =
      declare_parameter<double>("workspace_min_z", 0.76);
    workspace_max_z_ =
      declare_parameter<double>("workspace_max_z", 1.45);

    if (voxel_leaf_size_ <= 0.0) {
      throw std::runtime_error("voxel_leaf_size must be > 0");
    }
    if (sync_slop_ <= 0.0) {
      throw std::runtime_error("sync_slop must be > 0");
    }
    if (tf_timeout_ <= 0.0) {
      throw std::runtime_error("tf_timeout must be > 0");
    }
    if (sync_queue_size_ <= 0) {
      throw std::runtime_error("sync_queue_size must be > 0");
    }
    if (workspace_min_x_ >= workspace_max_x_ ||
        workspace_min_y_ >= workspace_max_y_ ||
        workspace_min_z_ >= workspace_max_z_)
    {
      throw std::runtime_error(
        "workspace min bounds must be smaller than max bounds");
    }

    publisher_ = create_publisher<CloudMsg>(
      output_topic_, rclcpp::SensorDataQoS());

    if (publish_debug_clouds_) {
      debug_publisher_1_ = create_publisher<CloudMsg>(
        debug_topic_1_, rclcpp::SensorDataQoS());
      debug_publisher_2_ = create_publisher<CloudMsg>(
        debug_topic_2_, rclcpp::SensorDataQoS());
    }

    sub_1_ = std::make_shared<message_filters::Subscriber<CloudMsg>>(
      this, topic_1_, rmw_qos_profile_sensor_data);
    sub_2_ = std::make_shared<message_filters::Subscriber<CloudMsg>>(
      this, topic_2_, rmw_qos_profile_sensor_data);

    ApproximatePolicy policy(
      static_cast<uint32_t>(sync_queue_size_));
    policy.setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(sync_slop_));

    sync_ = std::make_shared<
      message_filters::Synchronizer<ApproximatePolicy>>(
        static_cast<const ApproximatePolicy &>(policy), *sub_1_, *sub_2_);

    sync_->registerCallback(
      std::bind(
        &MultiCameraCloudFusion::cloudCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Dual-camera fusion: '%s' + '%s' -> '%s', target_frame='%s', "
      "voxel=%.3f m, sync_slop=%.3f s",
      topic_1_.c_str(),
      topic_2_.c_str(),
      output_topic_.c_str(),
      target_frame_.c_str(),
      voxel_leaf_size_,
      sync_slop_);
  }

private:
  bool transformCloudToTarget(
    const CloudMsg & msg,
    CloudPtr & output)
  {
    if (msg.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received PointCloud2 with empty frame_id");
      return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        target_frame_,
        msg.header.frame_id,
        rclcpp::Time(msg.header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s <- %s failed: %s",
        target_frame_.c_str(),
        msg.header.frame_id.c_str(),
        ex.what());
      return false;
    }

    auto input = std::make_shared<Cloud>();
    pcl::fromROSMsg(msg, *input);

    auto clean = std::make_shared<Cloud>();
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(
      *input, *clean, valid_indices);

    if (clean->empty()) {
      return false;
    }

    const auto & t = transform.transform.translation;
    const auto & q_msg = transform.transform.rotation;

    Eigen::Quaternionf q(
      static_cast<float>(q_msg.w),
      static_cast<float>(q_msg.x),
      static_cast<float>(q_msg.y),
      static_cast<float>(q_msg.z));

    if (q.norm() < 1e-6f) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Invalid quaternion in TF %s <- %s",
        target_frame_.c_str(),
        msg.header.frame_id.c_str());
      return false;
    }
    q.normalize();

    Eigen::Affine3f eigen_tf =
      Eigen::Affine3f::Identity();
    eigen_tf.translation() <<
      static_cast<float>(t.x),
      static_cast<float>(t.y),
      static_cast<float>(t.z);
    eigen_tf.linear() = q.toRotationMatrix();

    output = std::make_shared<Cloud>();
    pcl::transformPointCloud(
      *clean, *output, eigen_tf);
    return !output->empty();
  }

  CloudPtr cropWorkspace(
    const CloudPtr & input) const
  {
    if (!crop_to_workspace_) {
      return input;
    }

    auto crop_axis =
      [](const CloudPtr & cloud,
         const std::string & field,
         const double min_value,
         const double max_value) -> CloudPtr
      {
        auto output =
          std::make_shared<Cloud>();

        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(field);
        pass.setFilterLimits(
          static_cast<float>(min_value),
          static_cast<float>(max_value));
        pass.filter(*output);

        return output;
      };

    CloudPtr cropped =
      crop_axis(
        input, "x",
        workspace_min_x_, workspace_max_x_);
    cropped =
      crop_axis(
        cropped, "y",
        workspace_min_y_, workspace_max_y_);
    cropped =
      crop_axis(
        cropped, "z",
        workspace_min_z_, workspace_max_z_);

    return cropped;
  }

  void publishDebugCloud(
    const CloudPtr & cloud,
    const std_msgs::msg::Header & source_header,
    const rclcpp::Publisher<CloudMsg>::SharedPtr & publisher)
  {
    if (!publisher || !cloud) {
      return;
    }

    CloudMsg msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = source_header;
    msg.header.frame_id = target_frame_;
    publisher->publish(msg);
  }

  void cloudCallback(
    const CloudMsg::ConstSharedPtr & cloud_1_msg,
    const CloudMsg::ConstSharedPtr & cloud_2_msg)
  {
    CloudPtr cloud_1;
    CloudPtr cloud_2;

    if (!transformCloudToTarget(*cloud_1_msg, cloud_1) ||
        !transformCloudToTarget(*cloud_2_msg, cloud_2))
    {
      return;
    }

    if (publish_debug_clouds_) {
      publishDebugCloud(
        cloud_1,
        cloud_1_msg->header,
        debug_publisher_1_);
      publishDebugCloud(
        cloud_2,
        cloud_2_msg->header,
        debug_publisher_2_);
    }

    auto merged =
      std::make_shared<Cloud>();
    merged->points.reserve(
      cloud_1->size() + cloud_2->size());
    *merged += *cloud_1;
    *merged += *cloud_2;

    merged = cropWorkspace(merged);
    if (!merged || merged->empty()) {
      return;
    }

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(merged);

    const float leaf =
      static_cast<float>(voxel_leaf_size_);
    voxel.setLeafSize(
      leaf, leaf, leaf);

    auto filtered =
      std::make_shared<Cloud>();
    voxel.filter(*filtered);

    if (filtered->empty()) {
      return;
    }

    CloudMsg output_msg;
    pcl::toROSMsg(
      *filtered, output_msg);
    output_msg.header.frame_id =
      target_frame_;

    const rclcpp::Time stamp_1(
      cloud_1_msg->header.stamp);
    const rclcpp::Time stamp_2(
      cloud_2_msg->header.stamp);

    output_msg.header.stamp =
      (stamp_1 >= stamp_2)
      ? cloud_1_msg->header.stamp
      : cloud_2_msg->header.stamp;

    publisher_->publish(output_msg);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "camera1=%zu, camera2=%zu, merged_voxel=%zu",
      cloud_1->size(),
      cloud_2->size(),
      filtered->size());
  }

  std::string target_frame_;
  std::string topic_1_;
  std::string topic_2_;
  std::string output_topic_;
  std::string debug_topic_1_;
  std::string debug_topic_2_;

  bool publish_debug_clouds_{true};
  bool crop_to_workspace_{true};

  double voxel_leaf_size_{0.003};
  double sync_slop_{0.08};
  double tf_timeout_{0.20};

  int sync_queue_size_{10};

  double workspace_min_x_{-1.10};
  double workspace_max_x_{1.10};
  double workspace_min_y_{0.25};
  double workspace_max_y_{1.15};
  double workspace_min_z_{0.76};
  double workspace_max_z_{1.45};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<
    message_filters::Subscriber<CloudMsg>>
    sub_1_;
  std::shared_ptr<
    message_filters::Subscriber<CloudMsg>>
    sub_2_;
  std::shared_ptr<
    message_filters::Synchronizer<ApproximatePolicy>>
    sync_;

  rclcpp::Publisher<CloudMsg>::SharedPtr
    publisher_;
  rclcpp::Publisher<CloudMsg>::SharedPtr
    debug_publisher_1_;
  rclcpp::Publisher<CloudMsg>::SharedPtr
    debug_publisher_2_;
};

int main(
  int argc,
  char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      MultiCameraCloudFusion>());
  rclcpp::shutdown();
  return 0;
}
