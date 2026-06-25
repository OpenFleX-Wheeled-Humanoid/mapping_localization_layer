#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

using PointType = pcl::PointXYZI;
using PointTypeRGB = pcl::PointXYZRGB;
using PointTypeNormal = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using CloudTypeRGB = pcl::PointCloud<PointTypeRGB>;
using CloudTypeNormal = pcl::PointCloud<PointTypeNormal>;
using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;

namespace
{
double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time secToStamp(const double sec)
{
  builtin_interfaces::msg::Time stamp;
  if (sec <= 0.0) {
    stamp.sec = 0;
    stamp.nanosec = 0;
    return stamp;
  }

  const auto sec_floor = static_cast<int32_t>(std::floor(sec));
  const auto nsec = static_cast<uint32_t>(std::round((sec - static_cast<double>(sec_floor)) * 1e9));
  stamp.sec = sec_floor;
  stamp.nanosec = nsec >= 1000000000u ? 999999999u : nsec;
  return stamp;
}

geometry_msgs::msg::TransformStamped makeTransform(
    const std::string &frame_id,
    const std::string &child_frame_id,
    const builtin_interfaces::msg::Time &stamp,
    const M3D &rotation,
    const V3D &translation)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  transform.header.stamp = stamp;

  const Eigen::Quaterniond q(rotation);
  transform.transform.translation.x = translation.x();
  transform.transform.translation.y = translation.y();
  transform.transform.translation.z = translation.z();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  return transform;
}

std::array<double, 9> identityRotationArray()
{
  return {1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0};
}

M3D yawOnlyRotation(const M3D &rotation)
{
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
}
}  // namespace

struct PoseState
{
  M3D rotation{M3D::Identity()};
  V3D translation{V3D::Zero()};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool valid{false};
};

struct MotionEstimate
{
  V3D linear_body{V3D::Zero()};
  V3D angular_body{V3D::Zero()};
};

struct SelfFilterBox
{
  double min_x{-0.45};
  double max_x{0.45};
  double min_y{-0.35};
  double max_y{0.35};
  double min_z{-0.25};
  double max_z{0.35};
};

class LioBridgeNode : public rclcpp::Node
{
public:
  LioBridgeNode()
  : Node("lio_bridge_node")
  {
    loadParameters();

    const auto cloud_qos = rclcpp::SensorDataQoS();
    const auto odom_qos = rclcpp::QoS(50);
    const auto path_qos = rclcpp::QoS(10);

    raw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        raw_odom_topic_, odom_qos,
        std::bind(&LioBridgeNode::rawOdomCallback, this, std::placeholders::_1));
    raw_world_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        raw_world_cloud_topic_, cloud_qos,
        std::bind(&LioBridgeNode::rawWorldCloudCallback, this, std::placeholders::_1));

    if (publish_map_odom_) {
      wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          wheel_odom_topic_, odom_qos,
          std::bind(&LioBridgeNode::wheelOdomCallback, this, std::placeholders::_1));
      pgo_offset_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          pgo_offset_topic_, odom_qos,
          std::bind(&LioBridgeNode::pgoOffsetCallback, this, std::placeholders::_1));
      tf_timer_ = create_wall_timer(
          std::chrono::milliseconds(20),
          std::bind(&LioBridgeNode::tfTimerCallback, this));
    }

    body_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(body_cloud_topic_, cloud_qos);
    world_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(world_cloud_topic_, cloud_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(lio_odom_topic_, odom_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    path_.header.frame_id = world_frame_;
    path_.poses.clear();

    RCLCPP_INFO(
        get_logger(),
        "FAST-LIVO2 bridge started: raw_odom=%s raw_world_cloud=%s output_odom=%s body_cloud=%s world_cloud=%s mode=%s",
        raw_odom_topic_.c_str(),
        raw_world_cloud_topic_.c_str(),
        lio_odom_topic_.c_str(),
        body_cloud_topic_.c_str(),
        world_cloud_topic_.c_str(),
        publish_map_odom_ ? "mapping" : "navigation");
  }

private:
  void loadParameters()
  {
    declare_parameter("raw_odom_topic", "raw/odom");
    declare_parameter("raw_world_cloud_topic", "raw/world_cloud");
    declare_parameter("lio_odom_topic", "lio_odom");
    declare_parameter("body_cloud_topic", "body_cloud");
    declare_parameter("world_cloud_topic", "world_cloud");
    declare_parameter("path_topic", "lio_path");
    declare_parameter("wheel_odom_topic", "/odom");
    declare_parameter("pgo_offset_topic", "/pgo/offset");
    declare_parameter("world_frame", "odom");
    declare_parameter("odom_frame", "odom");
    declare_parameter("body_frame", "base_link");
    declare_parameter("publish_tf", false);
    declare_parameter("publish_map_odom", false);
    declare_parameter("planarize_output_pose", false);
    declare_parameter("freeze_map_odom_when_static", true);
    declare_parameter("static_linear_thresh", 0.01);
    declare_parameter("static_angular_thresh", 0.02);
    declare_parameter("static_hold_time", 0.2);
    declare_parameter("body_cloud_self_filter_enabled", true);
    declare_parameter("body_cloud_self_filter_min_x", -0.45);
    declare_parameter("body_cloud_self_filter_max_x", 0.45);
    declare_parameter("body_cloud_self_filter_min_y", -0.35);
    declare_parameter("body_cloud_self_filter_max_y", 0.35);
    declare_parameter("body_cloud_self_filter_min_z", -0.25);
    declare_parameter("body_cloud_self_filter_max_z", 0.35);
    declare_parameter("body_cloud_self_filter_boxes", std::vector<double>{});
    declare_parameter("r_ib", std::vector<double>(identityRotationArray().begin(), identityRotationArray().end()));
    declare_parameter("t_ib", std::vector<double>{0.0, 0.0, 0.0});

    raw_odom_topic_ = get_parameter("raw_odom_topic").as_string();
    raw_world_cloud_topic_ = get_parameter("raw_world_cloud_topic").as_string();
    lio_odom_topic_ = get_parameter("lio_odom_topic").as_string();
    body_cloud_topic_ = get_parameter("body_cloud_topic").as_string();
    world_cloud_topic_ = get_parameter("world_cloud_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    wheel_odom_topic_ = get_parameter("wheel_odom_topic").as_string();
    pgo_offset_topic_ = get_parameter("pgo_offset_topic").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    body_frame_ = get_parameter("body_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    publish_map_odom_ = get_parameter("publish_map_odom").as_bool();
    planarize_output_pose_ = get_parameter("planarize_output_pose").as_bool();
    freeze_map_odom_when_static_ = get_parameter("freeze_map_odom_when_static").as_bool();
    static_linear_thresh_ = get_parameter("static_linear_thresh").as_double();
    static_angular_thresh_ = get_parameter("static_angular_thresh").as_double();
    static_hold_time_ = get_parameter("static_hold_time").as_double();
    body_cloud_self_filter_enabled_ = get_parameter("body_cloud_self_filter_enabled").as_bool();
    body_cloud_self_filter_min_x_ = get_parameter("body_cloud_self_filter_min_x").as_double();
    body_cloud_self_filter_max_x_ = get_parameter("body_cloud_self_filter_max_x").as_double();
    body_cloud_self_filter_min_y_ = get_parameter("body_cloud_self_filter_min_y").as_double();
    body_cloud_self_filter_max_y_ = get_parameter("body_cloud_self_filter_max_y").as_double();
    body_cloud_self_filter_min_z_ = get_parameter("body_cloud_self_filter_min_z").as_double();
    body_cloud_self_filter_max_z_ = get_parameter("body_cloud_self_filter_max_z").as_double();
    const auto self_filter_boxes = get_parameter("body_cloud_self_filter_boxes").as_double_array();
    if (!self_filter_boxes.empty()) {
      if (self_filter_boxes.size() % 6 != 0) {
        throw std::runtime_error("body_cloud_self_filter_boxes must contain 6 values per box");
      }
      body_cloud_self_filter_boxes_.clear();
      for (size_t i = 0; i < self_filter_boxes.size(); i += 6) {
        SelfFilterBox box;
        box.min_x = self_filter_boxes[i + 0];
        box.max_x = self_filter_boxes[i + 1];
        box.min_y = self_filter_boxes[i + 2];
        box.max_y = self_filter_boxes[i + 3];
        box.min_z = self_filter_boxes[i + 4];
        box.max_z = self_filter_boxes[i + 5];
        if (box.min_x > box.max_x || box.min_y > box.max_y || box.min_z > box.max_z) {
          throw std::runtime_error("body_cloud_self_filter_boxes min values must be <= max values");
        }
        body_cloud_self_filter_boxes_.push_back(box);
      }
    } else {
      body_cloud_self_filter_boxes_.push_back({
          body_cloud_self_filter_min_x_,
          body_cloud_self_filter_max_x_,
          body_cloud_self_filter_min_y_,
          body_cloud_self_filter_max_y_,
          body_cloud_self_filter_min_z_,
          body_cloud_self_filter_max_z_});
    }

    const auto r_ib_vec = get_parameter("r_ib").as_double_array();
    const auto t_ib_vec = get_parameter("t_ib").as_double_array();
    if (r_ib_vec.size() != 9 || t_ib_vec.size() != 3) {
      throw std::runtime_error("r_ib must have 9 values and t_ib must have 3 values");
    }

    r_ib_ << r_ib_vec[0], r_ib_vec[1], r_ib_vec[2],
             r_ib_vec[3], r_ib_vec[4], r_ib_vec[5],
             r_ib_vec[6], r_ib_vec[7], r_ib_vec[8];
    t_ib_ << t_ib_vec[0], t_ib_vec[1], t_ib_vec[2];
    r_bi_ = r_ib_.transpose();
    t_bi_ = -(r_bi_ * t_ib_);

    RCLCPP_INFO(
        get_logger(), "Body cloud self filter: enabled=%s, boxes=%zu",
        body_cloud_self_filter_enabled_ ? "true" : "false",
        body_cloud_self_filter_boxes_.size());
  }

  PoseState poseFromOdometry(const nav_msgs::msg::Odometry &msg) const
  {
    PoseState pose;
    pose.translation << msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z;
    const Eigen::Quaterniond q(
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z);
    pose.rotation = q.normalized().toRotationMatrix();
    pose.stamp = rclcpp::Time(msg.header.stamp);
    pose.valid = true;
    return pose;
  }

  PoseState composePose(const PoseState &a, const M3D &rotation_b, const V3D &translation_b) const
  {
    PoseState composed = a;
    composed.rotation = a.rotation * rotation_b;
    composed.translation = a.rotation * translation_b + a.translation;
    return composed;
  }

  PoseState invertPose(const PoseState &pose) const
  {
    PoseState inverted = pose;
    inverted.rotation = pose.rotation.transpose();
    inverted.translation = -(inverted.rotation * pose.translation);
    return inverted;
  }

  PoseState applyPgoOffset(const PoseState &raw_pose)
  {
    PoseState corrected = raw_pose;
    std::lock_guard<std::mutex> lock(pgo_mutex_);
    corrected.rotation = pgo_offset_rotation_ * raw_pose.rotation;
    corrected.translation = pgo_offset_rotation_ * raw_pose.translation + pgo_offset_translation_;
    return corrected;
  }

  PoseState planarizePose(const PoseState &pose) const
  {
    PoseState planarized = pose;
    planarized.rotation = yawOnlyRotation(pose.rotation);
    return planarized;
  }

  MotionEstimate estimateMotion(const PoseState &current)
  {
    MotionEstimate motion;
    if (!previous_processed_pose_.valid) {
      previous_processed_pose_ = current;
      return motion;
    }

    const double dt = (current.stamp - previous_processed_pose_.stamp).seconds();
    if (dt <= 1e-3) {
      previous_processed_pose_ = current;
      return motion;
    }

    const V3D delta_world = current.translation - previous_processed_pose_.translation;
    motion.linear_body = current.rotation.transpose() * (delta_world / dt);

    const M3D delta_rotation = previous_processed_pose_.rotation.transpose() * current.rotation;
    const Eigen::AngleAxisd angle_axis(delta_rotation);
    if (std::isfinite(angle_axis.angle())) {
      motion.angular_body = angle_axis.axis() * (angle_axis.angle() / dt);
    }

    previous_processed_pose_ = current;
    return motion;
  }

  void publishProcessedOdom(const PoseState &processed_pose)
  {
    MotionEstimate motion;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      motion = estimateMotion(processed_pose);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = body_frame_;
    odom.header.stamp = processed_pose.stamp;
    odom.pose.pose.position.x = processed_pose.translation.x();
    odom.pose.pose.position.y = processed_pose.translation.y();
    odom.pose.pose.position.z = processed_pose.translation.z();

    const Eigen::Quaterniond q(processed_pose.rotation);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = motion.linear_body.x();
    odom.twist.twist.linear.y = motion.linear_body.y();
    odom.twist.twist.linear.z = motion.linear_body.z();
    odom.twist.twist.angular.x = motion.angular_body.x();
    odom.twist.twist.angular.y = motion.angular_body.y();
    odom.twist.twist.angular.z = motion.angular_body.z();

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.001;
    odom.pose.covariance[28] = 0.001;
    odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0] = 0.02;
    odom.twist.covariance[7] = 0.02;
    odom.twist.covariance[14] = 0.02;
    odom.twist.covariance[35] = 0.02;

    odom_pub_->publish(odom);
  }

  void publishProcessedPath(const PoseState &processed_pose)
  {
    if (path_pub_->get_subscription_count() == 0) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = world_frame_;
    pose.header.stamp = processed_pose.stamp;
    pose.pose.position.x = processed_pose.translation.x();
    pose.pose.position.y = processed_pose.translation.y();
    pose.pose.position.z = processed_pose.translation.z();

    if (publish_map_odom_) {
      if (!path_z_initialized_) {
        path_z_initialized_ = true;
        path_z_fixed_ = processed_pose.translation.z();
      }
      pose.pose.position.z = path_z_fixed_;
    }

    const Eigen::Quaterniond q(processed_pose.rotation);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_.header.frame_id = world_frame_;
    path_.header.stamp = processed_pose.stamp;
    path_.poses.push_back(pose);
    path_pub_->publish(path_);
  }

  void publishBodyAndWorldClouds(
      const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
      const PoseState &reference_pose,
      const PoseState &processed_pose,
      const PoseState &cloud_reference_pose)
  {
    // Skip empty clouds to avoid SIGFPE in PCL deserialization
    if (msg->width == 0 || msg->data.empty() || msg->point_step == 0) {
      return;
    }

    // Detect point format by checking field names.
    // FAST-LIVO2 publishes:
    //   - PointXYZINormal (has "normal_x" field) in LIO-only mode
    //   - PointXYZRGB (has "rgb" field) in LIVO mode
    bool is_rgb = false;
    bool is_normal = false;
    for (const auto &field : msg->fields) {
      if (field.name == "rgb") {
        is_rgb = true;
        break;
      }
      if (field.name == "normal_x") {
        is_normal = true;
      }
    }

    CloudType::Ptr cloud(new CloudType);
    if (is_rgb) {
      CloudTypeRGB::Ptr cloud_rgb(new CloudTypeRGB);
      pcl::fromROSMsg(*msg, *cloud_rgb);
      cloud->resize(cloud_rgb->size());
      for (size_t i = 0; i < cloud_rgb->size(); ++i) {
        auto &src = cloud_rgb->points[i];
        auto &dst = cloud->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = 0.299f * src.r + 0.587f * src.g + 0.114f * src.b;
      }
      cloud->width = cloud_rgb->width;
      cloud->height = cloud_rgb->height;
      cloud->is_dense = cloud_rgb->is_dense;
    } else if (is_normal) {
      CloudTypeNormal::Ptr cloud_normal(new CloudTypeNormal);
      pcl::fromROSMsg(*msg, *cloud_normal);
      cloud->resize(cloud_normal->size());
      for (size_t i = 0; i < cloud_normal->size(); ++i) {
        auto &src = cloud_normal->points[i];
        auto &dst = cloud->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
      }
      cloud->width = cloud_normal->width;
      cloud->height = cloud_normal->height;
      cloud->is_dense = cloud_normal->is_dense;
    } else {
      pcl::fromROSMsg(*msg, *cloud);
    }

    CloudType::Ptr body_cloud(new CloudType);
    const PoseState body_to_world = invertPose(processed_pose);
    Eigen::Matrix4f world_to_body = Eigen::Matrix4f::Identity();
    world_to_body.block<3, 3>(0, 0) = body_to_world.rotation.cast<float>();
    world_to_body.block<3, 1>(0, 3) = body_to_world.translation.cast<float>();
    pcl::transformPointCloud(*cloud, *body_cloud, world_to_body);
    body_cloud = filterBodyCloud(body_cloud);
    publishCloud(body_cloud_pub_, body_cloud, body_frame_, msg->header.stamp);

    CloudType::Ptr world_cloud(new CloudType);
    if (publish_map_odom_) {
      const Eigen::Matrix4f correction = buildWorldCorrection(reference_pose, cloud_reference_pose);
      pcl::transformPointCloud(*cloud, *world_cloud, correction);
    } else {
      *world_cloud = *cloud;
    }
    publishCloud(world_cloud_pub_, world_cloud, world_frame_, msg->header.stamp);
  }

  CloudType::Ptr filterBodyCloud(const CloudType::Ptr &cloud) const
  {
    if (!body_cloud_self_filter_enabled_ || !cloud) {
      return cloud;
    }

    CloudType::Ptr filtered(new CloudType);
    filtered->reserve(cloud->size());
    for (const auto &point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      bool in_box = false;
      for (const auto &box : body_cloud_self_filter_boxes_) {
        if (point.x >= box.min_x && point.x <= box.max_x &&
            point.y >= box.min_y && point.y <= box.max_y &&
            point.z >= box.min_z && point.z <= box.max_z) {
          in_box = true;
          break;
        }
      }
      if (!in_box) {
        filtered->push_back(point);
      }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;
    return filtered;
  }

  Eigen::Matrix4f buildWorldCorrection(
      const PoseState &raw_pose,
      const PoseState &reference_pose)
  {
    PoseState corrected_world_pose = reference_pose;
    bool have_refined_pose = false;

    {
      std::lock_guard<std::mutex> lock(map_odom_mutex_);
      if (map_to_odom_valid_ && latest_wheel_pose_.valid) {
        corrected_world_pose.rotation = map_to_odom_rotation_ * latest_wheel_pose_.rotation;
        corrected_world_pose.translation =
            map_to_odom_rotation_ * latest_wheel_pose_.translation + map_to_odom_translation_;
        corrected_world_pose = composePose(corrected_world_pose, r_bi_, t_bi_);
        have_refined_pose = true;
      }
    }

    if (!have_refined_pose) {
      corrected_world_pose = applyPgoOffset(reference_pose);
    }

    const M3D delta_rotation = corrected_world_pose.rotation * raw_pose.rotation.transpose();
    const V3D delta_translation = corrected_world_pose.translation - delta_rotation * raw_pose.translation;

    Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
    correction.block<3, 3>(0, 0) = delta_rotation.cast<float>();
    correction.block<3, 1>(0, 3) = delta_translation.cast<float>();
    return correction;
  }

  void publishCloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
      const CloudType::Ptr &cloud,
      const std::string &frame_id,
      const builtin_interfaces::msg::Time &stamp)
  {
    if (!cloud || publisher->get_subscription_count() == 0) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = stamp;
    publisher->publish(cloud_msg);
  }

  void rawOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    PoseState raw_pose = poseFromOdometry(*msg);
    PoseState processed_pose = composePose(raw_pose, r_ib_, t_ib_);
    if (planarize_output_pose_) {
      processed_pose = planarizePose(processed_pose);
    }
    const PoseState cloud_reference_pose = composePose(processed_pose, r_bi_, t_bi_);

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      latest_raw_pose_ = raw_pose;
      latest_processed_pose_ = processed_pose;
      latest_cloud_reference_pose_ = cloud_reference_pose;
    }

    publishProcessedOdom(processed_pose);
    publishProcessedPath(processed_pose);

    if (!publish_map_odom_ && publish_tf_) {
      tf_broadcaster_->sendTransform(
          makeTransform(world_frame_, body_frame_, msg->header.stamp, processed_pose.rotation, processed_pose.translation));
    }

    if (publish_map_odom_) {
      const bool freeze_map_odom = shouldFreezeMapToOdom();
      if (!freeze_map_odom) {
        updateMapToOdom(processed_pose);
      }
    }
  }

  void rawWorldCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PoseState raw_pose;
    PoseState processed_pose;
    PoseState cloud_reference_pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      raw_pose = latest_raw_pose_;
      processed_pose = latest_processed_pose_;
      cloud_reference_pose = latest_cloud_reference_pose_;
    }

    if (!raw_pose.valid || !processed_pose.valid || !cloud_reference_pose.valid) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping raw world cloud because pose is not ready yet");
      return;
    }

    publishBodyAndWorldClouds(msg, raw_pose, processed_pose, cloud_reference_pose);
  }

  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_wheel_pose_ = poseFromOdometry(*msg);

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double wz = msg->twist.twist.angular.z;
    const double linear_speed = std::hypot(vx, vy);
    const double angular_speed = std::abs(wz);
    const double now_sec = get_clock()->now().seconds();
    const bool low_speed =
        linear_speed < static_linear_thresh_ &&
        angular_speed < static_angular_thresh_;

    std::lock_guard<std::mutex> lock(motion_mutex_);
    last_motion_msg_time_ = now_sec;
    if (low_speed) {
      if (stationary_since_ < 0.0) {
        stationary_since_ = now_sec;
      }
      robot_is_stationary_ = (now_sec - stationary_since_) >= static_hold_time_;
    } else {
      stationary_since_ = -1.0;
      robot_is_stationary_ = false;
    }
  }

  void pgoOffsetCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pgo_mutex_);
    const Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    pgo_offset_rotation_ = q.normalized().toRotationMatrix();
    pgo_offset_translation_ << msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z;
    pgo_offset_dirty_ = true;
  }

  bool shouldFreezeMapToOdom()
  {
    if (!freeze_map_odom_when_static_) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(map_odom_mutex_);
      if (!map_to_odom_valid_) {
        return false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(pgo_mutex_);
      if (pgo_offset_dirty_) {
        return false;
      }
    }

    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (last_motion_msg_time_ < 0.0) {
      return false;
    }

    const double now_sec = get_clock()->now().seconds();
    if ((now_sec - last_motion_msg_time_) > 0.5) {
      return false;
    }
    return robot_is_stationary_;
  }

  void updateMapToOdom(const PoseState &processed_pose)
  {
    if (!latest_wheel_pose_.valid) {
      return;
    }

    PoseState corrected_map_pose = applyPgoOffset(processed_pose);

    const PoseState wheel_inverse = invertPose(latest_wheel_pose_);
    const M3D map_to_odom_rotation = corrected_map_pose.rotation * wheel_inverse.rotation;
    const V3D map_to_odom_translation =
        corrected_map_pose.rotation * wheel_inverse.translation + corrected_map_pose.translation;

    Eigen::Quaterniond q_map_to_odom(map_to_odom_rotation);
    const M3D planar_rotation = yawOnlyRotation(map_to_odom_rotation);

    std::lock_guard<std::mutex> lock(map_odom_mutex_);
    if (!map_odom_z_initialized_) {
      map_odom_z_initialized_ = true;
      map_odom_z_fixed_ = map_to_odom_translation.z();
    }

    map_to_odom_rotation_ = planar_rotation;
    map_to_odom_translation_ = map_to_odom_translation;
    map_to_odom_translation_.z() = map_odom_z_fixed_;
    map_to_odom_stamp_ = processed_pose.stamp;
    map_to_odom_valid_ = true;

    {
      std::lock_guard<std::mutex> pgo_lock(pgo_mutex_);
      pgo_offset_dirty_ = false;
    }

    (void)q_map_to_odom;
  }

  void tfTimerCallback()
  {
    std::lock_guard<std::mutex> lock(map_odom_mutex_);
    if (!map_to_odom_valid_) {
      return;
    }

    const auto stamp = secToStamp(get_clock()->now().seconds());
    tf_broadcaster_->sendTransform(
        makeTransform(world_frame_, odom_frame_, stamp, map_to_odom_rotation_, map_to_odom_translation_));
  }

  std::string raw_odom_topic_;
  std::string raw_world_cloud_topic_;
  std::string lio_odom_topic_;
  std::string body_cloud_topic_;
  std::string world_cloud_topic_;
  std::string path_topic_;
  std::string wheel_odom_topic_;
  std::string pgo_offset_topic_;
  std::string world_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  bool publish_tf_{false};
  bool publish_map_odom_{false};
  bool planarize_output_pose_{false};
  bool freeze_map_odom_when_static_{true};
  double static_linear_thresh_{0.01};
  double static_angular_thresh_{0.02};
  double static_hold_time_{0.2};
  bool body_cloud_self_filter_enabled_{true};
  double body_cloud_self_filter_min_x_{-0.45};
  double body_cloud_self_filter_max_x_{0.45};
  double body_cloud_self_filter_min_y_{-0.35};
  double body_cloud_self_filter_max_y_{0.35};
  double body_cloud_self_filter_min_z_{-0.25};
  double body_cloud_self_filter_max_z_{0.35};
  std::vector<SelfFilterBox> body_cloud_self_filter_boxes_;
  M3D r_ib_{M3D::Identity()};
  V3D t_ib_{V3D::Zero()};
  M3D r_bi_{M3D::Identity()};
  V3D t_bi_{V3D::Zero()};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_world_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pgo_offset_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::mutex pose_mutex_;
  PoseState latest_raw_pose_;
  PoseState latest_processed_pose_;
  PoseState latest_cloud_reference_pose_;
  PoseState previous_processed_pose_;

  std::mutex pgo_mutex_;
  M3D pgo_offset_rotation_{M3D::Identity()};
  V3D pgo_offset_translation_{V3D::Zero()};
  bool pgo_offset_dirty_{false};

  std::mutex map_odom_mutex_;
  M3D map_to_odom_rotation_{M3D::Identity()};
  V3D map_to_odom_translation_{V3D::Zero()};
  rclcpp::Time map_to_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool map_to_odom_valid_{false};
  bool map_odom_z_initialized_{false};
  double map_odom_z_fixed_{0.0};

  std::mutex motion_mutex_;
  double last_motion_msg_time_{-1.0};
  double stationary_since_{-1.0};
  bool robot_is_stationary_{false};

  PoseState latest_wheel_pose_;
  nav_msgs::msg::Path path_;
  bool path_z_initialized_{false};
  double path_z_fixed_{0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
