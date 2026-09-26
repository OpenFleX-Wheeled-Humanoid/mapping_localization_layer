#include "icp_registration/icp_registration.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/create_timer_ros.h>

namespace icp {

namespace {

constexpr std::size_t kMinAlignmentPoints = 15;

void normalizePoint(pcl::PointXYZI &) {}

bool isFinitePoint(const pcl::PointXYZI &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr sanitizeCloud(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    std::size_t *removed_count = nullptr) {
  typename pcl::PointCloud<PointT>::Ptr filtered(
      new pcl::PointCloud<PointT>);
  filtered->header = cloud->header;
  filtered->reserve(cloud->size());

  std::size_t removed = 0;
  for (const auto &point : cloud->points) {
    PointT normalized_point = point;
    normalizePoint(normalized_point);
    if (isFinitePoint(normalized_point)) {
      filtered->push_back(normalized_point);
    } else {
      ++removed;
    }
  }

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = true;
  if (removed_count != nullptr) {
    *removed_count = removed;
  }
  return filtered;
}

}  // namespace

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(30), refine_iter_(20),
      sc_loaded_(false), submap_half_range_(5),
      ndt_resolution_(1.0), ndt_step_size_(0.1),
      ndt_max_iterations_(30), ndt_epsilon_(0.01), ndt_score_thresh_(1.0),
      first_scan_(true), auto_realign_interval_sec_(1.0),
      initial_alignment_delay_sec_(0.0),
      min_points_for_alignment_(200),
      alignment_in_progress_(false) {
  is_ready_ = false;
  tf_smoothing_active_ = false;
  initial_localization_done_ = false;
  last_auto_realign_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  last_continuous_realign_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  node_start_time_ = this->now();
  cloud_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  cloud_in_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }
  // Read the pcd file
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  reader.read(pcd_path_, *cloud);
  std::size_t removed_points = 0;
  cloud = sanitizeCloud<pcl::PointXYZI>(cloud, &removed_points);
  if (removed_points > 0) {
    RCLCPP_WARN(this->get_logger(),
                "Dropped %zu non-finite map points from %s",
                removed_points, pcd_path_.c_str());
  }
  if (cloud->empty()) {
    throw std::runtime_error("PCD map became empty after filtering invalid points");
  }
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);
  cloud = sanitizeCloud<pcl::PointXYZI>(cloud, &removed_points);
  if (cloud->empty()) {
    throw std::runtime_error(
        "PCD map became empty after voxel filtering invalid points");
  }

  refine_map_ = cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*refine_map_, *point_rough);
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  rough_map_ = sanitizeCloud<pcl::PointXYZI>(filterd_point_rough);

  rough_iter_ = this->declare_parameter("rough_iter", 30);
  refine_iter_ = this->declare_parameter("refine_iter", 20);

  icp_rough_.setMaximumIterations(rough_iter_);

  icp_refine_.setMaximumIterations(refine_iter_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // Parameters
  map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  base_frame_id_ =
      this->declare_parameter("base_frame_id", std::string("base_link"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("laser"));
  thresh_ = this->declare_parameter("thresh", 0.15);
  continuous_thresh_ =
      this->declare_parameter("continuous_thresh", thresh_);
  xy_offset_ = this->declare_parameter("xy_offset", 0.2);
  yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
  if (yaw_resolution_ <= 0.0) {
    throw std::runtime_error(
        "yaw_resolution must be > 0 degrees, got " +
        std::to_string(yaw_resolution_ * 180.0 / M_PI));
  }
  auto_realign_interval_sec_ =
      this->declare_parameter("auto_realign_interval_sec", 1.0);
  initial_alignment_delay_sec_ =
      this->declare_parameter("initial_alignment_delay_sec", 0.0);
  continuous_realign_interval_sec_ =
      this->declare_parameter("continuous_realign_interval_sec", 0.0);
  min_points_for_alignment_ = static_cast<std::size_t>(
      this->declare_parameter("min_points_for_alignment", 200));
  map_crop_radius_ = this->declare_parameter("map_crop_radius", 50.0);
  tf_smooth_duration_ = this->declare_parameter("tf_smooth_duration", 0.5);
  pose_filter_alpha_ = this->declare_parameter("pose_filter_alpha", 0.35);
  pose_filter_ = PlanarPoseFilter(pose_filter_alpha_);
  constrain_to_2d_ = this->declare_parameter("constrain_to_2d", true);
  tf_smooth_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  initial_pose_.orientation.w = 1.0;
  try {
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
             initial_pose_vec.at(5));
    initial_pose_.orientation = tf2::toMsg(q);
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }
  initial_pose_ = projectPoseToPlane(initial_pose_);

  if (constrain_to_2d_) {
    RCLCPP_INFO(this->get_logger(),
                "ICP map->odom is constrained to planar x/y/yaw motion");
  }

  // NDT parameters
  ndt_resolution_ = this->declare_parameter("ndt_resolution", 1.0);
  ndt_step_size_ = this->declare_parameter("ndt_step_size", 0.1);
  ndt_max_iterations_ = this->declare_parameter("ndt_max_iterations", 30);
  ndt_epsilon_ = this->declare_parameter("ndt_epsilon", 0.01);
  ndt_score_thresh_ = this->declare_parameter("ndt_score_thresh", 1.0);
  submap_half_range_ = this->declare_parameter("submap_half_range", 5);

  // SC body frame: the frame used during mapping (mid360_link by default)
  sc_body_frame_ = this->declare_parameter("sc_body_frame", std::string("mid360_link"));

  // ========== 连续 ICP 严格验证参数 ==========
  continuous_max_correction_xy_ =
      this->declare_parameter("continuous_max_correction_xy", 0.3);
  continuous_max_correction_yaw_ =
      this->declare_parameter("continuous_max_correction_yaw", 0.26);
  continuous_min_inlier_points_ = static_cast<size_t>(
      this->declare_parameter("continuous_min_inlier_points", 100));
  continuous_max_rmse_ =
      this->declare_parameter("continuous_max_rmse", 0.15);
  continuous_min_fitness_score_ =
      this->declare_parameter("continuous_min_fitness_score", 0.95);
  continuous_consistency_window_ =
      this->declare_parameter("continuous_consistency_window", 3);
  continuous_consistency_tolerance_ =
      this->declare_parameter("continuous_consistency_tolerance", 0.1);
  continuous_tf_lookup_strict_ =
      this->declare_parameter("continuous_tf_lookup_strict", true);
  continuous_tf_max_extrapolation_ =
      this->declare_parameter("continuous_tf_max_extrapolation", 0.1);
  continuous_max_consecutive_rejects_ =
      this->declare_parameter("continuous_max_consecutive_rejects", 5);

  RCLCPP_INFO(this->get_logger(),
      "Continuous ICP validation: max_xy=%.2fm, max_yaw=%.1f°, "
      "min_points=%zu, max_rmse=%.3fm, consistency_window=%d",
      continuous_max_correction_xy_,
      continuous_max_correction_yaw_ * 180.0 / M_PI,
      continuous_min_inlier_points_,
      continuous_max_rmse_,
      continuous_consistency_window_);

  // Initialize NDT
  ndt_.setResolution(ndt_resolution_);
  ndt_.setStepSize(ndt_step_size_);
  ndt_.setMaximumIterations(ndt_max_iterations_);
  ndt_.setTransformationEpsilon(ndt_epsilon_);

  RCLCPP_INFO(this->get_logger(),
              "ICP config: thresh=%.2f, continuous_thresh=%.2f, xy_offset=%.1f, rough_iter=%d, refine_iter=%d, initial_delay=%.1fs",
              thresh_, continuous_thresh_, xy_offset_, rough_iter_, refine_iter_,
              initial_alignment_delay_sec_);

  // Derive map_dir from pcd_path and try to load SC database
  map_dir_ = pcd_path_.parent_path();
  double sc_dist_thresh = this->declare_parameter("sc_dist_thresh", 0.2);
  sc_manager_.setSCdistThres(sc_dist_thresh);
  if (loadScanContextDatabase() && loadKeyframePoses()) {
    RCLCPP_INFO(this->get_logger(),
                "ScanContext database loaded: %zu keyframes, SC relocalization enabled (sc_dist_thresh=%.2f)",
                keyframe_poses_.size(), sc_dist_thresh);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "ScanContext database not available, using grid-search ICP only");
  }

  // GPS-assisted SC relocalization
  gps_filter_radius_ = this->declare_parameter("gps_filter_radius", 20.0);
  auto gps_qos = rclcpp::QoS(1);
  gps_qos.reliable();
  gps_qos.transient_local();
  gps_map_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/gps/map_position", gps_qos,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(gps_mutex_);
        gps_map_x_ = msg->point.x;
        gps_map_y_ = msg->point.y;
        has_gps_position_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "GPS map position received: x=%.2f, y=%.2f",
                         gps_map_x_, gps_map_y_);
      });

  // LIO yaw jump detection parameters
  lio_jump_divergence_thresh_ = this->declare_parameter("lio_jump_divergence_thresh", 0.5);
  lio_jump_icp_score_factor_ = this->declare_parameter("lio_jump_icp_score_factor", 0.1);
  lio_jump_detection_timeout_ = this->declare_parameter("lio_jump_detection_timeout", 10.0);
  icp_wheel_abs_yaw_thresh_ = this->declare_parameter("icp_wheel_abs_yaw_thresh", 1.5);
  icp_wheel_recovery_yaw_gate_thresh_ =
      this->declare_parameter("icp_wheel_recovery_yaw_gate_thresh", 0.52);
  wheel_odom_pose_timeout_ = this->declare_parameter("wheel_odom_pose_timeout", 0.5);
  lio_jump_detect_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  std::string localization_hold_topic =
      this->declare_parameter("localization_hold_topic", std::string("/localization_hold"));
  localization_hold_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      localization_hold_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // Wheel odometry subscription for LIO yaw jump detection
  std::string wheel_odom_topic = this->declare_parameter("wheel_odom_topic", std::string("/odom"));
  auto wheel_odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic, wheel_odom_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_odom_yaw_ = yaw;
        wheel_odom_stamp_ = msg->header.stamp;
        wheel_odom_valid_ = true;
      });
  RCLCPP_INFO(this->get_logger(),
              "LIO yaw jump detection: wheel_odom=%s, divergence_thresh=%.2frad, score_factor=%.2f",
              wheel_odom_topic.c_str(), lio_jump_divergence_thresh_, lio_jump_icp_score_factor_);

  // Set up the pointcloud subscriber
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::SensorDataQoS();
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));

  // Timer-based continuous realignment: decouples ICP frequency from
  // pointcloud arrival rate.  Fires at the configured interval and uses
  // the latest cached cloud.
  if (continuous_realign_interval_sec_ > 0.0) {
    continuous_realign_timer_ = create_wall_timer(
        std::chrono::duration<double>(continuous_realign_interval_sec_),
        std::bind(&IcpNode::continuousRealignTimerCallback, this));
    RCLCPP_INFO(this->get_logger(),
                "Continuous realign timer created: interval=%.1fs",
                continuous_realign_interval_sec_);
  }

  // Set up the initial pose subscriber
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            bool expected = false;
            if (!alignment_in_progress_.compare_exchange_strong(expected, true)) {
              RCLCPP_WARN(this->get_logger(),
                          "Alignment already in progress, ignoring /initialpose");
              return;
            }
            std::lock_guard<std::mutex> lock(align_thread_mutex_);
            if (align_thread_.joinable()) {
              align_thread_.join();
            }
            align_thread_ = std::thread([this, msg]() {
              try {
                initialPoseCallback(msg);
              } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                             "initialPoseCallback exception: %s", e.what());
              }
              alignment_in_progress_.store(false);
            });
          });

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok() && !stop_tf_thread_.load()) {
      {
        std::lock_guard lock(mutex_);
        if (is_ready_) {
          geometry_msgs::msg::TransformStamped tf_to_publish;

          if (tf_smoothing_active_ && tf_smooth_duration_ > 0.0) {
            double elapsed = (now() - tf_smooth_start_time_).seconds();
            double alpha = std::clamp(elapsed / tf_smooth_duration_, 0.0, 1.0);

            // LERP translation
            auto &cur_t = map_to_odom_current_.transform.translation;
            auto &tgt_t = map_to_odom_target_.transform.translation;
            tf_to_publish.transform.translation.x = cur_t.x + alpha * (tgt_t.x - cur_t.x);
            tf_to_publish.transform.translation.y = cur_t.y + alpha * (tgt_t.y - cur_t.y);
            tf_to_publish.transform.translation.z = cur_t.z + alpha * (tgt_t.z - cur_t.z);

            // SLERP rotation
            tf2::Quaternion q_cur, q_tgt;
            tf2::fromMsg(map_to_odom_current_.transform.rotation, q_cur);
            tf2::fromMsg(map_to_odom_target_.transform.rotation, q_tgt);
            tf2::Quaternion q_interp = q_cur.slerp(q_tgt, alpha);
            tf_to_publish.transform.rotation = tf2::toMsg(q_interp);

            if (alpha >= 1.0) {
              tf_smoothing_active_ = false;
              map_to_odom_.transform = map_to_odom_target_.transform;
              RCLCPP_DEBUG(this->get_logger(), "TF smoothing complete");
            }
          } else {
            tf_to_publish.transform = map_to_odom_.transform;
          }

          tf_to_publish.header.stamp = now();
          tf_to_publish.header.frame_id = map_frame_id_;
          tf_to_publish.child_frame_id = odom_frame_id_;
          tf_broadcaster_->sendTransform(tf_to_publish);
        }
      }
      rate.sleep();
    }
  });

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {
  stop_tf_thread_.store(true);
  if (tf_publisher_thread_ && tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();
  }
  {
    std::lock_guard<std::mutex> lock(align_thread_mutex_);
    if (align_thread_.joinable()) {
      align_thread_.join();
    }
  }
}

Eigen::Matrix4d IcpNode::projectTransformToPlane(
    const Eigen::Matrix4d &transform) const {
  if (!constrain_to_2d_) {
    return transform;
  }

  Eigen::Matrix4d planar = Eigen::Matrix4d::Identity();
  planar(0, 3) = transform(0, 3);
  planar(1, 3) = transform(1, 3);

  const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  planar.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return planar;
}

geometry_msgs::msg::Pose IcpNode::projectPoseToPlane(
    const geometry_msgs::msg::Pose &pose) const {
  if (!constrain_to_2d_) {
    return pose;
  }

  geometry_msgs::msg::Pose planar_pose = pose;
  planar_pose.position.z = 0.0;

  const Eigen::Quaterniond q(
      pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z);
  const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const Eigen::Quaterniond planar_q(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  planar_pose.orientation.w = planar_q.w();
  planar_pose.orientation.x = planar_q.x();
  planar_pose.orientation.y = planar_q.y();
  planar_pose.orientation.z = planar_q.z();
  return planar_pose;
}

void IcpNode::updateMapToOdom(const Eigen::Matrix4d &new_transform, bool force_snap) {
  std::lock_guard lock(mutex_);

  Eigen::Matrix4d constrained_transform = projectTransformToPlane(new_transform);

  const double measured_yaw = std::atan2(
      constrained_transform(1, 0), constrained_transform(0, 0));
  const PlanarPose measured_pose{
      constrained_transform(0, 3), constrained_transform(1, 3), measured_yaw};
  if (force_snap || !initial_localization_done_ || !pose_filter_.initialized()) {
    pose_filter_.reset(measured_pose);
  } else {
    const PlanarPose filtered_pose = pose_filter_.update(measured_pose);
    constrained_transform(0, 3) = filtered_pose.x;
    constrained_transform(1, 3) = filtered_pose.y;
    constrained_transform.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(filtered_pose.yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  }

  // Build the new transform message
  Eigen::Quaterniond q_new(constrained_transform.block<3, 3>(0, 0));

  if (force_snap || !initial_localization_done_ || tf_smooth_duration_ <= 0.0) {
    // Snap immediately: first localization, SC reloc, or smoothing disabled
    map_to_odom_.transform.translation.x = constrained_transform(0, 3);
    map_to_odom_.transform.translation.y = constrained_transform(1, 3);
    map_to_odom_.transform.translation.z = constrained_transform(2, 3);
    map_to_odom_.transform.rotation.w = q_new.w();
    map_to_odom_.transform.rotation.x = q_new.x();
    map_to_odom_.transform.rotation.y = q_new.y();
    map_to_odom_.transform.rotation.z = q_new.z();
    initial_localization_done_ = true;
    tf_smoothing_active_ = false;
  } else {
    // Start smooth transition
    if (tf_smoothing_active_) {
      // Already smoothing: compute current interpolated position as new start
      double elapsed = (now() - tf_smooth_start_time_).seconds();
      double alpha = std::clamp(elapsed / tf_smooth_duration_, 0.0, 1.0);

      auto &cur_t = map_to_odom_current_.transform.translation;
      auto &tgt_t = map_to_odom_target_.transform.translation;
      map_to_odom_current_.transform.translation.x = cur_t.x + alpha * (tgt_t.x - cur_t.x);
      map_to_odom_current_.transform.translation.y = cur_t.y + alpha * (tgt_t.y - cur_t.y);
      map_to_odom_current_.transform.translation.z = cur_t.z + alpha * (tgt_t.z - cur_t.z);

      tf2::Quaternion q_c, q_t;
      tf2::fromMsg(map_to_odom_current_.transform.rotation, q_c);
      tf2::fromMsg(map_to_odom_target_.transform.rotation, q_t);
      tf2::Quaternion q_interp = q_c.slerp(q_t, alpha);
      map_to_odom_current_.transform.rotation = tf2::toMsg(q_interp);
    } else {
      // Start fresh: current position is what we've been publishing
      map_to_odom_current_.transform = map_to_odom_.transform;
    }

    map_to_odom_target_.transform.translation.x = constrained_transform(0, 3);
    map_to_odom_target_.transform.translation.y = constrained_transform(1, 3);
    map_to_odom_target_.transform.translation.z = constrained_transform(2, 3);
    map_to_odom_target_.transform.rotation.w = q_new.w();
    map_to_odom_target_.transform.rotation.x = q_new.x();
    map_to_odom_target_.transform.rotation.y = q_new.y();
    map_to_odom_target_.transform.rotation.z = q_new.z();
    tf_smooth_start_time_ = now();
    tf_smoothing_active_ = true;

    RCLCPP_DEBUG(this->get_logger(),
        "TF smoothing: (%.3f,%.3f) -> (%.3f,%.3f) over %.2fs",
        map_to_odom_current_.transform.translation.x,
        map_to_odom_current_.transform.translation.y,
        map_to_odom_target_.transform.translation.x,
        map_to_odom_target_.transform.translation.y,
        tf_smooth_duration_);
  }

  is_ready_ = true;
}

PointCloudXYZI::Ptr IcpNode::cropMapByPosition(
    const PointCloudXYZI::Ptr &full_map,
    const Eigen::Vector3d &position, double radius) const {
  if (radius <= 0.0 || !full_map || full_map->empty()) {
    return full_map;
  }

  PointCloudXYZI::Ptr cropped(new PointCloudXYZI);
  pcl::CropBox<PointType> crop_filter;
  crop_filter.setInputCloud(full_map);

  Eigen::Vector4f min_pt(
      static_cast<float>(position.x() - radius),
      static_cast<float>(position.y() - radius),
      -std::numeric_limits<float>::max(), 1.0f);
  Eigen::Vector4f max_pt(
      static_cast<float>(position.x() + radius),
      static_cast<float>(position.y() + radius),
      std::numeric_limits<float>::max(), 1.0f);

  crop_filter.setMin(min_pt);
  crop_filter.setMax(max_pt);
  crop_filter.filter(*cropped);

  cropped->header = full_map->header;
  return cropped;
}

bool IcpNode::mapPoseFromTransform(const Eigen::Matrix4d &map_to_odom,
                                   const std::string &target_frame,
                                   const rclcpp::Time &stamp,
                                   Eigen::Matrix4d &map_to_target) {
  try {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          odom_frame_id_, target_frame, stamp,
          rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::ExtrapolationException &) {
      transform = tf_buffer_->lookupTransform(
          odom_frame_id_, target_frame, tf2::TimePointZero);
    }

    Eigen::Matrix4d odom_to_target = Eigen::Matrix4d::Identity();
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q(transform.transform.rotation.w,
                         transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z);
    odom_to_target.block<3, 3>(0, 0) = q.toRotationMatrix();
    odom_to_target.block<3, 1>(0, 3) = t;
    map_to_target = projectTransformToPlane(map_to_odom * odom_to_target);
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to compute map->%s seed pose: %s",
                target_frame.c_str(), ex.what());
    return false;
  }
}

void IcpNode::seedLastAcceptedBasePose(const Eigen::Matrix4d &map_to_odom,
                                       const rclcpp::Time &stamp,
                                       const std::string &source) {
  Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
  if (!mapPoseFromTransform(map_to_odom, base_frame_id_, stamp, map_to_base)) {
    last_accepted_base_valid_ = false;
    return;
  }

  last_accepted_base_x_ = map_to_base(0, 3);
  last_accepted_base_y_ = map_to_base(1, 3);
  last_accepted_base_yaw_ = std::atan2(map_to_base(1, 0), map_to_base(0, 0));
  last_accepted_base_stamp_ = now();
  last_accepted_base_valid_ = true;
  consecutive_reject_count_ = 0;
  icp_wheel_yaw_offset_valid_ = false;

  RCLCPP_INFO(this->get_logger(),
              "%s seeded map->%s pose: x=%.2f y=%.2f yaw=%.1f deg",
              source.c_str(), base_frame_id_.c_str(),
              last_accepted_base_x_, last_accepted_base_y_,
              last_accepted_base_yaw_ * 180.0 / M_PI);
}

bool IcpNode::loadScanContextDatabase() {
  std::filesystem::path sc_dir = map_dir_ / "sc_data";
  if (!std::filesystem::exists(sc_dir)) {
    RCLCPP_INFO(this->get_logger(), "No sc_data directory found at %s",
                sc_dir.string().c_str());
    sc_loaded_ = false;
    return false;
  }

  // Read metadata
  std::filesystem::path meta_path = sc_dir / "metadata.txt";
  if (!std::filesystem::exists(meta_path)) {
    sc_loaded_ = false;
    return false;
  }

  size_t N;
  int rows, cols;
  {
    std::ifstream f(meta_path);
    f >> N >> rows >> cols;
    f.close();
  }

  if (N == 0) {
    sc_loaded_ = false;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading SC database: N=%zu, rows=%d, cols=%d",
              N, rows, cols);

  // Load polarcontexts
  {
    std::ifstream f(sc_dir / "polarcontexts.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontexts_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontexts_[i].resize(rows, cols);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontexts_[i].data()),
             rows * cols * sizeof(double));
    }
    f.close();
  }

  // Load invkeys
  {
    std::ifstream f(sc_dir / "invkeys.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontext_invkeys_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontext_invkeys_[i].resize(rows, 1);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontext_invkeys_[i].data()),
             rows * sizeof(double));
    }
    f.close();
  }

  // Load vkeys
  {
    std::ifstream f(sc_dir / "vkeys.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontext_vkeys_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontext_vkeys_[i].resize(1, cols);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontext_vkeys_[i].data()),
             cols * sizeof(double));
    }
    f.close();
  }

  // Load invkeys_mat
  {
    std::ifstream f(sc_dir / "invkeys_mat.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontext_invkeys_mat_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontext_invkeys_mat_[i].resize(rows);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontext_invkeys_mat_[i].data()),
             rows * sizeof(float));
    }
    f.close();
  }

  // Rebuild KD-tree
  sc_manager_.rebuildTree();
  sc_loaded_ = true;
  return true;
}

bool IcpNode::loadKeyframePoses() {
  std::filesystem::path poses_path = map_dir_ / "poses.txt";
  if (!std::filesystem::exists(poses_path)) {
    RCLCPP_WARN(this->get_logger(), "No poses.txt found at %s",
                poses_path.string().c_str());
    return false;
  }

  std::ifstream f(poses_path);
  std::string line;
  keyframe_poses_.clear();
  int line_num = 0;
  while (std::getline(f, line)) {
    ++line_num;
    if (line.empty()) continue;
    std::istringstream iss(line);
    KeyframePose kp;
    if (!(iss >> kp.filename >> kp.x >> kp.y >> kp.z >> kp.qw >> kp.qx >> kp.qy >> kp.qz)) {
      RCLCPP_WARN(this->get_logger(),
                  "Skipping malformed line %d in %s: \"%s\"",
                  line_num, poses_path.string().c_str(), line.c_str());
      continue;
    }
    double norm = std::sqrt(kp.qw * kp.qw + kp.qx * kp.qx + kp.qy * kp.qy + kp.qz * kp.qz);
    if (norm < 1e-9) {
      RCLCPP_WARN(this->get_logger(),
                  "Skipping line %d with zero quaternion in %s",
                  line_num, poses_path.string().c_str());
      continue;
    }
    keyframe_poses_.push_back(kp);
  }
  f.close();

  RCLCPP_INFO(this->get_logger(), "Loaded %zu keyframe poses from %s",
              keyframe_poses_.size(), poses_path.string().c_str());
  return !keyframe_poses_.empty();
}

PointCloudXYZI::Ptr IcpNode::buildSubmap(int center_idx, int half_range) {
  PointCloudXYZI::Ptr submap(new PointCloudXYZI);

  std::filesystem::path patches_dir = map_dir_ / "patches";
  if (!std::filesystem::exists(patches_dir)) {
    RCLCPP_WARN(this->get_logger(), "No patches directory found");
    return submap;
  }

  int min_idx = std::max(0, center_idx - half_range);
  int max_idx = std::min(static_cast<int>(keyframe_poses_.size()) - 1, center_idx + half_range);

  pcl::PCDReader reader;
  for (int i = min_idx; i <= max_idx; i++) {
    std::filesystem::path patch_path = patches_dir / keyframe_poses_[i].filename;
    if (!std::filesystem::exists(patch_path))
      continue;

    PointCloudXYZI::Ptr patch(new PointCloudXYZI);
    reader.read(patch_path.string(), *patch);

    // Transform patch to global coordinates
    Eigen::Quaterniond q(keyframe_poses_[i].qw, keyframe_poses_[i].qx,
                         keyframe_poses_[i].qy, keyframe_poses_[i].qz);
    Eigen::Vector3d t(keyframe_poses_[i].x, keyframe_poses_[i].y, keyframe_poses_[i].z);

    PointCloudXYZI::Ptr transformed(new PointCloudXYZI);
    pcl::transformPointCloud(*patch, *transformed, t.cast<float>(),
                             Eigen::Quaternionf(q.cast<float>()));
    *submap += *transformed;
  }

  // Voxel filter the submap
  if (!submap->empty()) {
    pcl::VoxelGrid<PointType> voxel;
    double leaf = 0.1;  // use refine resolution
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(submap);
    voxel.filter(*submap);
  }

  return submap;
}

std::vector<int> IcpNode::filterKeyframesByGps(double gps_x, double gps_y, double radius) const {
  std::vector<int> candidates;
  double r2 = radius * radius;
  for (int i = 0; i < static_cast<int>(keyframe_poses_.size()); i++) {
    double dx = keyframe_poses_[i].x - gps_x;
    double dy = keyframe_poses_[i].y - gps_y;
    if (dx * dx + dy * dy <= r2) {
      candidates.push_back(i);
    }
  }
  return candidates;
}

bool IcpNode::scRelocalize(PointCloudXYZI::Ptr cloud, Eigen::Matrix4d &result_transform) {
  if (!sc_loaded_ || keyframe_poses_.empty()) {
    return false;
  }

  // Use SC to find best matching keyframe (GPS-filtered if available)
  std::pair<int, float> sc_result;
  {
    std::lock_guard<std::mutex> lock(gps_mutex_);
    if (has_gps_position_) {
      auto candidates = filterKeyframesByGps(gps_map_x_, gps_map_y_, gps_filter_radius_);
      RCLCPP_INFO(this->get_logger(),
                  "GPS-filtered SC search: %zu/%zu keyframes within %.1fm of GPS (%.2f, %.2f)",
                  candidates.size(), keyframe_poses_.size(), gps_filter_radius_,
                  gps_map_x_, gps_map_y_);
      if (!candidates.empty()) {
        sc_result = sc_manager_.detectBestMatchAmongCandidates(*cloud, candidates);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "GPS filter returned 0 candidates, falling back to full SC search");
        sc_result = sc_manager_.detectBestMatchForScan(*cloud);
      }
    } else {
      sc_result = sc_manager_.detectBestMatchForScan(*cloud);
    }
  }
  int match_idx = sc_result.first;
  float yaw_diff = sc_result.second;

  if (match_idx < 0 || match_idx >= static_cast<int>(keyframe_poses_.size())) {
    RCLCPP_INFO(this->get_logger(), "SC found no match in database");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "SC matched keyframe %d (yaw_diff=%.1f deg, sc_dist=%.4f, thresh=%.4f)",
              match_idx, yaw_diff * 180.0 / M_PI, sc_manager_.last_match_dist, sc_manager_.SC_DIST_THRES);

  // Build submap around matched keyframe
  PointCloudXYZI::Ptr submap = buildSubmap(match_idx, submap_half_range_);
  if (submap->empty()) {
    RCLCPP_WARN(this->get_logger(), "Built empty submap for keyframe %d", match_idx);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Built submap with %zu points around keyframe %d",
              submap->size(), match_idx);

  // Construct initial guess from matched keyframe pose + SC yaw difference.
  // distanceBtnScanContext(current, keyframe) returns the column shift that
  // rotates the keyframe descriptor into the current descriptor, so:
  //   current_yaw = keyframe_yaw - yaw_diff
  const KeyframePose &kp = keyframe_poses_[match_idx];
  Eigen::Quaterniond q_kf(kp.qw, kp.qx, kp.qy, kp.qz);
  Eigen::Vector3d t_kf(kp.x, kp.y, kp.z);

  // Apply SC yaw correction
  Eigen::AngleAxisd yaw_correction(-static_cast<double>(yaw_diff), Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d rot_init = yaw_correction.toRotationMatrix() * q_kf.toRotationMatrix();

  Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
  init_guess.block<3, 3>(0, 0) = rot_init;
  init_guess.block<3, 1>(0, 3) = t_kf;

  // Diagnostic: print source cloud bounding box and init guess
  {
    Eigen::Vector4f src_min, src_max, tgt_min, tgt_max;
    pcl::getMinMax3D(*cloud, src_min, src_max);
    pcl::getMinMax3D(*submap, tgt_min, tgt_max);
    RCLCPP_INFO(this->get_logger(),
                "SC diag: source(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f], "
                "submap(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f]",
                cloud->size(),
                src_min[0], src_min[1], src_min[2], src_max[0], src_max[1], src_max[2],
                submap->size(),
                tgt_min[0], tgt_min[1], tgt_min[2], tgt_max[0], tgt_max[1], tgt_max[2]);
    RCLCPP_INFO(this->get_logger(),
                "SC diag: init_guess pos=(%.2f,%.2f,%.2f), kf_pos=(%.2f,%.2f,%.2f)",
                init_guess(0,3), init_guess(1,3), init_guess(2,3), kp.x, kp.y, kp.z);
  }

  // Downsample source cloud
  PointCloudXYZI::Ptr source_rough(new PointCloudXYZI);
  PointCloudXYZI::Ptr source_refine(new PointCloudXYZI);
  voxel_rough_filter_.setInputCloud(cloud);
  voxel_rough_filter_.filter(*source_rough);
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*source_refine);

  source_rough = sanitizeCloud<pcl::PointXYZI>(source_rough);
  source_refine = sanitizeCloud<pcl::PointXYZI>(source_refine);

  // Clip source cloud height to match submap z-range to avoid score inflation
  {
    Eigen::Vector4f tgt_min, tgt_max;
    pcl::getMinMax3D(*submap, tgt_min, tgt_max);
    float z_margin = 0.5f;
    float z_lo = tgt_min[2] - z_margin;
    float z_hi = tgt_max[2] + z_margin;
    auto height_filter = [z_lo, z_hi](const PointCloudXYZI::Ptr &c) {
      PointCloudXYZI::Ptr filtered(new PointCloudXYZI);
      filtered->reserve(c->size());
      for (const auto &p : c->points) {
        if (p.z >= z_lo && p.z <= z_hi) filtered->push_back(p);
      }
      return filtered;
    };
    source_rough = height_filter(source_rough);
    source_refine = height_filter(source_refine);
    RCLCPP_INFO(this->get_logger(),
                "SC height clip [%.1f, %.1f]: rough=%zu, refine=%zu pts",
                z_lo, z_hi, source_rough->size(), source_refine->size());
  }

  if (source_rough->size() < kMinAlignmentPoints ||
      source_refine->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(), "SC relocalize: insufficient points after voxel filter");
    return false;
  }

  // ICP fine alignment helper (reused for both NDT→ICP and direct ICP paths)
  // Use relaxed thresholds for SC relocalization (initial alignment is rough)
  double sc_icp_thresh = std::max(thresh_, 0.5);
  pcl::IterativeClosestPoint<PointType, PointType> icp_fine;
  icp_fine.setMaximumIterations(50);
  icp_fine.setMaxCorrespondenceDistance(3.0);
  icp_fine.setTransformationEpsilon(1e-6);
  icp_fine.setEuclideanFitnessEpsilon(1e-6);
  icp_fine.setInputSource(source_refine);
  icp_fine.setInputTarget(submap);

  // Stage 1: NDT coarse alignment against submap
  PointCloudXYZI::Ptr ndt_aligned(new PointCloudXYZI);
  ndt_.setInputSource(source_rough);
  ndt_.setInputTarget(submap);
  ndt_.align(*ndt_aligned, init_guess.cast<float>());

  bool ndt_ok = ndt_.hasConverged() && ndt_.getFitnessScore() <= ndt_score_thresh_;
  if (ndt_ok) {
    RCLCPP_INFO(this->get_logger(), "NDT converged with score=%.4f", ndt_.getFitnessScore());

    // Stage 2a: ICP fine alignment using NDT result
    Eigen::Matrix4f ndt_result = ndt_.getFinalTransformation();
    PointCloudXYZI::Ptr icp_aligned(new PointCloudXYZI);
    icp_fine.align(*icp_aligned, ndt_result);

    if (icp_fine.hasConverged() && icp_fine.getFitnessScore() <= sc_icp_thresh) {
      result_transform = icp_fine.getFinalTransformation().cast<double>();
      RCLCPP_INFO(this->get_logger(),
                  "SC+NDT+ICP relocalization success! icp_score=%.4f",
                  icp_fine.getFitnessScore());
      return true;
    }
    RCLCPP_INFO(this->get_logger(),
                "SC+NDT+ICP failed at ICP stage: converged=%d, score=%.3f (thresh=%.3f)",
                icp_fine.hasConverged(), icp_fine.getFitnessScore(), sc_icp_thresh);
  } else {
    RCLCPP_INFO(this->get_logger(), "SC+NDT failed: converged=%d, score=%.3f",
                ndt_.hasConverged(), ndt_.getFitnessScore());
  }

  // Stage 2b: NDT failed or NDT+ICP failed — try ICP directly with SC initial guess
  // SC provides a reasonable position + yaw, often good enough for ICP without NDT
  RCLCPP_INFO(this->get_logger(), "Trying SC+ICP directly (bypassing NDT)...");
  PointCloudXYZI::Ptr icp_direct_aligned(new PointCloudXYZI);
  icp_fine.align(*icp_direct_aligned, init_guess.cast<float>());

  if (icp_fine.hasConverged() && icp_fine.getFitnessScore() <= sc_icp_thresh) {
    result_transform = icp_fine.getFinalTransformation().cast<double>();
    RCLCPP_INFO(this->get_logger(),
                "SC+ICP direct relocalization success! icp_score=%.4f",
                icp_fine.getFitnessScore());
    return true;
  }

  RCLCPP_INFO(this->get_logger(),
              "SC+ICP direct also failed: converged=%d, score=%.3f (thresh=%.3f)",
              icp_fine.hasConverged(), icp_fine.getFitnessScore(), sc_icp_thresh);

  // Store SC keyframe pose so grid-search fallback uses it instead of (0,0,0)
  sc_fallback_pose_.position.x = kp.x;
  sc_fallback_pose_.position.y = kp.y;
  sc_fallback_pose_.position.z = kp.z;
  Eigen::Quaterniond q_init(rot_init);
  sc_fallback_pose_.orientation.w = q_init.w();
  sc_fallback_pose_.orientation.x = q_init.x();
  sc_fallback_pose_.orientation.y = q_init.y();
  sc_fallback_pose_.orientation.z = q_init.z();
  has_sc_fallback_pose_ = true;

  return false;
}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr incoming_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *incoming_cloud);

  // Transform to laser_frame_id_ if the incoming cloud is in a different frame
  std::string source_frame = msg->header.frame_id;
  if (!source_frame.empty() && source_frame != laser_frame_id_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
          laser_frame_id_, source_frame,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(0.1));
      Eigen::Vector3f t(
          tf_stamped.transform.translation.x,
          tf_stamped.transform.translation.y,
          tf_stamped.transform.translation.z);
      Eigen::Quaternionf q(
          tf_stamped.transform.rotation.w,
          tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y,
          tf_stamped.transform.rotation.z);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*incoming_cloud, *transformed, t, q);
      incoming_cloud = transformed;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Cannot transform cloud from '%s' to '%s': %s",
          source_frame.c_str(), laser_frame_id_.c_str(), ex.what());
      return;
    }
  }

  std::size_t removed_points = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr clean =
      sanitizeCloud<pcl::PointXYZI>(incoming_cloud, &removed_points);
  if (removed_points > 0) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropped %zu non-finite scan points before ICP", removed_points);
  }

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *cloud_in_ = *clean;
    cloud_stamp_ = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
        ? now()
        : rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  }

  if (clean->empty()) {
    return;
  }

  if (clean->size() < min_points_for_alignment_) {
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for denser scan before ICP: %zu / %zu points",
        clean->size(), min_points_for_alignment_);
    return;
  }

  const rclcpp::Time current_time =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
          ? now()
          : rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());

  // Already localized — continuous realignment is handled by the timer
  {
    std::lock_guard lock(mutex_);
    if (is_ready_) {
      return;
    }
  }

  if (initial_alignment_delay_sec_ > 0.0) {
    const double startup_age = (now() - node_start_time_).seconds();
    if (startup_age < initial_alignment_delay_sec_) {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Waiting %.1fs before initial ICP alignment so FAST-LIVO can stabilize (%.1f / %.1f)",
          initial_alignment_delay_sec_ - startup_age,
          startup_age,
          initial_alignment_delay_sec_);
      return;
    }
  }

  // Initial localization: retry periodically
  bool should_retry =
      first_scan_ ||
      (current_time - last_auto_realign_time_).seconds() >=
          auto_realign_interval_sec_;
  if (!should_retry) {
    return;
  }

  bool expected = false;
  if (!alignment_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  last_auto_realign_time_ = current_time;
  first_scan_ = false;

  // Build the initial guess pose for ICP
  auto pose_msg =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose_msg->header = msg->header;
  pose_msg->pose.pose = initial_pose_;
  RCLCPP_INFO(this->get_logger(),
              "Attempting ICP auto-alignment with %zu points",
              clean->size());

  {
    std::lock_guard<std::mutex> lock(align_thread_mutex_);
    if (align_thread_.joinable()) {
      align_thread_.join();
    }
    align_thread_ = std::thread([this, pose_msg]() {
      try {
        initialPoseCallback(pose_msg, false);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "initialPoseCallback exception: %s", e.what());
      }
      alignment_in_progress_.store(false);
    });
  }
}

IcpResult IcpNode::runIcpRefine(const PointCloudXYZI::Ptr &source,
                               const Eigen::Matrix4d &initial_guess) {
  IcpResult result;

  std::size_t removed_points = 0;
  PointCloudXYZI::Ptr refine_source(new PointCloudXYZI);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);
  refine_source = sanitizeCloud<pcl::PointXYZI>(refine_source, &removed_points);

  if (refine_source->size() < min_points_for_alignment_) {
    RCLCPP_WARN(this->get_logger(),
                "runIcpRefine: insufficient points (%zu)", refine_source->size());
    return result;
  }

  icp_refine_.setInputSource(refine_source);

  Eigen::Vector3d crop_center(initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3));
  PointCloudXYZI::Ptr local_map = cropMapByPosition(refine_map_, crop_center, map_crop_radius_);
  if (local_map->size() < min_points_for_alignment_) {
    local_map = refine_map_;
  }

  icp_refine_.setInputTarget(local_map);
  pcl::PointCloud<pcl::PointXYZI> aligned;
  icp_refine_.align(aligned, initial_guess.cast<float>());

  result.score = icp_refine_.getFitnessScore();
  result.converged = icp_refine_.hasConverged() && result.score < continuous_thresh_;
  result.map_to_laser = icp_refine_.getFinalTransformation().cast<double>();
  return result;
}

void IcpNode::publishLocalizationHold(const std::string &reason) {
  if (localization_hold_pub_ == nullptr) {
    return;
  }

  std_msgs::msg::Empty msg;
  localization_hold_pub_->publish(msg);
  RCLCPP_WARN(this->get_logger(),
              "Localization hold requested: %s", reason.c_str());
}

void IcpNode::checkLioYawJump(double lio_yaw) {
  // Expire stale jump flag
  if (lio_yaw_jump_detected_) {
    double elapsed = (now() - lio_jump_detect_time_).seconds();
    if (elapsed > lio_jump_detection_timeout_) {
      RCLCPP_WARN(this->get_logger(),
                  "LIO yaw jump flag expired after %.1fs", elapsed);
      lio_yaw_jump_detected_ = false;
      consecutive_reject_count_ = 0;
      icp_wheel_yaw_offset_valid_ = false;  // recalibrate after recovery
    }
  }

  // Read wheel odom yaw
  double wheel_yaw;
  bool wheel_valid;
  {
    std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
    wheel_yaw = wheel_odom_yaw_;
    wheel_valid = wheel_odom_valid_;
  }

  if (!wheel_valid) {
    return;
  }

  // Compute current divergence = lio_yaw - wheel_yaw (normalized to [-pi, pi])
  double divergence = lio_yaw - wheel_yaw;
  if (divergence > M_PI) divergence -= 2.0 * M_PI;
  if (divergence < -M_PI) divergence += 2.0 * M_PI;

  if (!prev_divergence_valid_) {
    prev_lio_wheel_divergence_ = divergence;
    prev_divergence_valid_ = true;
    RCLCPP_INFO(this->get_logger(),
                "LIO-wheel divergence initialized: %.1f°",
                divergence * 180.0 / M_PI);
    return;
  }

  // How much did the divergence change since last sample?
  double divergence_delta = divergence - prev_lio_wheel_divergence_;
  if (divergence_delta > M_PI) divergence_delta -= 2.0 * M_PI;
  if (divergence_delta < -M_PI) divergence_delta += 2.0 * M_PI;

  RCLCPP_DEBUG(this->get_logger(),
      "LIO-wheel divergence: cur=%.1f° prev=%.1f° delta=%.1f° (thresh=%.0f°)",
      divergence * 180.0 / M_PI,
      prev_lio_wheel_divergence_ * 180.0 / M_PI,
      divergence_delta * 180.0 / M_PI,
      lio_jump_divergence_thresh_ * 180.0 / M_PI);

  // Update previous divergence
  prev_lio_wheel_divergence_ = divergence;

  // Detect: divergence changed more than threshold → LIO jumped but wheels didn't
  if (std::abs(divergence_delta) > lio_jump_divergence_thresh_) {
    lio_yaw_jump_detected_ = true;
    lio_jump_detect_time_ = now();
    RCLCPP_WARN(this->get_logger(),
        "LIO yaw jump DETECTED: divergence_delta=%.1f° (thresh=%.0f°) "
        "— LIO drifted from wheel odom, will allow high-confidence ICP through",
        std::abs(divergence_delta) * 180.0 / M_PI,
        lio_jump_divergence_thresh_ * 180.0 / M_PI);
  }
}

void IcpNode::continuousRealignTimerCallback() {
  // Only fire after initial localization is done
  {
    std::lock_guard lock(mutex_);
    if (!is_ready_) {
      return;
    }
  }

  // Check if alignment is already running
  bool expected = false;
  if (!alignment_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  // Grab the latest cached cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_snapshot(
      new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Time scan_stamp;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_in_->empty()) {
      alignment_in_progress_.store(false);
      return;
    }
    *cloud_snapshot = *cloud_in_;
    scan_stamp = cloud_stamp_;
  }

  // Check cloud freshness — reject if too old
  const double cloud_age = (now() - scan_stamp).seconds();
  if (cloud_age > continuous_realign_interval_sec_ * 5.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Cached cloud too old (%.1fs), skipping continuous realign",
                          cloud_age);
    alignment_in_progress_.store(false);
    return;
  }

  if (cloud_snapshot->size() < min_points_for_alignment_) {
    alignment_in_progress_.store(false);
    return;
  }

  // Build initial guess from current map->odom * odom->base
  auto pose_msg =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose_msg->header.stamp = scan_stamp;
  pose_msg->header.frame_id = map_frame_id_;

  Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d map_to_odom_mat = Eigen::Matrix4d::Identity();

  try {
    auto tf_odom_base = tf_buffer_->lookupTransform(
        odom_frame_id_, base_frame_id_,
        scan_stamp,
        rclcpp::Duration::from_seconds(0.1));
    Eigen::Quaterniond q_ob(
        tf_odom_base.transform.rotation.w,
        tf_odom_base.transform.rotation.x,
        tf_odom_base.transform.rotation.y,
        tf_odom_base.transform.rotation.z);
    odom_to_base.block<3, 3>(0, 0) = q_ob.toRotationMatrix();
    odom_to_base(0, 3) = tf_odom_base.transform.translation.x;
    odom_to_base(1, 3) = tf_odom_base.transform.translation.y;
    odom_to_base(2, 3) = tf_odom_base.transform.translation.z;

    // Check for LIO yaw jump using wheel odometry as reference
    {
      double lio_yaw = std::atan2(odom_to_base(1, 0), odom_to_base(0, 0));
      checkLioYawJump(lio_yaw);
    }

    {
      std::lock_guard lock(mutex_);
      Eigen::Quaterniond q_mo(
          map_to_odom_.transform.rotation.w,
          map_to_odom_.transform.rotation.x,
          map_to_odom_.transform.rotation.y,
          map_to_odom_.transform.rotation.z);
      map_to_odom_mat.block<3, 3>(0, 0) = q_mo.toRotationMatrix();
      map_to_odom_mat(0, 3) = map_to_odom_.transform.translation.x;
      map_to_odom_mat(1, 3) = map_to_odom_.transform.translation.y;
      map_to_odom_mat(2, 3) = map_to_odom_.transform.translation.z;
    }

    Eigen::Matrix4d map_to_base = map_to_odom_mat * odom_to_base;
    Eigen::Quaterniond q_mb(map_to_base.block<3, 3>(0, 0));
    pose_msg->pose.pose.position.x = map_to_base(0, 3);
    pose_msg->pose.pose.position.y = map_to_base(1, 3);
    pose_msg->pose.pose.position.z = map_to_base(2, 3);
    pose_msg->pose.pose.orientation.w = q_mb.w();
    pose_msg->pose.pose.orientation.x = q_mb.x();
    pose_msg->pose.pose.orientation.y = q_mb.y();
    pose_msg->pose.pose.orientation.z = q_mb.z();
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Continuous realign timer: TF lookup failed: %s", ex.what());
    alignment_in_progress_.store(false);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Continuous re-alignment with %zu points (cloud age: %.1fs)",
              cloud_snapshot->size(), cloud_age);

  // --- Dual-ICP recovery for LIO yaw jump (Mode B) ---
  // When LIO yaw flip is detected, the normal ICP path uses a flipped
  // initial_guess and may converge to the wrong (mirrored) solution.
  // Instead, run ICP from TWO initial guesses: normal + 180°-flipped,
  // and pick the one with the best score.
  if (lio_yaw_jump_detected_) {
    RCLCPP_WARN(this->get_logger(),
        "Dual-ICP recovery: LIO yaw jump active, running two-guess ICP");

    // Compute base_to_laser (identity when laser_frame == base_frame)
    Eigen::Matrix4d base_to_laser = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
    try {
      if (base_frame_id_ != laser_frame_id_) {
        auto tf_bl = tf_buffer_->lookupTransform(
            base_frame_id_, laser_frame_id_,
            rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(0.5));
        Eigen::Quaterniond q_bl(
            tf_bl.transform.rotation.w, tf_bl.transform.rotation.x,
            tf_bl.transform.rotation.y, tf_bl.transform.rotation.z);
        base_to_laser.block<3, 3>(0, 0) = q_bl.toRotationMatrix();
        base_to_laser(0, 3) = tf_bl.transform.translation.x;
        base_to_laser(1, 3) = tf_bl.transform.translation.y;
        base_to_laser(2, 3) = tf_bl.transform.translation.z;
      }
      // laser→odom for computing map→odom from ICP result
      auto tf_lo = tf_buffer_->lookupTransform(
          laser_frame_id_, odom_frame_id_,
          scan_stamp,
          rclcpp::Duration::from_seconds(0.5));
      Eigen::Quaterniond q_lo(
          tf_lo.transform.rotation.w, tf_lo.transform.rotation.x,
          tf_lo.transform.rotation.y, tf_lo.transform.rotation.z);
      laser_to_odom.block<3, 3>(0, 0) = q_lo.toRotationMatrix();
      laser_to_odom(0, 3) = tf_lo.transform.translation.x;
      laser_to_odom(1, 3) = tf_lo.transform.translation.y;
      laser_to_odom(2, 3) = tf_lo.transform.translation.z;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
          "Dual-ICP: TF lookup failed: %s, falling back to normal path", ex.what());
      goto normal_path;
    }

    {
      // guess_A: normal initial guess = map→odom × odom→base × base→laser
      Eigen::Matrix4d guess_A = map_to_odom_mat * odom_to_base * base_to_laser;

      // guess_B: same position, yaw rotated 180°
      Eigen::Matrix4d yaw_flip = Eigen::Matrix4d::Identity();
      yaw_flip(0, 0) = -1.0;  // cos(180°)
      yaw_flip(1, 1) = -1.0;  // cos(180°)
      // yaw_flip(0,1)=0, yaw_flip(1,0)=0 (sin(180°)=0)
      Eigen::Matrix4d guess_B = guess_A;
      guess_B.block<3, 3>(0, 0) = yaw_flip.block<3, 3>(0, 0) * guess_A.block<3, 3>(0, 0);

      RCLCPP_INFO(this->get_logger(),
          "Dual-ICP: guess_A yaw=%.1f°, guess_B yaw=%.1f°",
          std::atan2(guess_A(1, 0), guess_A(0, 0)) * 180.0 / M_PI,
          std::atan2(guess_B(1, 0), guess_B(0, 0)) * 180.0 / M_PI);

      IcpResult result_A = runIcpRefine(cloud_snapshot, guess_A);
      IcpResult result_B = runIcpRefine(cloud_snapshot, guess_B);

      RCLCPP_INFO(this->get_logger(),
          "Dual-ICP results: A(conv=%d, score=%.4f) B(conv=%d, score=%.4f)",
          result_A.converged ? 1 : 0, result_A.score,
          result_B.converged ? 1 : 0, result_B.score);

      double wheel_yaw_now = 0.0;
      bool wheel_ready = false;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_yaw_now = wheel_odom_yaw_;
        wheel_ready = wheel_odom_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      }

      if (!wheel_ready || !icp_wheel_yaw_offset_valid_) {
        RCLCPP_WARN(this->get_logger(),
            "Dual-ICP: wheel yaw baseline unavailable, keeping last good pose");
        publishLocalizationHold("dual_icp_no_wheel_baseline");
        alignment_in_progress_.store(false);
        return;
      }

      const double expected_base_yaw =
          normalizeAngle(wheel_yaw_now + icp_wheel_yaw_offset_);
      const double score_limit = thresh_ * lio_jump_icp_score_factor_;

      const auto candidate_yaw_error = [&](const IcpResult &candidate) {
        if (!candidate.converged || candidate.score > score_limit) {
          return std::numeric_limits<double>::infinity();
        }
        const double candidate_yaw = std::atan2(candidate.map_to_laser(1, 0),
                                                candidate.map_to_laser(0, 0));
        return std::abs(normalizeAngle(candidate_yaw - expected_base_yaw));
      };

      const double yaw_error_A = candidate_yaw_error(result_A);
      const double yaw_error_B = candidate_yaw_error(result_B);
      RCLCPP_INFO(this->get_logger(),
          "Dual-ICP wheel gate: expected_base_yaw=%.1f°, A_err=%.1f°, B_err=%.1f°",
          expected_base_yaw * 180.0 / M_PI,
          std::isfinite(yaw_error_A) ? yaw_error_A * 180.0 / M_PI : -1.0,
          std::isfinite(yaw_error_B) ? yaw_error_B * 180.0 / M_PI : -1.0);

      IcpResult *best = nullptr;
      if (yaw_error_A <= icp_wheel_recovery_yaw_gate_thresh_ &&
          yaw_error_B <= icp_wheel_recovery_yaw_gate_thresh_) {
        best = (result_A.score <= result_B.score) ? &result_A : &result_B;
      } else if (yaw_error_A <= icp_wheel_recovery_yaw_gate_thresh_) {
        best = &result_A;
      } else if (yaw_error_B <= icp_wheel_recovery_yaw_gate_thresh_) {
        best = &result_B;
      }

      if (!best) {
        RCLCPP_WARN(this->get_logger(),
            "Dual-ICP: both candidates violate wheel yaw gate (expected=%.1f°), keeping last good pose",
            expected_base_yaw * 180.0 / M_PI);
        publishLocalizationHold("dual_icp_wheel_gate_reject");
        alignment_in_progress_.store(false);
        return;
      }

      // Compute map→odom = best_map_to_laser × laser→odom
      Eigen::Matrix4d new_map_to_odom = best->map_to_laser * laser_to_odom;

      // Update last_accepted_base from the ICP result (map→laser = map→base when laser=base)
      const double new_base_x = best->map_to_laser(0, 3);
      const double new_base_y = best->map_to_laser(1, 3);
      const double new_base_yaw = std::atan2(best->map_to_laser(1, 0),
                                              best->map_to_laser(0, 0));

      RCLCPP_WARN(this->get_logger(),
          "Dual-ICP RECOVERY: accepting best result (score=%.4f, "
          "base=(%.2f,%.2f,%.1f°), expected_yaw=%.1f°) — force-snapping map->odom",
          best->score, new_base_x, new_base_y,
          new_base_yaw * 180.0 / M_PI,
          expected_base_yaw * 180.0 / M_PI);

      last_accepted_base_x_ = new_base_x;
      last_accepted_base_y_ = new_base_y;
      last_accepted_base_yaw_ = new_base_yaw;
      last_accepted_base_stamp_ = now();
      last_accepted_base_valid_ = true;
      lio_yaw_jump_detected_ = false;
      consecutive_reject_count_ = 0;
      icp_wheel_yaw_offset_valid_ = false;  // recalibrate after recovery

      updateMapToOdom(new_map_to_odom, /*force_snap=*/true);
    }
    alignment_in_progress_.store(false);
    return;
  }
  normal_path:

  {
    std::lock_guard<std::mutex> lock(align_thread_mutex_);
    if (align_thread_.joinable()) {
      align_thread_.join();
    }
    align_thread_ = std::thread([this, pose_msg]() {
      try {
        initialPoseCallback(pose_msg, /*skip_sc=*/true);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "continuousRealignTimer exception: %s", e.what());
      }
      alignment_in_progress_.store(false);
    });
  }
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
    bool skip_sc) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_snapshot(
      new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Time scan_stamp;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *cloud_snapshot = *cloud_in_;
    scan_stamp = cloud_stamp_;
  }

  std::size_t removed_points = 0;
  cloud_snapshot = sanitizeCloud<pcl::PointXYZI>(cloud_snapshot, &removed_points);
  if (removed_points > 0) {
    RCLCPP_WARN(this->get_logger(),
                "Dropped %zu non-finite points from alignment snapshot",
                removed_points);
  }

  if (cloud_snapshot->empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Skipping ICP alignment because the input cloud is empty");
    return;
  }

  if (cloud_snapshot->size() < min_points_for_alignment_) {
    RCLCPP_INFO(this->get_logger(),
                "Skipping ICP alignment: only %zu points available",
                cloud_snapshot->size());
    return;
  }

  initial_pose_ = projectPoseToPlane(msg->pose.pose);

  // Try SC-based relocalization first if available (skip during continuous re-alignment)
  if (sc_loaded_ && !skip_sc) {
    RCLCPP_INFO(this->get_logger(), "Attempting SC-based relocalization...");

    // SC database was built with body_frame=mid360_link (mapping mode).
    // In navigation mode, body_cloud is in base_link frame.
    // Transform the source cloud from laser_frame (base_link) to mid360_link
    // so that ICP source and submap target are in the same coordinate system.
    PointCloudXYZI::Ptr sc_cloud = cloud_snapshot;
    Eigen::Matrix4d laser_to_sc_body = Eigen::Matrix4d::Identity();
    if (laser_frame_id_ != sc_body_frame_) {
      try {
        // lookupTransform(target, source) returns source→target transform
        // We want laser_frame → sc_body_frame, so target=sc_body, source=laser
        auto tf_l2s = tf_buffer_->lookupTransform(
            sc_body_frame_, laser_frame_id_,
            rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(1.0));
        Eigen::Vector3d t_l2s(tf_l2s.transform.translation.x,
                              tf_l2s.transform.translation.y,
                              tf_l2s.transform.translation.z);
        Eigen::Quaterniond q_l2s(
            tf_l2s.transform.rotation.w, tf_l2s.transform.rotation.x,
            tf_l2s.transform.rotation.y, tf_l2s.transform.rotation.z);
        laser_to_sc_body.block<3, 3>(0, 0) = q_l2s.toRotationMatrix();
        laser_to_sc_body.block<3, 1>(0, 3) = t_l2s;

        sc_cloud.reset(new PointCloudXYZI);
        pcl::transformPointCloud(*cloud_snapshot, *sc_cloud,
                                 laser_to_sc_body.cast<float>());
        RCLCPP_INFO(this->get_logger(),
                    "Transformed scan from %s to %s for SC matching",
                    laser_frame_id_.c_str(), sc_body_frame_.c_str());
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot transform to SC body frame: %s, using raw cloud", ex.what());
      }
    }

    Eigen::Matrix4d sc_result;  // this will be map → sc_body (mid360_link)
    if (scRelocalize(sc_cloud, sc_result)) {
      // SC relocalization succeeded
      // sc_result is map → mid360_link
      // We need map → odom = (map → mid360_link) × (mid360_link → odom)
      // sc_result = T_map^mid360 (transforms points from mid360_link to map)
      // We need T_map^odom = T_map^mid360 × T_mid360^odom
      Eigen::Matrix4d sc_body_from_odom = Eigen::Matrix4d::Identity();
      try {
        // lookupTransform(target, source): returns T_target^source
        // We want T_mid360^odom, so target=mid360_link, source=odom
        geometry_msgs::msg::TransformStamped transform;
        try {
          transform = tf_buffer_->lookupTransform(
              sc_body_frame_, odom_frame_id_,
              scan_stamp,
              rclcpp::Duration::from_seconds(1.0));
        } catch (tf2::ExtrapolationException &) {
          RCLCPP_WARN(this->get_logger(),
                      "SC: Scan timestamp too old for TF, using latest transform");
          transform = tf_buffer_->lookupTransform(
              sc_body_frame_, odom_frame_id_,
              tf2::TimePointZero);
        }
        Eigen::Vector3d t(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);
        Eigen::Quaterniond q(
            transform.transform.rotation.w, transform.transform.rotation.x,
            transform.transform.rotation.y, transform.transform.rotation.z);
        sc_body_from_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
        sc_body_from_odom.block<3, 1>(0, 3) = t;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "TF lookup failed after SC relocalization: %s", ex.what());
        // Fall through to grid-search fallback
        goto fallback;
      }

      {
        // T_map^odom = T_map^mid360 × T_mid360^odom
        Eigen::Matrix4d result = sc_result * sc_body_from_odom;
        const double sc_x = sc_result(0, 3);
        const double sc_y = sc_result(1, 3);
        const double sc_yaw = std::atan2(sc_result(1, 0), sc_result(0, 0));
        updateMapToOdom(result, /*force_snap=*/true);
        seedLastAcceptedBasePose(result, scan_stamp, "SC relocalization");
        RCLCPP_INFO(this->get_logger(),
                    "SC accepted map->%s pose: x=%.2f y=%.2f yaw=%.1f deg",
                    sc_body_frame_.c_str(), sc_x, sc_y, sc_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
                    "SC relocalization ready, publishing TF %s -> %s",
                    map_frame_id_.c_str(), odom_frame_id_.c_str());
      }
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "SC relocalization failed, falling back to grid-search ICP");
  }

fallback:
  // If SC matched a keyframe but NDT/ICP failed, use that keyframe pose
  // as the grid-search center instead of the (possibly zero) initial_pose_.
  geometry_msgs::msg::Pose fallback_pose = msg->pose.pose;
  if (has_sc_fallback_pose_) {
    RCLCPP_INFO(this->get_logger(),
                "Using SC keyframe pose as grid-search center: (%.2f, %.2f)",
                sc_fallback_pose_.position.x, sc_fallback_pose_.position.y);
    fallback_pose = sc_fallback_pose_;
    has_sc_fallback_pose_ = false;
  }
  fallback_pose = projectPoseToPlane(fallback_pose);

  // Original grid-search ICP path
  // Treat /initialpose as the robot base pose, then convert it to the
  // current laser-frame pose using the fixed base->laser extrinsic.
  Eigen::Vector3d pos(fallback_pose.position.x, fallback_pose.position.y,
                      fallback_pose.position.z);
  Eigen::Quaterniond q(
      fallback_pose.orientation.w, fallback_pose.orientation.x,
      fallback_pose.orientation.y, fallback_pose.orientation.z);
  Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
  map_to_base.block<3, 3>(0, 0) = q.toRotationMatrix();
  map_to_base.block<3, 1>(0, 3) = pos;

  Eigen::Matrix4d base_to_laser = Eigen::Matrix4d::Identity();
  if (base_frame_id_ != laser_frame_id_) {
    try {
      auto transform = tf_buffer_->lookupTransform(
          base_frame_id_, laser_frame_id_,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(1.0));
      Eigen::Vector3d t(transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z);
      Eigen::Quaterniond q_fixed(
          transform.transform.rotation.w, transform.transform.rotation.x,
          transform.transform.rotation.y, transform.transform.rotation.z);
      base_to_laser.block<3, 3>(0, 0) = q_fixed.toRotationMatrix();
      base_to_laser.block<3, 1>(0, 3) = t;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for fixed TF %s -> %s before ICP: %s",
                  base_frame_id_.c_str(), laser_frame_id_.c_str(), ex.what());
      return;
    }
  }

  Eigen::Matrix4d initial_guess = map_to_base * base_to_laser;

  Eigen::Matrix4d map_to_laser;
  if (skip_sc) {
    // Continuous re-alignment: skip grid-search, directly refine from current pose
    RCLCPP_INFO(this->get_logger(), "Continuous ICP refine from current pose");

    std::size_t removed_points = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
        new pcl::PointCloud<pcl::PointXYZI>);
    voxel_refine_filter_.setInputCloud(cloud_snapshot);
    voxel_refine_filter_.filter(*refine_source);
    refine_source = sanitizeCloud<pcl::PointXYZI>(refine_source, &removed_points);

    if (refine_source->size() < min_points_for_alignment_) {
      RCLCPP_WARN(this->get_logger(),
                  "Continuous ICP: insufficient points (%zu)", refine_source->size());
      return;
    }

    icp_refine_.setInputSource(refine_source);

    // Crop map around current estimated position
    Eigen::Vector3d crop_center(initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3));
    PointCloudXYZI::Ptr local_refine_map = cropMapByPosition(refine_map_, crop_center, map_crop_radius_);
    if (local_refine_map->size() < min_points_for_alignment_) {
      RCLCPP_WARN(this->get_logger(),
                  "Continuous ICP: cropped map too small (%zu pts), using full map",
                  local_refine_map->size());
      local_refine_map = refine_map_;
    }
    RCLCPP_DEBUG(this->get_logger(),
        "Continuous ICP map crop: %zu -> %zu pts (r=%.1f)",
        refine_map_->size(), local_refine_map->size(), map_crop_radius_);

    icp_refine_.setInputTarget(local_refine_map);
    pcl::PointCloud<pcl::PointXYZI> aligned;
    icp_refine_.align(aligned, initial_guess.cast<float>());

    score_ = icp_refine_.getFitnessScore();
    success_ = icp_refine_.hasConverged() && score_ < continuous_thresh_;

    RCLCPP_INFO(this->get_logger(),
                "Continuous ICP: converged=%d, score=%.4f (thresh=%.2f, initial_thresh=%.2f)",
                icp_refine_.hasConverged() ? 1 : 0, score_, continuous_thresh_, thresh_);

    if (!success_) {
      // Don't clear is_ready_ on continuous failure — keep last good pose
      RCLCPP_WARN(this->get_logger(),
                  "Continuous ICP did not converge, keeping last good pose");
      continuous_reject_streak_++;
      if (continuous_reject_streak_ >= continuous_max_consecutive_rejects_) {
        RCLCPP_ERROR(this->get_logger(),
                     "Continuous ICP failed %d times in a row, but keeping last pose (NOT triggering SC)",
                     continuous_reject_streak_);
      }
      return;
    }
    map_to_laser = icp_refine_.getFinalTransformation().cast<double>();

  } else {
    // Initial alignment: full grid-search
    RCLCPP_INFO(this->get_logger(), "Aligning the pointcloud");
    map_to_laser = multiAlignSync(cloud_snapshot, initial_guess);
    if (!success_) {
      std::lock_guard lock(mutex_);
      is_ready_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "ICP did not converge yet, will retry from the latest scan");
      return;
    }
  }

  Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
  try {
    // Get odom to laser transform — try scan timestamp first,
    // fall back to latest TF if scan is too old (e.g. after long grid-search)
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          laser_frame_id_, odom_frame_id_,
          scan_stamp,
          rclcpp::Duration::from_seconds(1.0));
    } catch (tf2::ExtrapolationException &) {
      RCLCPP_WARN(this->get_logger(),
                  "Scan timestamp too old for TF, using latest transform");
      transform = tf_buffer_->lookupTransform(
          laser_frame_id_, odom_frame_id_,
          tf2::TimePointZero);
    }
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q_odom(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);
    laser_to_odom.block<3, 3>(0, 0) = q_odom.toRotationMatrix();
    laser_to_odom.block<3, 1>(0, 3) = t;
  } catch (tf2::TransformException &ex) {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    is_ready_ = false;
    return;
  }
  Eigen::Matrix4d result = map_to_laser * laser_to_odom;

  // Continuous ICP results must pass the same gates before they can alter
  // map->odom.  The TF conversion above is required to form the correction.
  if (skip_sc && initial_localization_done_) {
    IcpResult icp_result;
    icp_result.converged = icp_refine_.hasConverged();
    icp_result.score = score_;
    icp_result.map_to_laser = map_to_laser;

    Eigen::Matrix4d result_old = Eigen::Matrix4d::Identity();
    {
      std::lock_guard lock(mutex_);
      Eigen::Quaterniond q_old(
          map_to_odom_.transform.rotation.w,
          map_to_odom_.transform.rotation.x,
          map_to_odom_.transform.rotation.y,
          map_to_odom_.transform.rotation.z);
      result_old.block<3, 3>(0, 0) = q_old.toRotationMatrix();
      result_old(0, 3) = map_to_odom_.transform.translation.x;
      result_old(1, 3) = map_to_odom_.transform.translation.y;
      result_old(2, 3) = map_to_odom_.transform.translation.z;
    }

    std::string reject_reason;
    if (!validateContinuousIcpResult(
            icp_result, result_old, result, scan_stamp, reject_reason)) {
      RCLCPP_WARN(this->get_logger(),
                  "Continuous ICP REJECTED by strict validation: %s",
                  reject_reason.c_str());
      ++continuous_reject_streak_;
      return;
    }

    const Eigen::Vector3d current_pose(
        map_to_laser(0, 3), map_to_laser(1, 3),
        std::atan2(map_to_laser(1, 0), map_to_laser(0, 0)));
    recent_icp_poses_.push_back(current_pose);
    if (recent_icp_poses_.size() >
        static_cast<size_t>(continuous_consistency_window_)) {
      recent_icp_poses_.pop_front();
    }
    continuous_reject_streak_ = 0;
    RCLCPP_INFO(this->get_logger(), "Continuous ICP ACCEPTED by strict validation");
  }

  // Sanity check: reject continuous ICP results that produce unreasonably
  // large corrections.  Compare the NEW map→base_link pose against the
  // PREVIOUS one.  This catches both ICP errors AND odom jumps.
  if (skip_sc && initial_localization_done_) {
    // Compute new map→base_link = result(map→odom) × odom→base_link
    // We already have map_to_laser (=map→base_link since laser=base_link)
    // and result = map_to_laser * laser_to_odom (=map→odom).
    // But map_to_laser IS the new map→base estimate from ICP.
    const double new_base_x = map_to_laser(0, 3);
    const double new_base_y = map_to_laser(1, 3);
    const double new_base_yaw = std::atan2(map_to_laser(1, 0), map_to_laser(0, 0));

    if (last_accepted_base_valid_) {
      const double d_x = new_base_x - last_accepted_base_x_;
      const double d_y = new_base_y - last_accepted_base_y_;
      const double pos_delta = std::hypot(d_x, d_y);
      double yaw_delta = std::abs(new_base_yaw - last_accepted_base_yaw_);
      if (yaw_delta > M_PI) yaw_delta = 2.0 * M_PI - yaw_delta;

      // Dynamic yaw threshold: omega_max * dt + margin.
      // At omega_max=1.0 rad/s and 1.5s ICP interval, max expected
      // yaw change ≈ 1.5 rad (~86°).  A fixed 35° or 55° threshold
      // falsely rejects normal turns.  Instead, compute from elapsed
      // time since last accepted result.
      constexpr double kOmegaMax = 1.2;        // slightly above controller's omega_max
      constexpr double kMinYawDelta = 0.61;     // floor: ~35°, always catch 180° flips
      constexpr double kYawMargin = 0.35;       // ~20° margin for LIO drift
      const double dt_since_last =
          (now() - last_accepted_base_stamp_).seconds();
      const double kMaxYawDelta = std::max(
          kMinYawDelta,
          kOmegaMax * dt_since_last + kYawMargin);

      RCLCPP_DEBUG(this->get_logger(),
          "ICP sanity: base (%.2f,%.2f,%.2f°) -> (%.2f,%.2f,%.2f°), "
          "pos_delta=%.3f, yaw_delta=%.3f",
          last_accepted_base_x_, last_accepted_base_y_,
          last_accepted_base_yaw_ * 180.0 / M_PI,
          new_base_x, new_base_y, new_base_yaw * 180.0 / M_PI,
          pos_delta, yaw_delta);

      if (yaw_delta > kMaxYawDelta) {
        // Check if this is a known LIO yaw jump with high-confidence ICP
        const double score_limit = thresh_ * lio_jump_icp_score_factor_;
        if (lio_yaw_jump_detected_ && score_ < score_limit) {
          double wheel_yaw_now;
          bool wheel_ready_now;
          {
            std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
            wheel_yaw_now = wheel_odom_yaw_;
            wheel_ready_now = wheel_odom_valid_ &&
                ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
          }

          if (!wheel_ready_now || !icp_wheel_yaw_offset_valid_) {
            RCLCPP_WARN(this->get_logger(),
                "LIO yaw jump RECOVERY rejected: wheel yaw baseline unavailable");
            publishLocalizationHold("jump_recovery_no_wheel_baseline");
            ++consecutive_reject_count_;
            return;
          }

          const double expected_base_yaw =
              normalizeAngle(wheel_yaw_now + icp_wheel_yaw_offset_);
          const double recovery_yaw_error =
              std::abs(normalizeAngle(new_base_yaw - expected_base_yaw));
          if (recovery_yaw_error > icp_wheel_recovery_yaw_gate_thresh_) {
            RCLCPP_WARN(this->get_logger(),
                "LIO yaw jump RECOVERY rejected: wheel yaw gate failed "
                "(candidate=%.1f°, expected=%.1f°, err=%.1f°, thresh=%.1f°)",
                new_base_yaw * 180.0 / M_PI,
                expected_base_yaw * 180.0 / M_PI,
                recovery_yaw_error * 180.0 / M_PI,
                icp_wheel_recovery_yaw_gate_thresh_ * 180.0 / M_PI);
            publishLocalizationHold("jump_recovery_wheel_gate_reject");
            ++consecutive_reject_count_;
            return;
          }

          // Dual gate passed: wheel odom confirmed LIO jump + ICP score excellent
          RCLCPP_WARN(this->get_logger(),
              "LIO yaw jump RECOVERY: accepting ICP result (yaw_delta=%.1f°, "
              "score=%.4f < %.4f, wheel_err=%.1f°) — force-snapping map->odom",
              yaw_delta * 180.0 / M_PI, score_, score_limit,
              recovery_yaw_error * 180.0 / M_PI);
          lio_yaw_jump_detected_ = false;
          consecutive_reject_count_ = 0;
          icp_wheel_yaw_offset_valid_ = false;  // recalibrate after recovery
          // Update accepted base pose and force-snap TF
          last_accepted_base_x_ = new_base_x;
          last_accepted_base_y_ = new_base_y;
          last_accepted_base_yaw_ = new_base_yaw;
          last_accepted_base_stamp_ = now();
          last_accepted_base_valid_ = true;
          updateMapToOdom(result, /*force_snap=*/true);
          RCLCPP_INFO(this->get_logger(),
                      "ICP ready (jump recovery), publishing TF %s -> %s",
                      map_frame_id_.c_str(), odom_frame_id_.c_str());
          return;
        }

        RCLCPP_WARN(this->get_logger(),
            "Continuous ICP rejected: base yaw_delta=%.1f° (max %.1f°, dt=%.2fs), "
            "pos_delta=%.3f m — ICP likely converged to mirror pose "
            "(lio_jump=%d, score=%.4f, score_limit=%.4f)",
            yaw_delta * 180.0 / M_PI, kMaxYawDelta * 180.0 / M_PI, dt_since_last,
            pos_delta,
            lio_yaw_jump_detected_ ? 1 : 0, score_, score_limit);
        publishLocalizationHold("continuous_icp_mirror_pose_reject");

        // Consecutive reject detection: if sanity check rejects multiple times
        // in a row, LIO is likely drifting gradually (not a sharp jump).
        // Trigger dual-ICP recovery on the next timer callback.
        ++consecutive_reject_count_;
        if (consecutive_reject_count_ >= kMaxConsecutiveRejects && !lio_yaw_jump_detected_) {
          lio_yaw_jump_detected_ = true;
          lio_jump_detect_time_ = now();
          RCLCPP_WARN(this->get_logger(),
              "Consecutive sanity rejects (%d) → triggering dual-ICP recovery",
              consecutive_reject_count_);
        }
        return;
      }
    }

    // Accept: reset consecutive reject counter and update last known good base pose
    consecutive_reject_count_ = 0;

    // Absolute yaw divergence check: compare accepted ICP yaw against wheel odom.
    // This catches gradual LIO drift where per-frame yaw_delta is small but
    // cumulative drift is large (the sanity check above can't catch this).
    {
      double wheel_yaw_now;
      bool wheel_valid_now;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_yaw_now = wheel_odom_yaw_;
        wheel_valid_now = wheel_odom_valid_;
      }

      if (wheel_valid_now) {
        // current offset = accepted_base_yaw - wheel_yaw (normalized to [-pi, pi])
        double cur_offset = new_base_yaw - wheel_yaw_now;
        if (cur_offset > M_PI) cur_offset -= 2.0 * M_PI;
        if (cur_offset < -M_PI) cur_offset += 2.0 * M_PI;

        if (!icp_wheel_yaw_offset_valid_) {
          icp_wheel_yaw_offset_ = cur_offset;
          icp_wheel_yaw_offset_valid_ = true;
          RCLCPP_INFO(this->get_logger(),
              "ICP-wheel yaw offset calibrated: %.1f°",
              cur_offset * 180.0 / M_PI);
        } else {
          // How far has the offset drifted from calibration?
          double offset_drift = cur_offset - icp_wheel_yaw_offset_;
          if (offset_drift > M_PI) offset_drift -= 2.0 * M_PI;
          if (offset_drift < -M_PI) offset_drift += 2.0 * M_PI;

          if (std::abs(offset_drift) > icp_wheel_abs_yaw_thresh_) {
            // ICP result has drifted far from wheel odom baseline — reject and trigger recovery
            RCLCPP_WARN(this->get_logger(),
                "Absolute yaw divergence DETECTED: ICP-wheel offset drift = %.1f° "
                "(calib=%.1f°, cur=%.1f°, thresh=%.0f°) — triggering dual-ICP recovery",
                offset_drift * 180.0 / M_PI,
                icp_wheel_yaw_offset_ * 180.0 / M_PI,
                cur_offset * 180.0 / M_PI,
                icp_wheel_abs_yaw_thresh_ * 180.0 / M_PI);
            lio_yaw_jump_detected_ = true;
            lio_jump_detect_time_ = now();
            publishLocalizationHold("absolute_yaw_divergence_detected");
            // Do NOT update last_accepted_base — this result is suspect
            return;
          }

          // Normal: low-pass update the calibrated offset
          icp_wheel_yaw_offset_ += icp_wheel_yaw_offset_alpha_ * offset_drift;
          // Normalize
          if (icp_wheel_yaw_offset_ > M_PI) icp_wheel_yaw_offset_ -= 2.0 * M_PI;
          if (icp_wheel_yaw_offset_ < -M_PI) icp_wheel_yaw_offset_ += 2.0 * M_PI;
        }
      }
    }

    last_accepted_base_x_ = new_base_x;
    last_accepted_base_y_ = new_base_y;
    last_accepted_base_yaw_ = new_base_yaw;
    last_accepted_base_stamp_ = now();
    last_accepted_base_valid_ = true;
  }

  updateMapToOdom(result);
  const double accepted_base_x = map_to_laser(0, 3);
  const double accepted_base_y = map_to_laser(1, 3);
  const double accepted_base_yaw =
      std::atan2(map_to_laser(1, 0), map_to_laser(0, 0));
  RCLCPP_INFO(this->get_logger(),
              "ICP accepted map->%s pose: x=%.2f y=%.2f yaw=%.1f deg score=%.4f",
              laser_frame_id_.c_str(), accepted_base_x, accepted_base_y,
              accepted_base_yaw * 180.0 / M_PI, score_);
  RCLCPP_INFO(this->get_logger(),
              "ICP ready, publishing TF %s -> %s",
              map_frame_id_.c_str(), odom_frame_id_.c_str());
}

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
  static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = std::asin(std::clamp(-rot(2, 0), -1.0, 1.0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;
  std::size_t removed_points = 0;
  source = sanitizeCloud<pcl::PointXYZI>(source, &removed_points);
  if (source->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(),
                "Skipping ICP: only %zu valid source points available",
                source->size());
    return Eigen::Matrix4d::Zero();
  }

  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
  Eigen::Vector3d rpy = rotate2rpy(rotation);
  Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
  std::vector<Eigen::Matrix4f> candidates;
  Eigen::Matrix4f temp_pose;

  RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (double yaw_delta = -yaw_offset_;
           yaw_delta <= yaw_offset_ + 1e-6;
           yaw_delta += yaw_resolution_) {
        Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                            xyz(2));
        Eigen::AngleAxisf yawAngle(rpy(2) + yaw_delta,
                                   Eigen::Vector3f::UnitZ());
        temp_pose.setIdentity();
        temp_pose.block<3, 3>(0, 0) =
            (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
        temp_pose.block<3, 1>(0, 3) = pos;
        candidates.push_back(temp_pose);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
      new pcl::PointCloud<pcl::PointXYZI>);

  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);

  rough_source = sanitizeCloud<pcl::PointXYZI>(rough_source, &removed_points);
  refine_source = sanitizeCloud<pcl::PointXYZI>(refine_source, &removed_points);

  // Crop maps around initial guess position (+ grid search extent)
  double effective_crop_radius = map_crop_radius_;
  if (effective_crop_radius > 0.0) {
    effective_crop_radius += xy_offset_;
  }
  PointCloudXYZI::Ptr local_rough_map = cropMapByPosition(rough_map_, xyz, effective_crop_radius);
  PointCloudXYZI::Ptr local_refine_map = cropMapByPosition(refine_map_, xyz, effective_crop_radius);
  if (local_rough_map->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(),
        "Cropped rough map too small (%zu pts within %.1fm), using full map",
        local_rough_map->size(), effective_crop_radius);
    local_rough_map = rough_map_;
    local_refine_map = refine_map_;
  }
  RCLCPP_INFO(this->get_logger(),
      "Map crop: rough %zu->%zu, refine %zu->%zu (radius=%.1fm around [%.1f,%.1f])",
      rough_map_->size(), local_rough_map->size(),
      refine_map_->size(), local_refine_map->size(),
      effective_crop_radius, xyz.x(), xyz.y());

  // Clip source cloud height to match target map z-range
  {
    Eigen::Vector4f tgt_min, tgt_max;
    pcl::getMinMax3D(*local_rough_map, tgt_min, tgt_max);
    float z_margin = 0.5f;
    float z_lo = tgt_min[2] - z_margin;
    float z_hi = tgt_max[2] + z_margin;
    auto height_filter = [z_lo, z_hi](const PointCloudXYZI::Ptr &c) {
      PointCloudXYZI::Ptr filtered(new PointCloudXYZI);
      filtered->reserve(c->size());
      for (const auto &p : c->points) {
        if (p.z >= z_lo && p.z <= z_hi) filtered->push_back(p);
      }
      return filtered;
    };
    rough_source = height_filter(rough_source);
    refine_source = height_filter(refine_source);
  }

  if (rough_source->size() < kMinAlignmentPoints ||
      refine_source->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(),
                "Skipping ICP: insufficient valid voxelized points (rough=%zu, refine=%zu)",
                rough_source->size(), refine_source->size());
    return Eigen::Matrix4d::Zero();
  }

  PointCloudXYZI::Ptr align_point(new PointCloudXYZI);

  // Diagnostic: source/target stats for grid search
  {
    Eigen::Vector4f src_min, src_max, tgt_min, tgt_max;
    pcl::getMinMax3D(*rough_source, src_min, src_max);
    pcl::getMinMax3D(*local_rough_map, tgt_min, tgt_max);
    RCLCPP_INFO(this->get_logger(),
                "Grid-search diag: source(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f], "
                "target(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f]",
                rough_source->size(),
                src_min[0], src_min[1], src_min[2], src_max[0], src_max[1], src_max[2],
                local_rough_map->size(),
                tgt_min[0], tgt_min[1], tgt_min[2], tgt_max[0], tgt_max[1], tgt_max[2]);
  }

  Eigen::Matrix4f best_rough_transform;
  double best_rough_score = 10.0;
  bool rough_converge = false;
  int total_converged = 0;
  int total_evaluated = 0;
  double worst_score = 0.0;
  constexpr double kGridSearchTimeoutSec = 15.0;
  auto tic = std::chrono::system_clock::now();
  icp_rough_.setInputTarget(local_rough_map);
  for (Eigen::Matrix4f &init_pose : candidates) {
    // Timeout: abort grid-search if it takes too long
    auto elapsed = std::chrono::duration<double>(
        std::chrono::system_clock::now() - tic).count();
    if (elapsed > kGridSearchTimeoutSec) {
      RCLCPP_WARN(this->get_logger(),
                  "Grid-search timeout after %.1fs (%d/%zu candidates evaluated)",
                  elapsed, total_evaluated, candidates.size());
      break;
    }
    total_evaluated++;
    icp_rough_.setInputSource(rough_source);
    icp_rough_.align(*align_point, init_pose);
    if (!icp_rough_.hasConverged())
      continue;
    double rough_score = icp_rough_.getFitnessScore();
    total_converged++;
    if (rough_score > worst_score)
      worst_score = rough_score;
    if (rough_score > 2 * thresh_)
      continue;
    if (rough_score < best_rough_score) {
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Grid-search done: %zu candidates, %d converged, best_rough=%.4f, worst=%.4f, thresh=%.2f",
              candidates.size(), total_converged, best_rough_score, worst_score, thresh_);

  if (!rough_converge)
    return Eigen::Matrix4d::Zero();

  icp_refine_.setInputSource(refine_source);
  icp_refine_.setInputTarget(local_refine_map);
  icp_refine_.align(*align_point, best_rough_transform);
  score_ = icp_refine_.getFitnessScore();

  RCLCPP_INFO(this->get_logger(),
              "Grid-search refine: converged=%d, score=%.4f (thresh=%.2f)",
              icp_refine_.hasConverged(), score_, thresh_);

  if (!icp_refine_.hasConverged())
    return Eigen::Matrix4d::Zero();
  if (score_ > thresh_)
    return Eigen::Matrix4d::Zero();
  success_ = true;
  auto toc = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = toc - tic;
  RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);
  RCLCPP_INFO(this->get_logger(), "score: %f", score_);

  return icp_refine_.getFinalTransformation().cast<double>();
}

// ========== 连续 ICP 严格验证函数实现 ==========
bool IcpNode::validateContinuousIcpResult(
    const IcpResult &result,
    const Eigen::Matrix4d &map_to_odom_old,
    const Eigen::Matrix4d &map_to_odom_new,
    const rclcpp::Time &scan_stamp,
    std::string &reject_reason) {

  // 0. 基础收敛检查
  if (!result.converged) {
    reject_reason = "ICP not converged";
    return false;
  }

  // 1. 检查 fitness score (越小越好，但我们设置的是最小值，表示质量下限)
  // 注意：PCL的getFitnessScore()返回的是平均距离平方，不是0-1的适配度
  // 这里continuous_min_fitness_score_实际上是一个阈值，而不是百分比
  // 我们应该检查score是否低于阈值
  if (result.score > continuous_max_rmse_) {
    reject_reason = "RMSE too high: " + std::to_string(result.score) +
                    " > " + std::to_string(continuous_max_rmse_);
    return false;
  }

  // 2. 计算 map→odom 的修正量
  Eigen::Vector3d trans_old(map_to_odom_old(0, 3), map_to_odom_old(1, 3), map_to_odom_old(2, 3));
  Eigen::Vector3d trans_new(map_to_odom_new(0, 3), map_to_odom_new(1, 3), map_to_odom_new(2, 3));
  double correction_xy = (trans_new - trans_old).head<2>().norm();

  double yaw_old = std::atan2(map_to_odom_old(1, 0), map_to_odom_old(0, 0));
  double yaw_new = std::atan2(map_to_odom_new(1, 0), map_to_odom_new(0, 0));
  double correction_yaw = std::abs(yaw_new - yaw_old);
  if (correction_yaw > M_PI) {
    correction_yaw = 2.0 * M_PI - correction_yaw;
  }

  // 检查单次修正量
  if (correction_xy > continuous_max_correction_xy_) {
    reject_reason = "XY correction too large: " + std::to_string(correction_xy) +
                    "m > " + std::to_string(continuous_max_correction_xy_) + "m";
    return false;
  }

  if (correction_yaw > continuous_max_correction_yaw_) {
    reject_reason = "Yaw correction too large: " +
                    std::to_string(correction_yaw * 180.0 / M_PI) + "° > " +
                    std::to_string(continuous_max_correction_yaw_ * 180.0 / M_PI) + "°";
    return false;
  }

  // 3. 连续一致性检查
  Eigen::Vector3d current_pose(trans_new.x(), trans_new.y(), yaw_new);

  if (recent_icp_poses_.size() >= static_cast<size_t>(continuous_consistency_window_)) {
    // 检查与最近N帧的一致性
    double max_deviation = 0.0;
    for (const auto &prev_pose : recent_icp_poses_) {
      double xy_diff = (current_pose.head<2>() - prev_pose.head<2>()).norm();
      max_deviation = std::max(max_deviation, xy_diff);
    }

    if (max_deviation > continuous_consistency_tolerance_) {
      reject_reason = "Consistency check failed: max_deviation=" +
                      std::to_string(max_deviation) + "m > " +
                      std::to_string(continuous_consistency_tolerance_) + "m";
      return false;
    }
  }

  // 4. TF 时间戳严格模式检查
  if (continuous_tf_lookup_strict_) {
    try {
      // 尝试精确查找该时间戳的TF
      auto tf_test = tf_buffer_->lookupTransform(
          odom_frame_id_, base_frame_id_,
          scan_stamp,
          rclcpp::Duration::from_seconds(continuous_tf_max_extrapolation_));

      // 检查TF时间差
      double tf_age = std::abs((rclcpp::Time(tf_test.header.stamp) - scan_stamp).seconds());
      if (tf_age > continuous_tf_max_extrapolation_) {
        reject_reason = "TF extrapolation too large: " + std::to_string(tf_age) +
                        "s > " + std::to_string(continuous_tf_max_extrapolation_) + "s";
        return false;
      }
    } catch (tf2::TransformException &ex) {
      reject_reason = std::string("TF lookup failed (strict mode): ") + ex.what();
      return false;
    }
  }

  // 所有检查通过
  return true;
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
