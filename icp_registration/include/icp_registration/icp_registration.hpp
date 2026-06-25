#ifndef ICP_REGISTRATION_HPP
#define ICP_REGISTRATION_HPP

// std
#include <filesystem>
#include <mutex>

#include <atomic>
#include <cstddef>

// ros
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

// ScanContext
#include "Scancontext.h"

namespace icp {
using PointType = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

struct KeyframePose {
  std::string filename;
  double x, y, z;
  double qw, qx, qy, qz;
};

struct IcpResult {
  bool converged = false;
  double score = std::numeric_limits<double>::max();
  Eigen::Matrix4d map_to_laser = Eigen::Matrix4d::Identity();
};

// Get initial map to odom pose estimation using ICP algorithm
class IcpNode : public rclcpp::Node {
public:
  IcpNode(const rclcpp::NodeOptions &options);
  ~IcpNode();
private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void continuousRealignTimerCallback();
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
      bool skip_sc = false);

  Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source,
                                 const Eigen::Matrix4d &init_guess);

  // Single ICP refine pass (used by continuous realign and dual-ICP recovery)
  IcpResult runIcpRefine(const PointCloudXYZI::Ptr &source,
                         const Eigen::Matrix4d &initial_guess);

  // Map cropping: extract local submap around estimated position
  PointCloudXYZI::Ptr cropMapByPosition(
      const PointCloudXYZI::Ptr &full_map,
      const Eigen::Vector3d &position, double radius) const;

  // TF smoothing: update map→odom with optional smooth transition
  void updateMapToOdom(const Eigen::Matrix4d &new_transform, bool force_snap = false);
  Eigen::Matrix4d projectTransformToPlane(const Eigen::Matrix4d &transform) const;
  geometry_msgs::msg::Pose projectPoseToPlane(
      const geometry_msgs::msg::Pose &pose) const;

  // SC-based relocalization
  bool loadScanContextDatabase();
  bool loadKeyframePoses();
  PointCloudXYZI::Ptr buildSubmap(int center_idx, int half_range);
  bool scRelocalize(PointCloudXYZI::Ptr cloud, Eigen::Matrix4d &result_transform);
  bool mapPoseFromTransform(const Eigen::Matrix4d &map_to_odom,
                            const std::string &target_frame,
                            const rclcpp::Time &stamp,
                            Eigen::Matrix4d &map_to_target);
  void seedLastAcceptedBasePose(const Eigen::Matrix4d &map_to_odom,
                                const rclcpp::Time &stamp,
                                const std::string &source);

  // GPS-filtered SC relocalization
  std::vector<int> filterKeyframesByGps(double gps_x, double gps_y, double radius) const;

  // LIO yaw jump detection using wheel odometry
  void checkLioYawJump(double lio_yaw);
  void publishLocalizationHold(const std::string &reason);

  // ROS2 part
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::mutex cloud_mutex_;
  std::atomic<bool> stop_tf_thread_{false};
  std::unique_ptr<std::thread> tf_publisher_thread_;
  std::thread align_thread_;
  std::mutex align_thread_mutex_;

  // Timer for continuous realignment (decoupled from cloud arrival rate)
  rclcpp::TimerBase::SharedPtr continuous_realign_timer_;

  // Voxelfilter used to downsample the pointcloud
  pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;

  // ICP
  int rough_iter_;
  int refine_iter_;
  pcl::IterativeClosestPoint<PointType, PointType> icp_rough_;
  pcl::IterativeClosestPoint<PointType, PointType> icp_refine_;

  // NDT
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;

  // ScanContext
  SCManager sc_manager_;
  bool sc_loaded_;
  std::vector<KeyframePose> keyframe_poses_;
  std::filesystem::path map_dir_;
  int submap_half_range_;
  std::string sc_body_frame_;  // frame used during mapping (mid360_link)

  // NDT parameters
  double ndt_resolution_;
  double ndt_step_size_;
  int ndt_max_iterations_;
  double ndt_epsilon_;
  double ndt_score_thresh_;

  // Store
  PointCloudXYZI::Ptr cloud_in_;
  rclcpp::Time cloud_stamp_{0, 0, RCL_ROS_TIME};  // timestamp of cloud_in_
  PointCloudXYZI::Ptr refine_map_;
  PointCloudXYZI::Ptr rough_map_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
  std::filesystem::path pcd_path_;
  std::string map_frame_id_, odom_frame_id_, base_frame_id_, laser_frame_id_;
  bool success_;
  double score_;
  double thresh_;
  double continuous_thresh_;
  double xy_offset_;
  double yaw_offset_;
  double yaw_resolution_;
  geometry_msgs::msg::Pose initial_pose_;
  bool constrain_to_2d_;

  bool is_ready_;
  bool first_scan_;
  rclcpp::Time node_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_auto_realign_time_;
  double auto_realign_interval_sec_;
  double initial_alignment_delay_sec_;
  double continuous_realign_interval_sec_;
  rclcpp::Time last_continuous_realign_time_;
  std::size_t min_points_for_alignment_;
  std::atomic_bool alignment_in_progress_;

  // Map cropping
  double map_crop_radius_;

  // TF smoothing state
  geometry_msgs::msg::TransformStamped map_to_odom_target_;
  geometry_msgs::msg::TransformStamped map_to_odom_current_;
  rclcpp::Time tf_smooth_start_time_;
  double tf_smooth_duration_;
  bool tf_smoothing_active_;
  bool initial_localization_done_;

  // SC fallback: when SC matches a keyframe but NDT/ICP fails,
  // pass the keyframe pose to grid-search as a better initial guess
  geometry_msgs::msg::Pose sc_fallback_pose_;
  bool has_sc_fallback_pose_ = false;

  // GPS-assisted SC relocalization
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_map_sub_;
  std::mutex gps_mutex_;
  bool has_gps_position_ = false;
  double gps_map_x_ = 0.0;
  double gps_map_y_ = 0.0;
  double gps_filter_radius_ = 20.0;

  // Continuous ICP sanity check: track last accepted map→base pose
  bool last_accepted_base_valid_ = false;
  double last_accepted_base_x_ = 0.0;
  double last_accepted_base_y_ = 0.0;
  double last_accepted_base_yaw_ = 0.0;
  rclcpp::Time last_accepted_base_stamp_{0, 0, RCL_ROS_TIME};

  // Wheel odometry subscription for LIO yaw jump detection
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr localization_hold_pub_;
  std::mutex wheel_odom_mutex_;
  double wheel_odom_yaw_ = 0.0;
  rclcpp::Time wheel_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool wheel_odom_valid_ = false;

  // LIO-wheel yaw divergence tracking
  // divergence = (lio_yaw - wheel_yaw), should stay stable when LIO is healthy
  double prev_lio_wheel_divergence_ = 0.0;
  bool prev_divergence_valid_ = false;

  // LIO yaw jump detection state
  bool lio_yaw_jump_detected_ = false;
  rclcpp::Time lio_jump_detect_time_{0, 0, RCL_ROS_TIME};

  // Consecutive sanity-check reject counter: catches gradual LIO drift
  // that the single-frame divergence detector misses
  int consecutive_reject_count_ = 0;
  static constexpr int kMaxConsecutiveRejects = 1;

  // LIO yaw jump detection parameters
  double lio_jump_divergence_thresh_ = 0.5;  // rad (~29°), divergence change threshold
  double lio_jump_icp_score_factor_ = 0.1;   // score < thresh * factor
  double lio_jump_detection_timeout_ = 10.0;  // seconds

  // Absolute yaw divergence: track calibrated offset between ICP map→base yaw and wheel odom yaw.
  // If the accepted ICP yaw drifts far from (wheel_yaw + offset), LIO is drifting gradually.
  double icp_wheel_yaw_offset_ = 0.0;   // calibrated: accepted_base_yaw - wheel_yaw
  bool icp_wheel_yaw_offset_valid_ = false;
  double icp_wheel_yaw_offset_alpha_ = 0.1;  // low-pass filter gain for offset tracking
  double icp_wheel_abs_yaw_thresh_ = 1.5;    // rad (~86°), trigger dual-ICP if exceeded
  double icp_wheel_recovery_yaw_gate_thresh_ = 0.52;  // rad (~30°)
  double wheel_odom_pose_timeout_ = 0.5;  // seconds
};
} // namespace icp
#endif
