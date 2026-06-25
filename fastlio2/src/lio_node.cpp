#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <atomic>
#include <iostream>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string odom_topic = "/odom";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    double imu_accel_scale = 10.0;
    // IMU body → base_link 外参 (导航模式: 补偿 TF/odom/body_cloud 到底盘中心)
    M3D r_ib = M3D::Identity();
    V3D t_ib = V3D::Zero();
    // 导航模式: publish_tf=false 时不发布 TF (交给 EKF)
    bool publish_tf = true;
    // 建图模式: publish_map_odom=true 时激活 PGO 回环修正 + 静止冻结
    bool publish_map_odom = false;
    std::string odom_frame = "odom";
    bool freeze_map_odom_when_static = true;
    double static_linear_thresh = 0.01;   // m/s
    double static_angular_thresh = 0.02;  // rad/s
    double static_hold_time = 0.2;        // s
    bool use_pointcloud2 = false;          // true: subscribe PointCloud2 (Gazebo), false: CustomMsg (real hw)
    bool body_cloud_self_filter_enabled = true;
    double body_cloud_self_filter_min_x = -0.45;
    double body_cloud_self_filter_max_x = 0.45;
    double body_cloud_self_filter_min_y = -0.35;
    double body_cloud_self_filter_max_y = 0.35;
    double body_cloud_self_filter_min_z = -0.25;
    double body_cloud_self_filter_max_z = 0.35;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "use_pointcloud2=%s, lidar_topic=%s",
                    m_node_config.use_pointcloud2 ? "true" : "false", m_node_config.lidar_topic.c_str());
        if (m_node_config.use_pointcloud2)
        {
            RCLCPP_INFO(this->get_logger(), "Subscribing to PointCloud2 on %s", m_node_config.lidar_topic.c_str());
            m_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(m_node_config.lidar_topic, 10, std::bind(&LIONode::pc2CB, this, std::placeholders::_1));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Subscribing to CustomMsg on %s", m_node_config.lidar_topic.c_str());
            m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));
        }

        // 建图模式: 订阅底盘 odom (静止检测) 和 PGO offset (回环修正)
        if (m_node_config.publish_map_odom)
        {
            m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                m_node_config.odom_topic, 50, std::bind(&LIONode::odomCB, this, std::placeholders::_1));
            m_pgo_offset_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/pgo/offset", 10,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(m_pgo_offset_mutex);
                    m_pgo_offset_r = Eigen::Quaterniond(
                        msg->pose.pose.orientation.w,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z).toRotationMatrix();
                    m_pgo_offset_t = Eigen::Vector3d(
                        msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
                    m_pgo_offset_dirty.store(true, std::memory_order_relaxed);
                });
            // 延迟创建 TF buffer，等 use_sim_time 时钟就绪后再订阅 /tf
            // 防止 buffer 被墙钟时间的 TF 消息污染导致 TF_OLD_DATA
            m_tf_init_timer = this->create_wall_timer(100ms, [this]() {
                if (this->get_clock()->ros_time_is_active()) {
                    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
                    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
                    m_tf_init_timer.reset();  // 取消定时器
                    RCLCPP_INFO(this->get_logger(), "TF buffer created (sim time active)");
                }
            });
            RCLCPP_INFO(this->get_logger(), "Mapping mode: PGO offset + static freeze enabled");
        }

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));

        // 建图模式: 50Hz 重发缓存的 map->odom TF
        if (m_node_config.publish_map_odom)
        {
            m_tf_timer = this->create_wall_timer(20ms, std::bind(&LIONode::tfTimerCB, this));
        }
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        if (config["use_pointcloud2"])
            m_node_config.use_pointcloud2 = config["use_pointcloud2"].as<bool>();
        if (config["odom_topic"])
            m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        if (config["imu_accel_scale"])
            m_node_config.imu_accel_scale = config["imu_accel_scale"].as<double>();
        if (config["publish_map_odom"])
            m_node_config.publish_map_odom = config["publish_map_odom"].as<bool>();
        if (config["publish_tf"])
            m_node_config.publish_tf = config["publish_tf"].as<bool>();
        if (config["odom_frame"])
            m_node_config.odom_frame = config["odom_frame"].as<std::string>();
        if (config["freeze_map_odom_when_static"])
            m_node_config.freeze_map_odom_when_static = config["freeze_map_odom_when_static"].as<bool>();
        if (config["static_linear_thresh"])
            m_node_config.static_linear_thresh = config["static_linear_thresh"].as<double>();
        if (config["static_angular_thresh"])
            m_node_config.static_angular_thresh = config["static_angular_thresh"].as<double>();
        if (config["static_hold_time"])
            m_node_config.static_hold_time = config["static_hold_time"].as<double>();
        if (config["body_cloud_self_filter_enabled"])
            m_node_config.body_cloud_self_filter_enabled = config["body_cloud_self_filter_enabled"].as<bool>();
        if (config["body_cloud_self_filter_min_x"])
            m_node_config.body_cloud_self_filter_min_x = config["body_cloud_self_filter_min_x"].as<double>();
        if (config["body_cloud_self_filter_max_x"])
            m_node_config.body_cloud_self_filter_max_x = config["body_cloud_self_filter_max_x"].as<double>();
        if (config["body_cloud_self_filter_min_y"])
            m_node_config.body_cloud_self_filter_min_y = config["body_cloud_self_filter_min_y"].as<double>();
        if (config["body_cloud_self_filter_max_y"])
            m_node_config.body_cloud_self_filter_max_y = config["body_cloud_self_filter_max_y"].as<double>();
        if (config["body_cloud_self_filter_min_z"])
            m_node_config.body_cloud_self_filter_min_z = config["body_cloud_self_filter_min_z"].as<double>();
        if (config["body_cloud_self_filter_max_z"])
            m_node_config.body_cloud_self_filter_max_z = config["body_cloud_self_filter_max_z"].as<double>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        // 加载 IMU body → base_link 外参 (可选，默认为零/单位矩阵)
        if (config["r_ib"] && config["t_ib"])
        {
            std::vector<double> r_ib_vec = config["r_ib"].as<std::vector<double>>();
            std::vector<double> t_ib_vec = config["t_ib"].as<std::vector<double>>();
            m_node_config.r_ib << r_ib_vec[0], r_ib_vec[1], r_ib_vec[2],
                                  r_ib_vec[3], r_ib_vec[4], r_ib_vec[5],
                                  r_ib_vec[6], r_ib_vec[7], r_ib_vec[8];
            m_node_config.t_ib << t_ib_vec[0], t_ib_vec[1], t_ib_vec[2];
            RCLCPP_INFO(this->get_logger(), "Loaded IMU->base extrinsic: t_ib = [%.3f, %.3f, %.3f]",
                        m_node_config.t_ib.x(), m_node_config.t_ib.y(), m_node_config.t_ib.z());
        }

        RCLCPP_INFO(
            this->get_logger(),
            "body_cloud self filter %s: x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]",
            m_node_config.body_cloud_self_filter_enabled ? "enabled" : "disabled",
            m_node_config.body_cloud_self_filter_min_x,
            m_node_config.body_cloud_self_filter_max_x,
            m_node_config.body_cloud_self_filter_min_y,
            m_node_config.body_cloud_self_filter_max_y,
            m_node_config.body_cloud_self_filter_min_z,
            m_node_config.body_cloud_self_filter_max_z);
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * m_node_config.imu_accel_scale,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        if (!m_lidar_received_once) {
            m_lidar_received_once = true;
            RCLCPP_INFO(this->get_logger(), "First CustomMsg received: point_num=%u, cloud_after_filter=%zu, stamp=%.3f",
                        msg->point_num, cloud->size(), Utils::getSec(msg->header));
        }
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    void pc2CB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::pc2ToPCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    // 底盘速度回调: 更新静止检测状态 (仅建图模式)
    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        const double wz = msg->twist.twist.angular.z;
        const double linear_speed = std::hypot(vx, vy);
        const double angular_speed = std::abs(wz);
        const double now_sec = this->get_clock()->now().seconds();
        const bool low_speed =
            linear_speed < m_node_config.static_linear_thresh &&
            angular_speed < m_node_config.static_angular_thresh;

        std::lock_guard<std::mutex> lock(m_motion_mutex);
        m_latest_linear_speed = linear_speed;
        m_latest_angular_speed = angular_speed;
        m_last_motion_msg_time = now_sec;
        if (low_speed) {
            if (m_stationary_since < 0.0) {
                m_stationary_since = now_sec;
            }
            m_robot_is_stationary = (now_sec - m_stationary_since) >= m_node_config.static_hold_time;
        } else {
            m_stationary_since = -1.0;
            m_robot_is_stationary = false;
        }
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            if (m_package.cloud->points.empty())
            {
                m_state_data.lidar_buffer.pop_front();
                return false;
            }
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    CloudType::Ptr buildYawLeveledWorldCloud(CloudType::Ptr cloud, const double &time)
    {
        const M3D &r_wl = m_builder->lidar_processor()->r_wl();
        const V3D &t_wl = m_builder->lidar_processor()->t_wl();

        if (!m_node_config.publish_map_odom || !m_tf_buffer)
        {
            const double yaw = std::atan2(r_wl(1, 0), r_wl(0, 0));
            const M3D r_wl_yaw = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
            return m_builder->lidar_processor()->transformCloud(cloud, r_wl_yaw, t_wl);
        }

        geometry_msgs::msg::TransformStamped odom_to_body;
        try {
            const auto query_stamp = Utils::getTime(time);
            const rclcpp::Time query_time(query_stamp, this->get_clock()->get_clock_type());
            odom_to_body = m_tf_buffer->lookupTransform(m_node_config.odom_frame, m_node_config.body_frame, query_time);
        } catch (const tf2::TransformException &ex) {
            const std::string ex_msg = ex.what();
            const bool future_extrapolation =
                ex_msg.find("future") != std::string::npos ||
                ex_msg.find("extrapolation") != std::string::npos;
            if (future_extrapolation) {
                try {
                    const rclcpp::Time latest_time(0, 0, this->get_clock()->get_clock_type());
                    odom_to_body = m_tf_buffer->lookupTransform(
                        m_node_config.odom_frame, m_node_config.body_frame, latest_time);
                } catch (const tf2::TransformException &) {
                    return m_builder->lidar_processor()->transformCloud(cloud, r_wl, t_wl);
                }
            } else {
                return m_builder->lidar_processor()->transformCloud(cloud, r_wl, t_wl);
            }
        }

        geometry_msgs::msg::TransformStamped map_to_odom;
        {
            std::lock_guard<std::mutex> lock(m_map_odom_mutex);
            if (!m_map_odom_valid)
                return m_builder->lidar_processor()->transformCloud(cloud, r_wl, t_wl);
            map_to_odom = m_map_to_odom;
        }

        Eigen::Quaterniond q_map_odom(
            map_to_odom.transform.rotation.w,
            map_to_odom.transform.rotation.x,
            map_to_odom.transform.rotation.y,
            map_to_odom.transform.rotation.z);
        V3D t_map_odom(
            map_to_odom.transform.translation.x,
            map_to_odom.transform.translation.y,
            map_to_odom.transform.translation.z);

        Eigen::Quaterniond q_odom_body(
            odom_to_body.transform.rotation.w,
            odom_to_body.transform.rotation.x,
            odom_to_body.transform.rotation.y,
            odom_to_body.transform.rotation.z);
        V3D t_odom_body(
            odom_to_body.transform.translation.x,
            odom_to_body.transform.translation.y,
            odom_to_body.transform.translation.z);

        const M3D r_map_body = (q_map_odom * q_odom_body).toRotationMatrix();
        const V3D t_map_body = t_map_odom + q_map_odom * t_odom_body;
        const M3D r_map_lidar = r_map_body * m_kf->x().r_il;
        const V3D t_map_lidar = t_map_body + r_map_body * m_kf->x().t_il;
        return m_builder->lidar_processor()->transformCloud(cloud, r_map_lidar, t_map_lidar);
    }

    CloudType::Ptr filterBodyCloud(CloudType::Ptr cloud) const
    {
        if (!cloud || !m_node_config.body_cloud_self_filter_enabled)
            return cloud;

        CloudType::Ptr filtered(new CloudType);
        filtered->reserve(cloud->size());
        for (const auto &point : cloud->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;

            const bool in_self_box =
                point.x >= m_node_config.body_cloud_self_filter_min_x &&
                point.x <= m_node_config.body_cloud_self_filter_max_x &&
                point.y >= m_node_config.body_cloud_self_filter_min_y &&
                point.y <= m_node_config.body_cloud_self_filter_max_y &&
                point.z >= m_node_config.body_cloud_self_filter_min_z &&
                point.z <= m_node_config.body_cloud_self_filter_max_z;
            if (!in_self_box)
                filtered->push_back(point);
        }

        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;
        return filtered;
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time,
                         const M3D &r_wb, const V3D &t_wb, const V3D &body_angular_vel = V3D::Zero())
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = t_wb.x();
        odom.pose.pose.position.y = t_wb.y();
        odom.pose.pose.position.z = t_wb.z();
        Eigen::Quaterniond q(r_wb);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度转换到 body 坐标系
        V3D vel = r_wb.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        odom.twist.twist.angular.x = body_angular_vel.x();
        odom.twist.twist.angular.y = body_angular_vel.y();
        odom.twist.twist.angular.z = body_angular_vel.z();

        // 协方差 (供 robot_localization EKF 使用)
        odom.pose.covariance[0]  = 0.01;   // x
        odom.pose.covariance[7]  = 0.01;   // y
        odom.pose.covariance[14] = 0.01;   // z
        odom.pose.covariance[21] = 0.001;  // roll
        odom.pose.covariance[28] = 0.001;  // pitch
        odom.pose.covariance[35] = 0.01;   // yaw
        odom.twist.covariance[0]  = 0.01;  // vx
        odom.twist.covariance[7]  = 0.01;  // vy
        odom.twist.covariance[14] = 0.01;  // vz
        odom.twist.covariance[35] = 0.01;  // vyaw

        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time,
                     const M3D &r_wb, const V3D &t_wb)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = t_wb.x();
        pose.pose.position.y = t_wb.y();
        pose.pose.position.z = t_wb.z();
        Eigen::Quaterniond q(r_wb);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time,
                     const M3D &r_wb, const V3D &t_wb)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(r_wb);
        transformStamped.transform.translation.x = t_wb.x();
        transformStamped.transform.translation.y = t_wb.y();
        transformStamped.transform.translation.z = t_wb.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    // 计算并缓存 map->odom TF (含 PGO 回环修正)
    void updateMapToOdom(const double &time)
    {
        Eigen::Matrix3d pgo_r;
        Eigen::Vector3d pgo_t;
        {
            std::lock_guard<std::mutex> lock(m_pgo_offset_mutex);
            pgo_r = m_pgo_offset_r;
            pgo_t = m_pgo_offset_t;
        }
        // T(map->body)_corrected = pgo_offset * T(map->body)_raw
        Eigen::Quaterniond q_map_body(pgo_r * m_kf->x().r_wi);
        V3D t_map_body = pgo_r * m_kf->x().t_wi + pgo_t;

        // 查询 LiDAR 帧结束时刻对应的 odom->body TF
        if (!m_tf_buffer) return;  // buffer 尚未创建 (等待 sim time 就绪)
        geometry_msgs::msg::TransformStamped odom_to_body;
        try {
            const auto query_stamp = Utils::getTime(time);
            const rclcpp::Time query_time(query_stamp, this->get_clock()->get_clock_type());
            odom_to_body = m_tf_buffer->lookupTransform(m_node_config.odom_frame, m_node_config.body_frame, query_time);
            m_map_odom_exact_time_ready = true;
        } catch (const tf2::TransformException &ex) {
            const std::string ex_msg = ex.what();
            const bool future_extrapolation =
                ex_msg.find("future") != std::string::npos ||
                ex_msg.find("extrapolation") != std::string::npos;
            if (future_extrapolation) {
                try {
                    const rclcpp::Time latest_time(0, 0, this->get_clock()->get_clock_type());
                    odom_to_body = m_tf_buffer->lookupTransform(
                        m_node_config.odom_frame, m_node_config.body_frame, latest_time);
                    if (!m_map_odom_exact_time_ready) {
                        RCLCPP_INFO_ONCE(this->get_logger(),
                            "Seeding map->%s with latest %s->%s TF before exact-time transforms are available",
                            m_node_config.odom_frame.c_str(), m_node_config.odom_frame.c_str(),
                            m_node_config.body_frame.c_str());
                    }
                } catch (const tf2::TransformException &) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "map->%s update skipped: cannot lookup %s->%s: %s",
                        m_node_config.odom_frame.c_str(), m_node_config.odom_frame.c_str(),
                        m_node_config.body_frame.c_str(), ex.what());
                    return;
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "map->%s update skipped: cannot lookup %s->%s: %s",
                    m_node_config.odom_frame.c_str(), m_node_config.odom_frame.c_str(),
                    m_node_config.body_frame.c_str(), ex.what());
                return;
            }
        }

        Eigen::Quaterniond q_odom_body(
            odom_to_body.transform.rotation.w,
            odom_to_body.transform.rotation.x,
            odom_to_body.transform.rotation.y,
            odom_to_body.transform.rotation.z);
        V3D t_odom_body(
            odom_to_body.transform.translation.x,
            odom_to_body.transform.translation.y,
            odom_to_body.transform.translation.z);

        // T(map->odom) = T(map->body)_corrected * T(odom->body)^-1
        Eigen::Quaterniond q_body_odom = q_odom_body.inverse();
        Eigen::Quaterniond q_map_odom = q_map_body * q_body_odom;
        V3D t_map_odom = t_map_body + q_map_body * (-(q_body_odom * t_odom_body));

        // 平面建图: Z/roll/pitch 只取首次值后冻结，避免 SLAM Z 漂移
        if (!m_map_odom_z_initialized) {
            m_map_odom_z_initialized = true;
            m_map_odom_z_fixed = t_map_odom.z();
        }
        t_map_odom.z() = m_map_odom_z_fixed;
        // 只保留 yaw, 冻结 roll/pitch
        Eigen::Matrix3d r_map_odom = q_map_odom.toRotationMatrix();
        double yaw = std::atan2(r_map_odom(1, 0), r_map_odom(0, 0));
        q_map_odom = Eigen::AngleAxisd(yaw, V3D::UnitZ());

        std::lock_guard<std::mutex> lock(m_map_odom_mutex);
        m_map_to_odom.header.frame_id = m_node_config.world_frame;
        m_map_to_odom.child_frame_id = m_node_config.odom_frame;
        m_map_to_odom.transform.translation.x = t_map_odom.x();
        m_map_to_odom.transform.translation.y = t_map_odom.y();
        m_map_to_odom.transform.translation.z = t_map_odom.z();
        m_map_to_odom.transform.rotation.x = q_map_odom.x();
        m_map_to_odom.transform.rotation.y = q_map_odom.y();
        m_map_to_odom.transform.rotation.z = q_map_odom.z();
        m_map_to_odom.transform.rotation.w = q_map_odom.w();
        m_map_odom_valid = true;
        m_pgo_offset_dirty.store(false, std::memory_order_relaxed);
    }

    // 重发缓存的 map->odom TF (只更新时间戳)
    void republishMapToOdom(const double &time)
    {
        std::lock_guard<std::mutex> lock(m_map_odom_mutex);
        if (!m_map_odom_valid) return;
        m_map_to_odom.header.stamp = Utils::getTime(time);
        m_tf_broadcaster->sendTransform(m_map_to_odom);
    }

    // 静止时冻结 map->odom 数值更新，避免 IMU 噪声引起微抖
    bool shouldFreezeMapToOdom()
    {
        if (!m_node_config.freeze_map_odom_when_static)
            return false;

        {
            std::lock_guard<std::mutex> lock(m_map_odom_mutex);
            if (!m_map_odom_valid) return false;
        }

        if (m_pgo_offset_dirty.load(std::memory_order_relaxed))
            return false;

        std::lock_guard<std::mutex> lock(m_motion_mutex);
        if (m_last_motion_msg_time < 0.0)
            return false;
        const double now_sec = this->get_clock()->now().seconds();
        if ((now_sec - m_last_motion_msg_time) > 0.5)
            return false;
        return m_robot_is_stationary;
    }

    void timerCB()
    {
        // 定期输出缓冲区状态，方便诊断
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Buffer: imu=%zu lidar=%zu, status=%d",
            m_state_data.imu_buffer.size(), m_state_data.lidar_buffer.size(),
            static_cast<int>(m_builder->status()));

        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        // 计算去偏后的 body-frame 角速度 (用于 odom twist.angular)
        if (!m_package.imus.empty()) {
            V3D gyro_corrected = m_package.imus.back().gyro - m_kf->x().bg;
            m_last_body_angular_vel = m_node_config.r_ib.transpose() * gyro_corrected;
        }

        if (m_node_config.publish_map_odom)
        {
            // === 建图模式: 只发布 map->odom TF (含 PGO 回环修正) ===
            // 完整 TF 链: map->odom (本节点) -> base_link (底盘控制器) -> mid360_link (URDF)
            // 不再直接发布 map->mid360_link，避免 mid360_link 多父节点 / TF 回路
            const M3D &r_wi = m_kf->x().r_wi;
            const V3D &t_wi = m_kf->x().t_wi;

            // 计算并发布 map->odom TF (含 PGO 回环修正 + 静止冻结)
            const bool freeze_map_odom = shouldFreezeMapToOdom();
            if (freeze_map_odom != m_freeze_map_odom_active) {
                m_freeze_map_odom_active = freeze_map_odom;
                if (freeze_map_odom) {
                    RCLCPP_INFO(this->get_logger(),
                        "Freeze map->odom updates while static (linear<%.3f m/s, angular<%.3f rad/s)",
                        m_node_config.static_linear_thresh, m_node_config.static_angular_thresh);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Resume map->odom updates");
                }
            }
            if (!freeze_map_odom) {
                updateMapToOdom(m_package.cloud_end_time);
            }

            publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time, r_wi, t_wi, m_last_body_angular_vel);

            // body_cloud: LiDAR→IMU body (无 r_ib/t_ib 补偿)
            CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
            body_cloud = filterBodyCloud(body_cloud);
            publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

            CloudType::Ptr world_cloud = buildYawLeveledWorldCloud(m_package.cloud, m_package.cloud_end_time);
            publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

            // Path: Z 锁定在初始值，避免 SLAM Z 漂移导致路径跳跃
            V3D t_wi_flat = t_wi;
            if (!m_path_z_initialized) {
                m_path_z_initialized = true;
                m_path_z_fixed = t_wi.z();
            }
            t_wi_flat.z() = m_path_z_fixed;
            publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time, r_wi, t_wi_flat);
        }
        else
        {
            // === 导航模式: 使用 r_ib/t_ib 补偿到底盘中心 ===
            const M3D &r_wi = m_kf->x().r_wi;
            const V3D &t_wi = m_kf->x().t_wi;
            const M3D &r_ib = m_node_config.r_ib;
            const V3D &t_ib = m_node_config.t_ib;
            M3D r_wb = r_wi * r_ib;
            V3D t_wb = r_wi * t_ib + t_wi;

            // Publish odom->base_link TF (补偿后的底盘中心位姿)
            if (m_node_config.publish_tf)
                broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time, r_wb, t_wb);

            publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time, r_wb, t_wb, m_last_body_angular_vel);

            // body_cloud: LiDAR→base_link = T_ib × T_il
            M3D r_bl = r_ib * m_kf->x().r_il;
            V3D t_bl = r_ib * m_kf->x().t_il + t_ib;
            CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, r_bl, t_bl);
            body_cloud = filterBodyCloud(body_cloud);
            publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

            CloudType::Ptr world_cloud = buildYawLeveledWorldCloud(m_package.cloud, m_package.cloud_end_time);
            publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

            publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time, r_wb, t_wb);
        }
    }

    // 50Hz 重发 map->odom TF (仅建图模式)
    void tfTimerCB()
    {
        double now = this->get_clock()->now().seconds();
        republishMapToOdom(now);
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc2_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_tf_timer;
    rclcpp::TimerBase::SharedPtr m_tf_init_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    // PGO 回环修正 offset (建图模式, 初始为单位变换)
    std::mutex m_pgo_offset_mutex;
    Eigen::Matrix3d m_pgo_offset_r{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d m_pgo_offset_t{Eigen::Vector3d::Zero()};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_pgo_offset_sub;
    std::atomic<bool> m_pgo_offset_dirty{false};

    // 缓存的 map->odom TF (建图模式)
    std::mutex m_map_odom_mutex;
    geometry_msgs::msg::TransformStamped m_map_to_odom;
    bool m_map_odom_valid{false};
    bool m_map_odom_exact_time_ready{false};
    bool m_map_odom_z_initialized{false};
    double m_map_odom_z_fixed{0.0};
    bool m_path_z_initialized{false};
    double m_path_z_fixed{0.0};

    // 静止检测 (建图模式)
    mutable std::mutex m_motion_mutex;
    double m_latest_linear_speed{0.0};
    double m_latest_angular_speed{0.0};
    double m_last_motion_msg_time{-1.0};
    double m_stationary_since{-1.0};
    bool m_robot_is_stationary{false};
    bool m_freeze_map_odom_active{false};
    bool m_lidar_received_once{false};

    // 最近一次去偏后的 body-frame 角速度 (用于 odom twist.angular)
    V3D m_last_body_angular_vel{V3D::Zero()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}
