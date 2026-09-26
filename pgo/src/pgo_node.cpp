#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include <pcl/io/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string map_frame = "map";
    std::string body_frame = "mid360_link";
    double map_z_min = -0.1;
    double map_z_max = 3.0;
    double global_map_resolution = 0.1;
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time = -1.0;
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        loadParameters();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        m_global_cloud.reset(new CloudType);
        auto cloud_qos = rclcpp::SensorDataQoS();
        rclcpp::QoS odom_qos = rclcpp::QoS(10);
        rclcpp::QoS map_qos(1);
        map_qos.transient_local();
        map_qos.reliable();
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, cloud_qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, odom_qos.get_rmw_qos_profile());
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10000);
        m_global_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pgo/global_cloud", map_qos);
        m_offset_pub = this->create_publisher<nav_msgs::msg::Odometry>("/pgo/offset", 10);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("/pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));
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
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.body_frame = config["local_frame"].as<std::string>();

        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();

        // ScanContext parameters (with defaults)
        if (config["sc_dist_thresh"])
            m_pgo_config.sc_dist_thresh = config["sc_dist_thresh"].as<double>();
        if (config["sc_max_radius"])
            m_pgo_config.sc_max_radius = config["sc_max_radius"].as<double>();
        if (config["sc_num_exclude_recent"])
            m_pgo_config.sc_num_exclude_recent = config["sc_num_exclude_recent"].as<int>();
        if (config["sc_num_candidates"])
            m_pgo_config.sc_num_candidates = config["sc_num_candidates"].as<int>();

        // NDT parameters (with defaults)
        if (config["ndt_resolution"])
            m_pgo_config.ndt_resolution = config["ndt_resolution"].as<double>();
        if (config["ndt_step_size"])
            m_pgo_config.ndt_step_size = config["ndt_step_size"].as<double>();
        if (config["ndt_max_iterations"])
            m_pgo_config.ndt_max_iterations = config["ndt_max_iterations"].as<int>();
        if (config["ndt_epsilon"])
            m_pgo_config.ndt_epsilon = config["ndt_epsilon"].as<double>();
        if (config["ndt_score_thresh"])
            m_pgo_config.ndt_score_thresh = config["ndt_score_thresh"].as<double>();

        RCLCPP_INFO(this->get_logger(), "SC params: dist_thresh=%.2f, max_radius=%.1f, exclude_recent=%d, candidates=%d",
                    m_pgo_config.sc_dist_thresh, m_pgo_config.sc_max_radius,
                    m_pgo_config.sc_num_exclude_recent, m_pgo_config.sc_num_candidates);
        RCLCPP_INFO(this->get_logger(), "NDT params: resolution=%.2f, step=%.2f, max_iter=%d, epsilon=%.4f, score_thresh=%.2f",
                    m_pgo_config.ndt_resolution, m_pgo_config.ndt_step_size,
                    m_pgo_config.ndt_max_iterations, m_pgo_config.ndt_epsilon,
                    m_pgo_config.ndt_score_thresh);

        if (config["map_z_min"])
            m_node_config.map_z_min = config["map_z_min"].as<double>();
        if (config["map_z_max"])
            m_node_config.map_z_max = config["map_z_max"].as<double>();
        m_node_config.global_map_resolution = m_pgo_config.submap_resolution;
        if (config["global_map_resolution"])
            m_node_config.global_map_resolution = config["global_map_resolution"].as<double>();
        RCLCPP_INFO(this->get_logger(), "Map Z filter: [%.2f, %.2f]",
                    m_node_config.map_z_min, m_node_config.map_z_max);
        RCLCPP_INFO(this->get_logger(), "Global map resolution: %.2f", m_node_config.global_map_resolution);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }
        m_state.last_message_time = cp.pose.second;

        cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                       odom_msg->pose.pose.orientation.x,
                                       odom_msg->pose.pose.orientation.y,
                                       odom_msg->pose.pose.orientation.z)
                        .toRotationMatrix();
        cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        cp.cloud = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *cp.cloud);
        m_state.cloud_buffer.push(cp);
    }

    void publishOffset(builtin_interfaces::msg::Time &time)
    {
        // 将 pgo 的回环修正 offset 发布给 lio_node，由其叠加到高频 TF 中
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = time;
        msg.header.frame_id = m_node_config.map_frame;
        msg.pose.pose.position.x = t.x();
        msg.pose.pose.position.y = t.y();
        msg.pose.pose.position.z = t.z();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        m_offset_pub->publish(msg);
    }

    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->historyPairs().size() == 0)
            return;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();
        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    CloudType::Ptr buildFilteredWorldCloud(const KeyPoseWithCloud &key_pose)
    {
        CloudType::Ptr world_cloud(new CloudType);
        pcl::transformPointCloud(*key_pose.body_cloud, *world_cloud, key_pose.t_global, Eigen::Quaterniond(key_pose.r_global));

        CloudType::Ptr filtered(new CloudType);
        filtered->reserve(world_cloud->size());
        for (const auto &pt : world_cloud->points)
        {
            if (pt.z >= m_node_config.map_z_min && pt.z <= m_node_config.map_z_max)
                filtered->push_back(pt);
        }
        return filtered;
    }

    void downsampleGlobalCloud()
    {
        if (!m_global_cloud || m_global_cloud->empty() || m_node_config.global_map_resolution <= 0.0)
            return;

        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(m_node_config.global_map_resolution,
                               m_node_config.global_map_resolution,
                               m_node_config.global_map_resolution);
        voxel_grid.setInputCloud(m_global_cloud);
        CloudType::Ptr filtered(new CloudType);
        voxel_grid.filter(*filtered);
        m_global_cloud = filtered;
    }

    void rebuildGlobalCloud()
    {
        m_global_cloud.reset(new CloudType);
        for (const auto &key_pose : m_pgo->keyPoses())
        {
            CloudType::Ptr filtered = buildFilteredWorldCloud(key_pose);
            *m_global_cloud += *filtered;
        }
        downsampleGlobalCloud();
        m_global_cloud_keypose_count = m_pgo->keyPoses().size();
    }

    void appendNewKeyPosesToGlobalCloud()
    {
        if (!m_global_cloud)
            m_global_cloud.reset(new CloudType);

        const auto &key_poses = m_pgo->keyPoses();
        if (m_global_cloud_keypose_count > key_poses.size())
        {
            rebuildGlobalCloud();
            return;
        }

        for (size_t i = m_global_cloud_keypose_count; i < key_poses.size(); ++i)
        {
            CloudType::Ptr filtered = buildFilteredWorldCloud(key_poses[i]);
            *m_global_cloud += *filtered;
        }
        downsampleGlobalCloud();
        m_global_cloud_keypose_count = key_poses.size();
    }

    void publishGlobalCloud(builtin_interfaces::msg::Time &time)
    {
        if (!m_global_cloud || m_global_cloud->empty())
            return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*m_global_cloud, cloud_msg);
        cloud_msg.header.frame_id = m_node_config.map_frame;
        cloud_msg.header.stamp = time;
        m_global_cloud_pub->publish(cloud_msg);
    }

    void timerCB()
    {
        CloudWithPose cp;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (m_state.cloud_buffer.empty())
                return;
            cp = m_state.cloud_buffer.back();
            while (!m_state.cloud_buffer.empty())
            {
                m_state.cloud_buffer.pop();
            }
        }
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        if (!m_pgo->addKeyPose(cp))
            return;

        m_pgo->searchForLoopPairs();
        const bool rebuild_global_cloud = m_pgo->hasLoop() || m_global_cloud_keypose_count > m_pgo->keyPoses().size();
        m_pgo->smoothAndUpdate();

        if (rebuild_global_cloud)
            rebuildGlobalCloud();
        else
            appendNewKeyPosesToGlobalCloud();

        publishGlobalCloud(cur_time);

        // 仅在回环优化后发一次修正 TF，覆盖 lio_node 的累积误差
        publishOffset(cur_time);

        publishLoopMarkers(cur_time);
    }

    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        std::ofstream txt_file(poses_txt_path);

        CloudType::Ptr ret(new CloudType);
        size_t total_before = 0;
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {

            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            }
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global, Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            total_before += world_cloud->size();

            // Z 轴裁剪：去除世界坐标系下地面以下和天花板以上的噪声点
            CloudType::Ptr filtered(new CloudType);
            filtered->reserve(world_cloud->size());
            for (const auto &pt : world_cloud->points)
            {
                if (pt.z >= m_node_config.map_z_min && pt.z <= m_node_config.map_z_max)
                    filtered->push_back(pt);
            }
            *ret += *filtered;
        }
        txt_file.close();

        RCLCPP_INFO(this->get_logger(), "Z filter [%.2f, %.2f]: %zu -> %zu points (removed %zu)",
                    m_node_config.map_z_min, m_node_config.map_z_max,
                    total_before, ret->size(), total_before - ret->size());
        pcl::io::savePCDFileBinary(map_path.string(), *ret);

        // Save ScanContext database
        std::filesystem::path sc_dir = p_dir / "sc_data";
        if (m_pgo->saveScanContextDatabase(sc_dir.string()))
        {
            RCLCPP_INFO(this->get_logger(), "ScanContext database saved to %s", sc_dir.string().c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to save ScanContext database");
        }

        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_offset_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    CloudType::Ptr m_global_cloud;
    size_t m_global_cloud_keypose_count{0};
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}
