#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

class PCDMapPublisher : public rclcpp::Node
{
public:
    PCDMapPublisher() : Node("pcd_map_publisher")
    {
        this->declare_parameter<std::string>("pcd_path", "");
        this->declare_parameter<std::string>("topic_name", "/pgo/saved_map_cloud");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<double>("publish_hz", 1.0);
        this->declare_parameter<double>("voxel_size", 0.0);

        this->get_parameter("pcd_path", m_pcd_path);
        this->get_parameter("topic_name", m_topic_name);
        this->get_parameter("frame_id", m_frame_id);
        this->get_parameter("publish_hz", m_publish_hz);
        this->get_parameter("voxel_size", m_voxel_size);

        if (m_pcd_path.empty())
        {
            throw std::runtime_error("Parameter 'pcd_path' is empty");
        }
        if (!std::filesystem::exists(m_pcd_path))
        {
            throw std::runtime_error("PCD file does not exist: " + m_pcd_path);
        }

        m_cloud = std::make_shared<CloudType>();
        if (!loadCloudFromFile())
        {
            throw std::runtime_error("Failed to load PCD file: " + m_pcd_path);
        }
        if (m_voxel_size > 0.0)
        {
            applyVoxelFilter();
        }
        if (m_cloud->empty())
        {
            throw std::runtime_error("Loaded point cloud is empty after filtering");
        }

        printCloudStats();

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();
        m_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_topic_name, qos);

        publishCloud();

        const double hz = (m_publish_hz <= 0.0) ? 1.0 : m_publish_hz;
        const auto period = std::chrono::duration<double>(1.0 / hz);
        m_timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&PCDMapPublisher::publishCloud, this));

        RCLCPP_INFO(this->get_logger(),
                    "Publishing '%s' in frame '%s' to topic '%s' at %.2f Hz",
                    m_pcd_path.c_str(),
                    m_frame_id.c_str(),
                    m_topic_name.c_str(),
                    hz);
    }

private:
    bool loadCloudFromFile()
    {
        if (pcl::io::loadPCDFile<PointType>(m_pcd_path, *m_cloud) == 0)
        {
            return removeNanPoints();
        }

        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(m_pcd_path, cloud_xyz) != 0)
        {
            return false;
        }

        m_cloud->clear();
        m_cloud->reserve(cloud_xyz.size());
        for (const auto &point : cloud_xyz.points)
        {
            PointType p{};
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.intensity = 1.0f;
            m_cloud->push_back(p);
        }
        m_cloud->width = static_cast<uint32_t>(m_cloud->size());
        m_cloud->height = 1;
        m_cloud->is_dense = false;
        return removeNanPoints();
    }

    bool removeNanPoints()
    {
        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, valid_indices);
        return true;
    }

    void applyVoxelFilter()
    {
        CloudType::Ptr downsampled(new CloudType);
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(m_voxel_size, m_voxel_size, m_voxel_size);
        voxel.setInputCloud(m_cloud);
        voxel.filter(*downsampled);
        m_cloud = downsampled;
    }

    void printCloudStats() const
    {
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();

        for (const auto &p : m_cloud->points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            {
                continue;
            }
            min_x = std::min(min_x, p.x);
            min_y = std::min(min_y, p.y);
            min_z = std::min(min_z, p.z);
            max_x = std::max(max_x, p.x);
            max_y = std::max(max_y, p.y);
            max_z = std::max(max_z, p.z);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Cloud points: %zu, bounds x:[%.2f, %.2f] y:[%.2f, %.2f] z:[%.2f, %.2f]",
                    m_cloud->size(),
                    min_x, max_x,
                    min_y, max_y,
                    min_z, max_z);
    }

    void publishCloud()
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*m_cloud, msg);
        msg.header.frame_id = m_frame_id;
        msg.header.stamp = this->now();
        m_pub->publish(msg);
    }

    std::string m_pcd_path;
    std::string m_topic_name;
    std::string m_frame_id;
    double m_publish_hz = 1.0;
    double m_voxel_size = 0.0;

    CloudType::Ptr m_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int ret_code = 0;
    try
    {
        rclcpp::spin(std::make_shared<PCDMapPublisher>());
    }
    catch (const std::exception &e)
    {
        std::cerr << "[pcd_map_publisher] " << e.what() << std::endl;
        ret_code = 1;
    }
    rclcpp::shutdown();
    return ret_code;
}
