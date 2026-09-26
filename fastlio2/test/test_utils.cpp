#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <std_msgs/msg/header.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>

#include "utils.h"

namespace {

livox_ros_driver2::msg::CustomPoint makePoint(
    float x, float y, float z, uint8_t reflectivity, uint8_t tag, uint8_t line, uint32_t offset_time)
{
    livox_ros_driver2::msg::CustomPoint p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.reflectivity = reflectivity;
    p.tag = tag;
    p.line = line;
    p.offset_time = offset_time;
    return p;
}

}  // namespace

TEST(UtilsTest, TimeRoundTrip)
{
    const double input_sec = 123.456789;
    const auto time_msg = Utils::getTime(input_sec);
    std_msgs::msg::Header header;
    header.stamp = time_msg;

    const double output_sec = Utils::getSec(header);
    EXPECT_NEAR(output_sec, input_sec, 1e-6);
}

TEST(UtilsTest, LivoxToPCLFiltersInvalidPoints)
{
    auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    msg->point_num = 6;
    msg->points = {
        makePoint(1.0f, 0.0f, 0.0f, 10, 0x10, 0, 100000),    // valid
        makePoint(1.0f, 0.0f, 0.0f, 20, 0x10, 4, 200000),    // valid (line 4 allowed)
        makePoint(1.0f, 0.0f, 0.0f, 30, 0x20, 0, 300000),    // invalid tag
        makePoint(0.01f, 0.0f, 0.0f, 40, 0x10, 0, 400000),   // too near
        makePoint(25.0f, 0.0f, 0.0f, 50, 0x10, 0, 500000),   // too far
        makePoint(2.0f, 0.0f, 0.0f, 60, 0x00, 2, 600000),    // valid
    };

    auto cloud = Utils::livox2PCL(msg, 1, 0.1, 20.0);
    ASSERT_NE(cloud, nullptr);
    ASSERT_EQ(cloud->size(), 3u);

    EXPECT_FLOAT_EQ(cloud->points[0].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud->points[0].intensity, 10.0f);
    EXPECT_NEAR(cloud->points[0].curvature, 0.1f, 1e-6f);

    EXPECT_FLOAT_EQ(cloud->points[1].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud->points[1].intensity, 20.0f);
    EXPECT_NEAR(cloud->points[1].curvature, 0.2f, 1e-6f);

    EXPECT_FLOAT_EQ(cloud->points[2].x, 2.0f);
    EXPECT_FLOAT_EQ(cloud->points[2].intensity, 60.0f);
    EXPECT_NEAR(cloud->points[2].curvature, 0.6f, 1e-6f);
}

TEST(UtilsTest, LivoxToPCLSupportsDownsampleByFilterNum)
{
    auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    msg->point_num = 4;
    msg->points = {
        makePoint(1.0f, 0.0f, 0.0f, 10, 0x10, 0, 100000),  // selected
        makePoint(2.0f, 0.0f, 0.0f, 20, 0x10, 0, 200000),  // skipped by filter_num
        makePoint(3.0f, 0.0f, 0.0f, 30, 0x10, 0, 300000),  // selected
        makePoint(4.0f, 0.0f, 0.0f, 40, 0x10, 0, 400000),  // skipped by filter_num
    };

    auto cloud = Utils::livox2PCL(msg, 2, 0.1, 20.0);
    ASSERT_NE(cloud, nullptr);
    ASSERT_EQ(cloud->size(), 2u);
    EXPECT_FLOAT_EQ(cloud->points[0].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud->points[1].x, 3.0f);
}
