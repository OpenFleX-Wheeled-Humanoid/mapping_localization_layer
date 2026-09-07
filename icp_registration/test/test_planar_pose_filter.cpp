#include <gtest/gtest.h>

#include "icp_registration/planar_pose_filter.hpp"

namespace icp {

TEST(PlanarPoseFilter, FirstMeasurementIsAcceptedWithoutLag) {
  PlanarPoseFilter filter(0.25);
  const auto result = filter.update({2.0, -1.0, 1.2});
  EXPECT_DOUBLE_EQ(result.x, 2.0);
  EXPECT_DOUBLE_EQ(result.y, -1.0);
  EXPECT_DOUBLE_EQ(result.yaw, 1.2);
}

TEST(PlanarPoseFilter, SmoothsTranslation) {
  PlanarPoseFilter filter(0.25);
  filter.reset({0.0, 0.0, 0.0});
  const auto result = filter.update({1.0, -2.0, 0.0});
  EXPECT_DOUBLE_EQ(result.x, 0.25);
  EXPECT_DOUBLE_EQ(result.y, -0.5);
}

TEST(PlanarPoseFilter, UsesShortestYawPathAcrossPi) {
  PlanarPoseFilter filter(0.5);
  filter.reset({0.0, 0.0, 179.0 * M_PI / 180.0});
  const auto result = filter.update({0.0, 0.0, -179.0 * M_PI / 180.0});
  EXPECT_NEAR(result.yaw, 180.0 * M_PI / 180.0, 1e-9);
}

}  // namespace icp
