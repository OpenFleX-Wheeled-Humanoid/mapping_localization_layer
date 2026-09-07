#ifndef ICP_REGISTRATION_PLANAR_POSE_FILTER_HPP
#define ICP_REGISTRATION_PLANAR_POSE_FILTER_HPP

#include <algorithm>
#include <cmath>

namespace icp {

struct PlanarPose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

inline double normalizePlanarAngle(double angle) {
  constexpr double kTwoPi = 2.0 * M_PI;
  while (angle > M_PI) angle -= kTwoPi;
  while (angle < -M_PI) angle += kTwoPi;
  return angle;
}

// Filters the slowly varying map->odom correction, not robot motion itself.
class PlanarPoseFilter {
public:
  explicit PlanarPoseFilter(double alpha = 0.35) : alpha_(std::clamp(alpha, 0.0, 1.0)) {}

  void reset(const PlanarPose &pose) {
    state_ = pose;
    state_.yaw = normalizePlanarAngle(state_.yaw);
    initialized_ = true;
  }

  bool initialized() const { return initialized_; }

  PlanarPose update(const PlanarPose &measurement) {
    if (!initialized_) {
      reset(measurement);
      return state_;
    }

    state_.x += alpha_ * (measurement.x - state_.x);
    state_.y += alpha_ * (measurement.y - state_.y);
    const double yaw_error = normalizePlanarAngle(measurement.yaw - state_.yaw);
    state_.yaw = normalizePlanarAngle(state_.yaw + alpha_ * yaw_error);
    return state_;
  }

  const PlanarPose &state() const { return state_; }

private:
  double alpha_;
  bool initialized_ = false;
  PlanarPose state_;
};

}  // namespace icp

#endif  // ICP_REGISTRATION_PLANAR_POSE_FILTER_HPP
