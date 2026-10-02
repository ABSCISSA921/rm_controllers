#pragma once

#include <Eigen/Core>

namespace rm_chassis_controllers
{
// Normal-ground [IMU heading velocity, its derivative] estimator.
// Adapts the original velocity/acceleration KF idea without changing K10's wheel-midpoint V.
class VelocityKalmanFilter
{
public:
  struct Params
  {
    double velocity_process_density{0.02}; // m^2/s^3
    double jerk_density{10.}; // m^2/s^5
    double velocity_stddev{0.03}; // m/s
    double acceleration_stddev{0.5}; // m/s^2
    double velocity_filter_tau{0.002}, acceleration_filter_tau{0.001}; // s
    double velocity_residual_threshold{0.12}; // m/s, before measurement assimilation
    double velocity_noise_multiplier{25.}; // Multiplies variance, not standard deviation.
  };

  struct Input
  {
    Eigen::Matrix3d rotation_world_base;
    double yaw, yaw_rate;
    Eigen::Vector3d specific_force_base, angular_velocity_base;
    Eigen::Vector3d imu_position_base;
    // Wheel midpoint relative to base_link, supplied by the caller's geometry.
    Eigen::Vector3d wheel_midpoint_base, wheel_midpoint_rate_base;
    double kinematic_velocity, dt;
  };

  struct Output
  {
    double velocity; // Signed wheel-midpoint speed along current horizontal heading.
  };

  bool configure(const Params &params);
  void reset();
  bool update(const Input &input, Output &output);

private:
  Params params_;
  bool initialized_{false};
  Eigen::Vector2d state_{Eigen::Vector2d::Zero()}, observation_{Eigen::Vector2d::Zero()};
  double point_velocity_filtered_{0.};
  Eigen::Matrix2d covariance_{Eigen::Matrix2d::Zero()};
};
} // namespace rm_chassis_controllers
