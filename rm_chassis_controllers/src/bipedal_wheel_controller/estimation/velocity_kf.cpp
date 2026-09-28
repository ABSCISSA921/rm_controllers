#include "bipedal_wheel_controller/estimation/velocity_kf.h"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace rm_chassis_controllers
{
bool VelocityKalmanFilter::configure(const Params &params)
{
  for (double value : {params.velocity_process_density, params.jerk_density, params.velocity_stddev,
                       params.acceleration_stddev, params.velocity_filter_tau, params.acceleration_filter_tau,
                       params.velocity_residual_threshold, params.velocity_noise_multiplier})
    if (!std::isfinite(value) || value <= 0.)
      return false;
  if (params.velocity_noise_multiplier < 1.)
    return false;
  params_ = params;
  reset();
  return true;
}

void VelocityKalmanFilter::reset()
{
  initialized_ = false;
}

bool VelocityKalmanFilter::update(const Input &input, Output &output)
{
  if (!std::isfinite(input.dt) || input.dt <= 0.)
    return false;

  const Eigen::Vector3d point_velocity = input.rotation_world_base *
      (input.angular_velocity_base.cross(input.imu_position_base - input.wheel_midpoint_base) -
       input.wheel_midpoint_rate_base);
  const Eigen::Vector3d forward(std::cos(input.yaw), std::sin(input.yaw), 0.);
  const Eigen::Vector3d lateral(-std::sin(input.yaw), std::cos(input.yaw), 0.);
  const Eigen::Vector3d acceleration = input.rotation_world_base * input.specific_force_base +
                                      Eigen::Vector3d(0., 0., -9.81);
  const double point_forward = forward.dot(point_velocity), point_lateral = lateral.dot(point_velocity);
  // d(t dot v_I)/dt = t dot a_I + yaw_rate * (n dot v_I).
  // Ordinary-ground approximation: wheel-midpoint lateral velocity is zero, so n dot v_I = n dot c.
  const double turning_acceleration = input.yaw_rate * point_lateral;
  const Eigen::Vector2d raw(input.kinematic_velocity + point_forward,
                            forward.dot(acceleration) + turning_acceleration);
  if (!raw.allFinite() || !acceleration.allFinite() || !std::isfinite(point_forward))
    return false;

  Eigen::Vector2d observation = raw;
  double point_filtered = point_forward;
  if (initialized_)
  {
    const double beta_v = -std::expm1(-input.dt / params_.velocity_filter_tau);
    const double beta_a = -std::expm1(-input.dt / params_.acceleration_filter_tau);
    observation[0] = observation_[0] + beta_v * (raw[0] - observation_[0]);
    observation[1] = observation_[1] + beta_a * (raw[1] - observation_[1]);
    // The same causal filter and seed as z_v: never subtract an unfiltered alternating correction.
    point_filtered = point_velocity_filtered_ + beta_v * (point_forward - point_velocity_filtered_);
  }
  Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Zero();
  measurement_covariance.diagonal() << std::pow(params_.velocity_stddev, 2),
                                      std::pow(params_.acceleration_stddev, 2);
  Eigen::Vector2d state = observation, predicted = observation;
  Eigen::Vector2d innovation = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix2d covariance = measurement_covariance, gain = Eigen::Matrix2d::Zero();
  bool deweighted = false;
  double velocity_residual = std::numeric_limits<double>::quiet_NaN();
  if (initialized_)
  {
    const double dt = input.dt;
    Eigen::Matrix2d transition;
    transition << 1., dt, 0., 1.;
    Eigen::Matrix2d process_covariance;
    process_covariance << dt * dt * dt / 3., dt * dt / 2., dt * dt / 2., dt;
    process_covariance *= params_.jerk_density;
    process_covariance(0, 0) += params_.velocity_process_density * dt;
    predicted = transition * state_;
    const Eigen::Matrix2d predicted_covariance = transition * covariance_ * transition.transpose() + process_covariance;
    innovation = observation - predicted;
    velocity_residual = innovation[0];
    deweighted = std::abs(velocity_residual) > params_.velocity_residual_threshold;
    if (deweighted)
      measurement_covariance(0, 0) *= params_.velocity_noise_multiplier;
    const Eigen::Matrix2d innovation_covariance = predicted_covariance + measurement_covariance;
    const Eigen::LDLT<Eigen::Matrix2d> factor(innovation_covariance);
    if (factor.info() != Eigen::Success || !factor.isPositive())
      return false;
    gain = factor.solve(predicted_covariance.transpose()).transpose();
    state = predicted + gain * innovation;
    const Eigen::Matrix2d residual = Eigen::Matrix2d::Identity() - gain;
    covariance = residual * predicted_covariance * residual.transpose() +
                 gain * measurement_covariance * gain.transpose();
  }
  const double velocity = state[0] - point_filtered;
  if (!state.allFinite() || !covariance.allFinite() || !std::isfinite(velocity))
    return false;

  output.velocity = velocity;
  output.observation_raw = raw;
  output.observation = observation;
  output.predicted = predicted;
  if (!initialized_)
    output.predicted.setConstant(std::numeric_limits<double>::quiet_NaN()); // No prediction on the seed frame.
  output.state = state;
  output.innovation = innovation;
  output.acceleration_world = acceleration;
  output.point_velocity_raw = point_forward;
  output.point_velocity_filtered = point_filtered;
  output.point_velocity_lateral = point_lateral;
  output.turning_acceleration = turning_acceleration;
  output.velocity_variance = measurement_covariance(0, 0);
  output.velocity_gain = gain(0, 0);
  output.acceleration_velocity_gain = gain(1, 0);
  output.velocity_residual = velocity_residual;
  output.seeded = !initialized_;
  output.velocity_deweighted = deweighted;
  state_ = state;
  covariance_ = covariance;
  observation_ = observation;
  point_velocity_filtered_ = point_filtered;
  initialized_ = true;
  return true;
}
} // namespace rm_chassis_controllers
