#include "bipedal_wheel_controller/base_task.h"

#include <cmath>

namespace rm_chassis_controllers
{
BaseTaskObservation observeBaseTask(const ChassisState& state, const Eigen::Matrix3d& rotation_world_base,
                                    const Eigen::Vector3d& hip_midpoint_base, double wheel_radius)
{
  BaseTaskObservation out;
  out.wheel_midpoint_base = hip_midpoint_base;
  for (int side = 0; side < 2; ++side)
  {
    const double angle = state.x[THETA_L + 2 * side] - state.pitch;
    const double rate = state.x[DTHETA_L + 2 * side] - state.x[BODY_RATE];
    const double length = state.length[side], length_rate = state.dlength[side];
    out.wheel_midpoint_base += .5 * Eigen::Vector3d(-length * std::sin(angle), 0., -length * std::cos(angle));
    out.wheel_midpoint_rate_base +=
        .5 * Eigen::Vector3d(-length_rate * std::sin(angle) - length * std::cos(angle) * rate, 0.,
                             -length_rate * std::cos(angle) + length * std::sin(angle) * rate);
  }
  const Eigen::Vector3d omega(state.angular_vel.x, state.angular_vel.y, state.angular_vel.z);
  const Eigen::Vector3d forward(std::cos(state.yaw), std::sin(state.yaw), 0.);
  const Eigen::Vector3d point_velocity =
      rotation_world_base * (omega.cross(out.wheel_midpoint_base) + out.wheel_midpoint_rate_base);
  const Eigen::Vector3d axle_world = rotation_world_base.col(1);
  const double radius_projection = std::sqrt(1. - axle_world.z() * axle_world.z());
  const double wheel_height = wheel_radius * radius_projection;
  const Eigen::Vector3d contact_radius =
      -wheel_radius * (Eigen::Vector3d::UnitZ() - axle_world.z() * axle_world) / radius_projection;
  const Eigen::Vector3d rolling_direction = axle_world.cross(Eigen::Vector3d::UnitZ()).normalized();
  const Eigen::Vector3d lean_velocity = -(rotation_world_base * omega).cross(contact_radius);
  // Axial spin contributes only along rolling_direction. Solve that component
  // from the existing heading wheel speed, retaining the lean-induced y/z speed.
  out.wheel_velocity_world = lean_velocity + rolling_direction *
      ((state.x[V] - forward.dot(lean_velocity)) / forward.dot(rolling_direction));
  out.velocity_world = out.wheel_velocity_world - point_velocity;
  out.height = wheel_height - (rotation_world_base * out.wheel_midpoint_base).z();
  out.height_rate = out.velocity_world.z();
  out.forward_velocity = forward.dot(out.velocity_world);
  return out;
}

StateCost10 baseTaskOutputMatrix(const Eigen::Vector2d& lengths, const State10& trim,
                                 const Eigen::Vector3d& hip_midpoint_base)
{
  StateCost10 output = StateCost10::Identity();
  for (int side = 0; side < 2; ++side)
  {
    const double derivative = .5 * lengths[side] * std::cos(trim[THETA_L + 2 * side]);
    output(S, THETA_L + 2 * side) = derivative;
    output(V, DTHETA_L + 2 * side) = derivative;
  }
  const double hip_derivative =
      std::sin(trim[BODY_PITCH]) * hip_midpoint_base.x() - std::cos(trim[BODY_PITCH]) * hip_midpoint_base.z();
  output(S, BODY_PITCH) = hip_derivative;
  output(V, BODY_RATE) = hip_derivative;
  return output;
}

StateCost10 baseTaskCost(const Eigen::Vector2d& lengths, const State10& trim, const Eigen::Vector3d& hip_midpoint_base,
                         const State10& output_weights, const State10& internal_weights)
{
  const StateCost10 output = baseTaskOutputMatrix(lengths, trim, hip_midpoint_base);
  StateCost10 cost = output.transpose() * output_weights.asDiagonal() * output;
  cost.diagonal() += internal_weights;
  return cost;
}
}  // namespace rm_chassis_controllers
