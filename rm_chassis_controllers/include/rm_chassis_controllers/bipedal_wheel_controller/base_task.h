#pragma once

#include "bipedal_wheel_controller/definitions.h"
#include <Eigen/Geometry>

namespace rm_chassis_controllers
{
using StateCost10 = Eigen::Matrix<double, 10, 10>;

// Geometry uses the physical VMC chord, not the equivalent mass-distribution angle.
struct BaseTaskObservation
{
  Eigen::Vector3d wheel_midpoint_base{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d wheel_midpoint_rate_base{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d wheel_velocity_world{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d velocity_world{ Eigen::Vector3d::Zero() };
  double height{ 0. }, height_rate{ 0. }, forward_velocity{ 0. };
};

// Flat ground, rolling thin discs and sagittal legs. Wheel centres move laterally
// as the axles lean; zero lateral contact speed does not mean zero centre speed.
BaseTaskObservation observeBaseTask(const ChassisState& state, const Eigen::Matrix3d& rotation_world_base,
                                    const Eigen::Vector3d& hip_midpoint_base, double wheel_radius);

// Linear output map at a stationary, zero-roll trim with frozen leg lengths.
// Rows S,V denote base origin position/speed; the other eight coordinates retain
// their existing definitions. Stretch-rate terms are handled by observation and reference.
StateCost10 baseTaskOutputMatrix(const Eigen::Vector2d& lengths, const State10& trim,
                                 const Eigen::Vector3d& hip_midpoint_base);
StateCost10 baseTaskCost(const Eigen::Vector2d& lengths, const State10& trim, const Eigen::Vector3d& hip_midpoint_base,
                         const State10& output_weights, const State10& internal_weights);
}  // namespace rm_chassis_controllers
