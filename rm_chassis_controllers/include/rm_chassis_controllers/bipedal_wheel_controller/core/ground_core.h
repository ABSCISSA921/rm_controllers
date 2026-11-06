#pragma once

#include <array>
#include <Eigen/Geometry>
#include <control_toolbox/pid.h>
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/estimation/velocity_kf.h"
#include "bipedal_wheel_controller/normal_reference.h"

namespace rm_chassis_controllers
{
// All joint arrays use LH, LK, RH, RK, LW, RW. IMU vectors are in the sensor frame;
// orientation_world_imu rotates that frame into the inertial world frame.
struct GroundSample
{
  Joint6 position{ Joint6::Zero() }, velocity{ Joint6::Zero() };
  Eigen::Quaterniond orientation_world_imu{ Eigen::Quaterniond::Identity() };
  Eigen::Vector3d gyro_imu{ Eigen::Vector3d::Zero() }, specific_force_imu{ Eigen::Vector3d::Zero() };
  double dt{ 0.001 };
};

struct GroundInstallation
{
  Eigen::Vector3d hip_midpoint_base{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d imu_position_base{ Eigen::Vector3d::Zero() };
  Eigen::Quaterniond rotation_base_imu{ Eigen::Quaterniond::Identity() };
};

// Commands have already passed the caller's frame conversion/ramp. Ground-only
// execution starts in Normal; action routing remains in the ROS mode manager.
struct GroundCommand
{
  double forward_velocity{ 0. }, yaw_rate{ 0. }, leg_length{ 0.2 };
  bool complete_stand{ true };
};

struct GroundPidGains
{
  double p{ 0. }, i{ 0. }, d{ 0. }, i_max{ 0. }, i_min{ 0. };
  bool antiwindup{ false };
};

struct GroundCoreParameters
{
  Config lqr;
  double l1{ 0.21 }, l2{ 0.248 };
  ModelParams model;
  double wheel_track{ 0.446 };
  GroundInstallation installation;
  VelocityKalmanFilter::Params velocity_estimator;
  std::array<GroundPidGains, 2> leg_pid;
  GroundPidGains roll_pid;
};

// These stages are also called by the ROS plugin, using its existing state and
// PID objects. No hardware handles, TF lookup, ROS master, or simulator is needed.
namespace ground_core
{
bool transformImu(const GroundSample& sample, const GroundInstallation& installation, ChassisState& state,
                  Eigen::Matrix3d& rotation_world_base);
void updateVirtualLegs(LegState legs[2], const Joint6& position, const Joint6& velocity);
double readVirtualLeg(ChassisState& state, LegState& leg, int side, double hip, double hip_rate, double knee_rate,
                      double pitch_rate);
void observe(ChassisState& state, double pitch_rate, const Eigen::Vector2d& carrier_rates,
             const Eigen::Vector2d& wheel_rates, double wheel_radius);
void updateGroundVelocity(ChassisState& state, LegState legs[2], VelocityKalmanFilter& filter,
                          const Eigen::Matrix3d& rotation_world_base, const Eigen::Vector3d& imu_position_base,
                          const Eigen::Vector3d& hip_midpoint_base, double wheel_radius);
void updateBaseObservation(ChassisState& state, const Eigen::Matrix3d& rotation_world_base,
                           const Eigen::Vector3d& hip_midpoint_base, double wheel_radius);
void integrateVelocity(ChassisState& state);
void computeFeedback(const Config& config, ChassisState& state, NormalFeedback& feedback,
                     LengthReference& length_reference, const GroundCommand& command);
void updateTaskReference(const Config& config, ChassisState& state, NormalFeedback& feedback,
                         LengthReference& length_reference, const GroundCommand& command);
Joint6 computeGroundTorques(const Config& config, const ModelParams& model, double wheel_track, ChassisState& state,
                            LegState legs[2], NormalFeedback& feedback, LengthReference& length_reference,
                            const std::array<control_toolbox::Pid*, 2>& leg_pid, control_toolbox::Pid& roll_pid,
                            const GroundCommand& command);
double springForce(double length);
}  // namespace ground_core

class GroundControllerCore
{
public:
  // The caller designs config.lqr once, outside the control loop.
  void configure(const GroundCoreParameters& parameters);
  void enterNormal();
  Joint6 step(const GroundSample& sample, const GroundCommand& command);
  void updateObservation(const GroundSample& sample);
  void updateTaskReference(const GroundCommand& command);
  const LengthReference& lengthReference() const
  {
    return length_reference_;
  }
  const ChassisState& state() const
  {
    return state_;
  }
  const NormalFeedback& feedback() const
  {
    return feedback_;
  }
  const LegState& leg(Side side) const
  {
    return legs_[side];
  }
  const GroundCoreParameters& parameters() const
  {
    return parameters_;
  }

private:
  GroundCoreParameters parameters_;
  ChassisState state_;
  LegState legs_[2];
  NormalFeedback feedback_;
  VelocityKalmanFilter velocity_filter_;
  std::array<control_toolbox::Pid, 2> leg_pid_;
  control_toolbox::Pid roll_pid_;
  LengthReference length_reference_;
  Eigen::Matrix3d rotation_world_base_{ Eigen::Matrix3d::Identity() };
};
}  // namespace rm_chassis_controllers
