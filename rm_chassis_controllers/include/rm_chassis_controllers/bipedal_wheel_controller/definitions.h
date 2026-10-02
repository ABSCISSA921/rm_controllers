//
// Created by guanlin on 25-8-30.
//

#pragma once

#include "bipedal_wheel_controller/vmc/VMC.h"
#include <array>
#include <string>
#include <Eigen/Core>
#include <geometry_msgs/Vector3.h>
#include <utility>

namespace rm_chassis_controllers
{

namespace lqr10
{
using State10 = Eigen::Matrix<double, 10, 1>;
using Input4 = Eigen::Matrix<double, 4, 1>;
using Gain10 = Eigen::Matrix<double, 4, 10>;
using Joint6 = Eigen::Matrix<double, 6, 1>;
using Curve8 = Eigen::Matrix<double, 8, 1>;
enum Index
{
  S,
  V,
  YAW,
  YAW_RATE,
  THETA_L,
  DTHETA_L,
  THETA_R,
  DTHETA_R,
  BODY_PITCH,
  BODY_RATE
};
enum class Reason
{
  None = 0,
  NotNormal = 1,
  InvalidSnapshot = 2,
  InvalidTime = 3,
  OutsideDomain = 4,
  Posture = 5,
  NonfiniteOutput = 7,
  Terminated = 8
};

struct PhysicalLeg
{
  double mass{0.};
  Curve8 lb{Curve8::Zero()}, lw{Curve8::Zero()}, inertia{Curve8::Zero()};
  Curve8 offset{Curve8::Zero()};
};
struct PhysicalModel
{
  double mb{0.}, Ib{0.}, Izz{0.}, mw{0.}, Iw{0.}, Rw{0.}, Rl{0.}, g{0.}, lc{0.}, phi_c{0.};
  std::array<PhysicalLeg, 2> legs;
  Eigen::Vector2d geometry_domain{Eigen::Vector2d::Zero()};
};
// Fixed-size stopped-controller transaction. Model/domain/Uff are immutable after init.
struct GainUpdate
{
  Eigen::Matrix<double, 10, 40> coeffs{Eigen::Matrix<double, 10, 40>::Zero()};
  State10 q{State10::Zero()};
  Input4 r{Input4::Zero()};
  uint64_t revision{0};
};
struct Config
{
  Eigen::Matrix<double, 10, 40> coeffs{Eigen::Matrix<double, 10, 40>::Zero()};
  Eigen::Matrix<double, 2, 2> domain{Eigen::Matrix<double, 2, 2>::Zero()};
  Input4 uff{Input4::Zero()};
  double wheel_radius{0.};
  double max_angle{0.};
  double length_reference_tau{0.};
  double position_release_tau{0.}, hold_capture_speed{0.};
  PhysicalModel model;
  State10 q{State10::Zero()};
  Input4 r{Input4::Zero()};
  void validateParameters() const;
};

// Per-controller observation history. Neither side shares a static variable.
struct ObservationHistory
{
  bool initialized{false}, ever_initialized{false};
  double s{0.}, yaw{0.}, wrapped_yaw{0.}, last_time{0.}, last_v{0.};
};

// Shared reference history, Normal outputs and controller lifecycle state; not a sensor snapshot.
struct NormalFeedback
{
  State10 reference{State10::Zero()}; // Xref, including the integrated s/yaw targets.
  bool reference_valid{false};
  Input4 output{Input4::Zero()}; // U4 consumed by VMC/wheels and the compatibility status.
  Eigen::Vector2d axial_force{Eigen::Vector2d::Zero()}; // F consumed by VMC and the same status.
  bool active{false}; // Selects Normal U4/F for status publication; does not authorize control.
  bool faulted{false}; // Latched until starting(), including faults outside Normal.
  Reason reason{Reason::NotNormal}; // Current Normal status or first latched fault cause.
};
} // namespace lqr10

struct ControlParams
{
  double recovery_leg_speed{5.};
  double jumpOverTime_{5.}, down5cmStairPitchThreshold{.1}, down5cmStairThetaThreshold{.2};
  double jump_up_force{210.}, off_ground_force{90.};
};
struct ModelParams
{
  double M{};         // Mass used by existing axial turning compensation, kg.
  double f_gravity{}; // Per-side nominal axial support, N.
};

struct ChassisGeometryParams
{
  double chassis_height; // 底盘高度 (m)
  double wheel_track;    // 轮距(左右)
};

struct SpringParams
{
  double s2;
  double s3;
  double alpha_s;
  double f_spring; // Spring Force
};

struct LegStateThresholdParams
{
  double under_lower;
  double under_upper;
  double front_lower;
  double front_upper;
  double behind_lower;
  double behind_upper;
  double upstair_des_theta;
  double upstair_des_length;
  double upstair_exit_theta_threshold;
  double upstair_exit_length_threshold;
  double unstick_threshold;
  double arrive_time_threshold;
};

struct LegCommand
{
  double force;    // Thrust
  double torque;   // Torque
  double input[2]; // input
};

enum LegOrientation
{
  UNDER,
  FRONT,
  BEHIND
};

enum JumpPhase
{
  LEG_RETRACTION,
  JUMP_UP,
  OFF_GROUND,
  IDLE,
};

enum BalanceMode
{
  NORMAL,
  STAND_UP,
  SIT_DOWN,
  RECOVER,
  UPSTAIRS,
  PROTECT
};

enum Side
{
  LEFT = 0,
  RIGHT,
};

enum
{
  WHEEL_T = 0,
  LEG_Tp,
};

// Named mode views derived from the canonical X10, not a second estimator.
struct LegPosture
{
  double theta{0.}, rate{0.};
};

struct LegState
{
  LegPosture posture;
  double angle[2]; // [0]: hip, [1]: knee
  VMCPtr vmc{nullptr};
  bool unstick = false;
};

struct ChassisState
{
  // Current whole-body observation, shared by Normal and the original mode interface.
  lqr10::State10 x{lqr10::State10::Zero()};
  Eigen::Vector2d length{Eigen::Vector2d::Zero()}, dlength{Eigen::Vector2d::Zero()};
  double time{0.}, dt{0.}; // ROS label and host-supplied computation step; not interchangeable clocks.
  bool valid{false}, raw_valid{false};
  lqr10::Reason observation_reason{lqr10::Reason::InvalidSnapshot};

  // Raw body-frame IMU / wrapped attitude also serve recovery outside the LQR observation range.
  geometry_msgs::Vector3 angular_vel;
  geometry_msgs::Vector3 linear_acc;
  double x_vel = 0.0; // Last valid speed for odometry, which runs before the next observation update.
  double recovery_pitch = 0.0, recovery_roll = 0.0, upright_z = 1.0;
  double roll = 0.0;
  double roll_rate = 0.0; // ZYX Euler derivative, rad/s; valid only with the full observation.
  double pitch = 0.0;
  double yaw = 0.0;
  double yaw_total = 0.0;
  double yaw_total_last = 0.0;
};

constexpr std::array<std::pair<JumpPhase, const double>, 3> jumpLengthDes = {
    {{JumpPhase::LEG_RETRACTION, 0.11}, {JumpPhase::JUMP_UP, 0.34}, {JumpPhase::OFF_GROUND, 0.11}}};
} // namespace rm_chassis_controllers
