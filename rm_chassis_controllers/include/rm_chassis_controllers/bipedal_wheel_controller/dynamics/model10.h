#pragma once
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
namespace lqr10
{
// Non-real-time, stateless model/design routines. Normal only evaluates poly33.
struct LinearModel
{
  Eigen::Matrix<double, 10, 10> A{Eigen::Matrix<double, 10, 10>::Zero()};
  Eigen::Matrix<double, 10, 4> B{Eigen::Matrix<double, 10, 4>::Zero()};
  State10 equilibrium{State10::Zero()}, residual{State10::Zero()};
  Input4 uff{Input4::Zero()};
};
struct DesignReport
{
  double care_residual{0.}, minimum_p{0.}, max_real{-1e30};
  double fit_error{0.}, equilibrium_error{0.};
  unsigned nodes{0}, validation_points{0};
};
Input4 staticFeedforward(const PhysicalModel &);
State10 staticEquilibrium(const PhysicalModel &, const Eigen::Vector2d &lengths, const Input4 &uff);
LinearModel linearModel(const PhysicalModel &, const Eigen::Vector2d &lengths);
Gain10 solveCare(const LinearModel &, const State10 &q, const Input4 &r, DesignReport &);
GainUpdate designTable(const Config &, const State10 &q, const Input4 &r, DesignReport &);
bool evaluate(const Config &, const Eigen::Vector2d &lengths, Gain10 &, Input4 &, State10 *equilibrium = nullptr);
} // namespace lqr10
} // namespace rm_chassis_controllers
