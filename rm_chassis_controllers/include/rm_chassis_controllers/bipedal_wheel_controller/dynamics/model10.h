#pragma once
#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
// Stateless model/design routines; Normal uses evaluate() for poly33 and analytic trim.
struct LinearModel
{
  Eigen::Matrix<double, 10, 10> A{ Eigen::Matrix<double, 10, 10>::Zero() };
  Eigen::Matrix<double, 10, 4> B{ Eigen::Matrix<double, 10, 4>::Zero() };
  State10 equilibrium{ State10::Zero() }, residual{ State10::Zero() };
  Input4 uff{ Input4::Zero() };
};
struct DesignReport
{
  double care_residual{ 0. }, max_real{ -1e30 };
  double fit_error{ 0. }, equilibrium_error{ 0. };
  unsigned nodes{ 0 }, validation_points{ 0 };
};
PhysicalLeg makePhysicalLeg(double mass);
Input4 staticFeedforward(const PhysicalModel& p);
State10 staticEquilibrium(const PhysicalModel& p, const Eigen::Vector2d& lengths, const Input4& uff);
LinearModel linearModel(const PhysicalModel& p, const Eigen::Vector2d& lengths);
Gain10 solveCare(const LinearModel& m, const State10& q, const Input4& r, DesignReport& report);
GainUpdate designTable(const Config& config, const State10& q, const Input4& r, DesignReport& report);
// Runtime evaluation uses the supplied lengths directly, without domain or finite-value rejection.
void evaluate(const Config& config, const Eigen::Vector2d& lengths, Gain10& gain, Input4& uff,
              State10* equilibrium = nullptr);
}  // namespace rm_chassis_controllers
