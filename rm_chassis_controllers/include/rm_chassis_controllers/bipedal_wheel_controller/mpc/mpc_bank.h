#pragma once

#include <memory>
#include <vector>

#include "bipedal_wheel_controller/mpc/mpc_backend.h"

namespace rm_chassis_controllers
{
class mpc_bank
{
public:
  struct Entry
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector2d lengths;
    State10 equilibrium;
    Input4 feedforward;
    mpc_backend backend;
  };

  bool initialize(const PhysicalModel& model, const mpc_backend::Params& params, const std::vector<double>& leg_lengths,
                  double hysteresis);
  Entry* select(const Eigen::Vector2d& lengths);
  void reset();
  const Entry* active() const;
  std::size_t size() const;

private:
  std::size_t selectAxis(double length, std::size_t previous) const;
  std::vector<double> leg_lengths_;
  std::vector<std::unique_ptr<Entry>> entries_;
  double hysteresis_{ 0. };
  std::size_t active_index_{ 0 };
  bool selected_{ false };
};
}  // namespace rm_chassis_controllers
