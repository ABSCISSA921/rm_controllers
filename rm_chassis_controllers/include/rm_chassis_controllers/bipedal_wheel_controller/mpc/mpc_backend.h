#pragma once

#include <memory>

#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
struct LinearModel;

class mpc_backend
{
public:
  using StateTrajectory = Eigen::Matrix<double, 10, Eigen::Dynamic>;

  struct Params
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double sample_period{ 0. };
    int horizon{ 16 };
    State10 q{ State10::Ones() };
    Input4 r{ Input4::Ones() };

    Input4 input_min{ Input4::Zero() };
    Input4 input_max{ Input4::Zero() };

    double rho{ 1. };
    int max_iter{ 100 };
    double abs_pri_tol{ 1e-4 };
    double abs_dua_tol{ 1e-4 };
    double max_solve_time{ 0. };
  };

  enum class Status
  {
    NOT_INITIALIZED,
    SUCCESS,
    INVALID_INPUT,
    SOLVER_FAILED,
    TIME_LIMIT
  };

  struct Report
  {
    Status status{ Status::NOT_INITIALIZED };
    int iterations{ 0 };
    double solve_time{ 0. };
  };

  mpc_backend();
  ~mpc_backend();

  mpc_backend(const mpc_backend&) = delete;
  mpc_backend& operator=(const mpc_backend&) = delete;
  mpc_backend(mpc_backend&&) = delete;
  mpc_backend& operator=(mpc_backend&&) = delete;

  bool initialize(const LinearModel& continuous_model, const Params& params);

  bool setInputBounds(const Input4& input_min, const Input4& input_max);

  bool solve(const State10& state, const StateTrajectory& reference, Input4& output);

  void reset();
  bool initialized() const noexcept;
  const Report& report() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
