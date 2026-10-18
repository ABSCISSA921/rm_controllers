#include "bipedal_wheel_controller/mpc/mpc_backend.h"
#include "bipedal_wheel_controller/dynamics/model10.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <tinympc/tiny_api.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

namespace rm_chassis_controllers
{
namespace
{
constexpr int NX = 10;
constexpr int NU = 4;

struct SolverDeleter
{
  void operator()(TinySolver* solver) const noexcept
  {
    if (!solver)
      return;
    delete solver->solution;
    delete solver->settings;
    delete solver->cache;
    delete solver->work;
    delete solver;
  }
};

}

struct mpc_backend::Impl
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::unique_ptr<TinySolver, SolverDeleter> solver;
  Params params;
  State10 equilibrium{ State10::Zero() };
  Input4 feedforward{ Input4::Zero() };
  Report report;

  void clearWorkspace()
  {
    if (!solver)
      return;
    auto& work = *solver->work;
    work.v.setZero();
    work.vnew.setZero();
    work.z.setZero();
    work.znew.setZero();
    work.g.setZero();
    work.y.setZero();
    solver->solution->x.setZero();
    solver->solution->u.setZero();
    solver->solution->iter = 0;
    solver->solution->solved = 0;
  }

  bool updateBounds(const Input4& lower, const Input4& upper)
  {
    const Input4 lower_deviation = lower - feedforward;
    const Input4 upper_deviation = upper - feedforward;
    if (!lower_deviation.allFinite() || !upper_deviation.allFinite() || (lower.array() > upper.array()).any())
      return false;
    const tinyMatrix state_bounds = tinyMatrix::Zero(NX, params.horizon);
    const tinyMatrix input_lower = lower_deviation.replicate(1, params.horizon - 1);
    const tinyMatrix input_upper = upper_deviation.replicate(1, params.horizon - 1);
    if (tiny_set_bound_constraints(solver.get(), state_bounds, state_bounds, input_lower, input_upper) != 0)
      return false;
    params.input_min = lower;
    params.input_max = upper;
    clearWorkspace();
    return true;
  }
};

mpc_backend::mpc_backend() : impl_(new Impl)
{
}

mpc_backend::~mpc_backend() = default;

bool mpc_backend::initialize(const LinearModel& model, const Params& params)
{
  impl_->solver.reset();
  impl_->report = Report{};
  impl_->report.status = Status::INVALID_INPUT;
  if (!std::isfinite(params.sample_period) || params.sample_period <= 0. || params.horizon < 2 ||
      !params.q.allFinite() || !params.r.allFinite() || (params.q.array() < 0.).any() ||
      (params.r.array() <= 0.).any() || !std::isfinite(params.rho) || params.rho <= 0. || params.max_iter <= 0 ||
      !std::isfinite(params.abs_pri_tol) || params.abs_pri_tol <= 0. || !std::isfinite(params.abs_dua_tol) ||
      params.abs_dua_tol <= 0. || !std::isfinite(params.max_solve_time) || params.max_solve_time < 0. ||
      !model.A.allFinite() || !model.B.allFinite() || !model.equilibrium.allFinite() || !model.uff.allFinite() ||
      !model.residual.allFinite() || !params.input_min.allFinite() || !params.input_max.allFinite() ||
      (params.input_min.array() > params.input_max.array()).any())
    return false;

  impl_->report.status = Status::SOLVER_FAILED;
  try
  {
    std::unique_ptr<Impl> next(new Impl);
    next->params = params;
    next->equilibrium = model.equilibrium;
    next->feedforward = model.uff;
    tinyMatrix continuous = tinyMatrix::Zero(NX + NU + 1, NX + NU + 1);
    continuous.topLeftCorner(NX, NX) = model.A;
    continuous.block(0, NX, NX, NU) = model.B;
    continuous.block(0, NX + NU, NX, 1) = model.residual;
    continuous *= params.sample_period;
    if (!continuous.allFinite())
      return false;
    const tinyMatrix discrete = continuous.exp();
    if (!discrete.allFinite())
      return false;

    TinySolver* raw = nullptr;
    int result;
    try
    {
      result = tiny_setup(&raw, discrete.topLeftCorner(NX, NX), discrete.block(0, NX, NX, NU),
                          discrete.block(0, NX + NU, NX, 1), params.q.asDiagonal(), params.r.asDiagonal(), params.rho,
                          NX, NU, params.horizon, 0);
    }
    catch (...)
    {
      SolverDeleter{}(raw);
      throw;
    }
    next->solver.reset(raw);
    if (result != 0 || !next->solver)
      return false;

    auto& settings = *next->solver->settings;
    settings.abs_pri_tol = params.abs_pri_tol;
    settings.abs_dua_tol = params.abs_dua_tol;
    settings.max_iter = params.max_iter;
    settings.check_termination = 1;
    settings.en_state_bound = 0;
    settings.en_input_bound = 1;
    settings.adaptive_rho = 0;
    if (!next->updateBounds(params.input_min, params.input_max))
    {
      impl_->report.status = Status::INVALID_INPUT;
      return false;
    }
    next->report.status = Status::SUCCESS;
    impl_ = std::move(next);
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

bool mpc_backend::setInputBounds(const Input4& input_min, const Input4& input_max)
{
  impl_->report = Report{};
  if (!initialized())
    return false;
  impl_->report.status = Status::INVALID_INPUT;
  try
  {
    if (!impl_->updateBounds(input_min, input_max))
      return false;
    impl_->report.status = Status::SUCCESS;
    return true;
  }
  catch (const std::exception&)
  {
    impl_->report.status = Status::SOLVER_FAILED;
    return false;
  }
}

bool mpc_backend::solve(const State10& state, const StateTrajectory& reference, Input4& output)
{
  impl_->report = Report{};
  if (!initialized())
    return false;
  if (!state.allFinite() || reference.cols() != impl_->params.horizon || !reference.allFinite())
  {
    impl_->clearWorkspace();
    impl_->report.status = Status::INVALID_INPUT;
    return false;
  }

  const auto start = std::chrono::steady_clock::now();
  try
  {
    auto& solver = *impl_->solver;
    const tinyVector initial_state = state - impl_->equilibrium;
    const tinyMatrix state_reference = reference.colwise() - impl_->equilibrium;
    if (!initial_state.allFinite() || !state_reference.allFinite())
    {
      impl_->clearWorkspace();
      impl_->report.status = Status::INVALID_INPUT;
      return false;
    }

    tiny_set_x0(&solver, initial_state);
    tiny_set_x_ref(&solver, state_reference);
    const int result = tiny_solve(&solver);
    impl_->report.solve_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    impl_->report.iterations = solver.solution->iter;
    impl_->report.status = Status::SOLVER_FAILED;
    if (impl_->params.max_solve_time > 0. && impl_->report.solve_time > impl_->params.max_solve_time)
      impl_->report.status = Status::TIME_LIMIT;
    else if (result == 0 && solver.solution->solved == 1)
    {
      const Input4 command = impl_->feedforward + solver.solution->u.col(0);
      if (command.allFinite())
      {
        output = command.cwiseMax(impl_->params.input_min).cwiseMin(impl_->params.input_max);
        impl_->report.status = Status::SUCCESS;
        return true;
      }
    }
  }
  catch (const std::exception&)
  {
    impl_->report.solve_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    impl_->report.status = Status::SOLVER_FAILED;
  }
  impl_->clearWorkspace();
  return false;
}

void mpc_backend::reset()
{
  impl_->clearWorkspace();
  impl_->report = Report{};
  if (initialized())
    impl_->report.status = Status::SUCCESS;
}

bool mpc_backend::initialized() const noexcept
{
  return static_cast<bool>(impl_->solver);
}

const mpc_backend::Report& mpc_backend::report() const noexcept
{
  return impl_->report;
}
}
