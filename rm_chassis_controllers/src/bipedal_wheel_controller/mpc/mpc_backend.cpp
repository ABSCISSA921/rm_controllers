#include "bipedal_wheel_controller/mpc/mpc_backend.h"
#include "bipedal_wheel_controller/dynamics/model10.h"

#include <Eigen/Cholesky>
#include <unsupported/Eigen/MatrixFunctions>
#include <tinympc/tiny_api.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
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

bool positiveFinite(double value)
{
  return std::isfinite(value) && value > 0.;
}

bool validBounds(const Input4& lower, const Input4& upper, const Input4& feedforward)
{
  return lower.allFinite() && upper.allFinite() && (lower.array() <= upper.array()).all() &&
         (lower - feedforward).allFinite() && (upper - feedforward).allFinite();
}

bool prepareCache(TinySolver& solver, const mpc_backend::Params& params, tinyMatrix& terminal_reference_map)
{
  const auto& a = solver.work->Adyn;
  const auto& b = solver.work->Bdyn;
  const tinyMatrix q = (params.q.array() + params.rho).matrix().asDiagonal();
  const tinyMatrix r = (params.r.array() + params.rho).matrix().asDiagonal();
  if (!q.allFinite() || !r.allFinite())
    return false;

  tinyMatrix p = q;
  bool converged = false;
  for (int iteration = 0; iteration < 10000; ++iteration)
  {
    const tinyMatrix hessian = r + b.transpose() * p * b;
    const Eigen::LDLT<tinyMatrix> factor(hessian);
    if (factor.info() != Eigen::Success || !factor.isPositive())
      return false;
    const tinyMatrix gain = factor.solve(b.transpose() * p * a);
    tinyMatrix next = q + a.transpose() * p * (a - b * gain);
    next = ((next + next.transpose()) * .5).eval();
    if (!next.allFinite() || !gain.allFinite())
      return false;
    const double error = (next - p).cwiseAbs().maxCoeff();
    p.swap(next);
    if (error <= 1e-11 * (1. + p.cwiseAbs().maxCoeff()))
    {
      converged = true;
      break;
    }
  }
  if (!converged)
    return false;

  const tinyMatrix hessian = r + b.transpose() * p * b;
  const Eigen::LDLT<tinyMatrix> input_factor(hessian), state_factor(p);
  if (input_factor.info() != Eigen::Success || !input_factor.isPositive() || state_factor.info() != Eigen::Success ||
      !state_factor.isPositive())
    return false;

  auto& cache = *solver.cache;
  cache.rho = params.rho;
  cache.Pinf = p;
  cache.Kinf = input_factor.solve(b.transpose() * p * a);
  cache.Quu_inv = input_factor.solve(tinyMatrix::Identity(NU, NU));
  cache.AmBKt = (a - b * cache.Kinf).transpose();
  cache.APf = cache.AmBKt * p * solver.work->fdyn;
  cache.BPf = b.transpose() * p * solver.work->fdyn;
  cache.C1 = cache.Quu_inv;
  cache.C2 = cache.AmBKt;
  const tinyMatrix terminal_weight = p - params.rho * tinyMatrix::Identity(NX, NX);
  terminal_reference_map = state_factor.solve(terminal_weight);
  solver.work->Q = params.q;
  solver.work->R = params.r;

  const tinyMatrix residual = q + a.transpose() * p * (a - b * cache.Kinf) - p;
  return cache.Kinf.allFinite() && cache.Quu_inv.allFinite() && cache.AmBKt.allFinite() && cache.APf.allFinite() &&
         cache.BPf.allFinite() && terminal_reference_map.allFinite() && residual.allFinite() &&
         residual.cwiseAbs().maxCoeff() <= 1e-8 * (1. + p.cwiseAbs().maxCoeff());
}
}

struct mpc_backend::Impl
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::unique_ptr<TinySolver, SolverDeleter> solver;
  Params params;
  State10 equilibrium{ State10::Zero() };
  Input4 feedforward{ Input4::Zero() };
  tinyMatrix terminal_reference_map;
  Report report;

  void clearWorkspace()
  {
    if (!solver)
      return;
    auto& work = *solver->work;
    work.x.setZero();
    work.u.setZero();
    work.q.setZero();
    work.r.setZero();
    work.p.setZero();
    work.d.setZero();
    work.v.setZero();
    work.vnew.setZero();
    work.z.setZero();
    work.znew.setZero();
    work.g.setZero();
    work.y.setZero();
    work.Xref.setZero();
    work.Uref.setZero();
    work.Qu.setZero();
    work.primal_residual_state = 0.;
    work.primal_residual_input = 0.;
    work.dual_residual_state = 0.;
    work.dual_residual_input = 0.;
    work.status = 11;
    work.iter = 0;
    solver->solution->x.setZero();
    solver->solution->u.setZero();
    solver->solution->iter = 0;
    solver->solution->solved = 0;
  }

  bool updateBounds(const Input4& lower, const Input4& upper)
  {
    if (!validBounds(lower, upper, feedforward))
      return false;
    const tinyMatrix state_bounds = tinyMatrix::Zero(NX, params.horizon);
    const tinyMatrix input_lower = (lower - feedforward).replicate(1, params.horizon - 1);
    const tinyMatrix input_upper = (upper - feedforward).replicate(1, params.horizon - 1);
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
  if (!positiveFinite(params.sample_period) || params.horizon < 2 || !params.q.allFinite() || !params.r.allFinite() ||
      (params.q.array() < 0.).any() || (params.r.array() <= 0.).any() || !positiveFinite(params.rho) ||
      params.max_iter <= 0 || !positiveFinite(params.abs_pri_tol) || !positiveFinite(params.abs_dua_tol) ||
      !std::isfinite(params.max_solve_time) || params.max_solve_time < 0. || !model.A.allFinite() ||
      !model.B.allFinite() || !model.equilibrium.allFinite() || !model.uff.allFinite() || !model.residual.allFinite() ||
      model.residual.cwiseAbs().maxCoeff() > 1e-8 || !validBounds(params.input_min, params.input_max, model.uff))
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
    if (result != 0 || !next->solver || !prepareCache(*next->solver, params, next->terminal_reference_map))
      return false;

    auto& settings = *next->solver->settings;
    settings.abs_pri_tol = params.abs_pri_tol;
    settings.abs_dua_tol = params.abs_dua_tol;
    settings.max_iter = params.max_iter;
    settings.check_termination = 1;
    settings.en_state_bound = 0;
    settings.en_input_bound = 1;
    settings.en_state_soc = settings.en_input_soc = 0;
    settings.en_state_linear = settings.en_input_linear = 0;
    settings.en_tv_state_linear = settings.en_tv_input_linear = 0;
    settings.adaptive_rho = 0;
    if (!next->updateBounds(params.input_min, params.input_max))
      return false;
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
    solver.work->x.col(0) = state - impl_->equilibrium;
    solver.work->Xref = reference.colwise() - impl_->equilibrium;
    solver.work->Xref.col(impl_->params.horizon - 1) =
        (impl_->terminal_reference_map * solver.work->Xref.col(impl_->params.horizon - 1)).eval();
    if (!solver.work->x.col(0).allFinite() || !solver.work->Xref.allFinite())
    {
      impl_->clearWorkspace();
      impl_->report.status = Status::INVALID_INPUT;
      return false;
    }

    const int result = tiny_solve(&solver);
    impl_->report.solve_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    impl_->report.iterations = solver.solution->iter;
    impl_->report.status = Status::SOLVER_FAILED;
    if (impl_->params.max_solve_time > 0. && impl_->report.solve_time > impl_->params.max_solve_time)
      impl_->report.status = Status::TIME_LIMIT;
    else if (result == 0 && solver.solution->solved == 1 && solver.solution->x.allFinite() &&
             solver.solution->u.allFinite())
    {
      Input4 command = impl_->feedforward + solver.solution->u.col(0);
      if (command.allFinite())
      {
        bool within_bounds = true;
        for (int i = 0; i < NU; ++i)
        {
          const double scale = std::max({ 1., std::abs(impl_->params.input_min[i]),
                                          std::abs(impl_->params.input_max[i]), std::abs(impl_->feedforward[i]) });
          const double tolerance = 64. * std::numeric_limits<double>::epsilon() * scale;
          if ((command[i] < impl_->params.input_min[i] && impl_->params.input_min[i] - command[i] > tolerance) ||
              (command[i] > impl_->params.input_max[i] && command[i] - impl_->params.input_max[i] > tolerance))
            within_bounds = false;
          command[i] = std::max(impl_->params.input_min[i], std::min(impl_->params.input_max[i], command[i]));
        }
        if (within_bounds)
        {
          output = command;
          impl_->report.status = Status::SUCCESS;
          return true;
        }
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
