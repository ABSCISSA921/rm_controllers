#include "bipedal_wheel_controller/mpc/mpc_backend.h"
#include "bipedal_wheel_controller/dynamics/model10.h"

#include <gtest/gtest.h>
#include <Eigen/Cholesky>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rm_chassis_controllers
{
namespace
{
using Backend = mpc_backend;

LinearModel simpleModel()
{
  LinearModel model;
  model.A = -.7 * Eigen::Matrix<double, 10, 10>::Identity();
  model.B.topRows<4>().setIdentity();
  model.equilibrium << .1, -.2, .3, -.4, .5, -.6, .7, -.8, .9, -1.;
  model.uff << .3, -.2, .4, -.5;
  return model;
}

Backend::Params parameters()
{
  Backend::Params params;
  params.sample_period = .1;
  params.horizon = 4;
  params.q = State10::LinSpaced(1., 10.);
  params.r << .2, .3, .4, .5;
  params.input_min.setConstant(-20.);
  params.input_max.setConstant(20.);
  params.rho = .7;
  params.max_iter = 10000;
  params.abs_pri_tol = params.abs_dua_tol = 1e-9;
  return params;
}

Backend::StateTrajectory trajectory(const LinearModel& model, int horizon)
{
  Backend::StateTrajectory result = model.equilibrium.replicate(1, horizon);
  for (int i = 0; i < horizon; ++i)
    for (int j = 0; j < 4; ++j)
      result(j, i) += (j % 2 == 0 ? 1. : -1.) * (.2 + .1 * i);
  return result;
}

double scalarQp(const LinearModel& model, const Backend::Params& params, const State10& state,
                const Backend::StateTrajectory& reference, int channel)
{
  const int intervals = params.horizon - 1;
  const double a = std::exp(-.7 * params.sample_period), b = (1. - a) / .7;
  const double q_aug = params.q[channel] + params.rho, r_aug = params.r[channel] + params.rho;
  const double c = r_aug * (1. - a * a) - q_aug * b * b;
  const double p_aug = (-c + std::sqrt(c * c + 4. * b * b * q_aug * r_aug)) / (2. * b * b);
  const double terminal_weight = p_aug - params.rho;
  Eigen::MatrixXd hessian = params.r[channel] * Eigen::MatrixXd::Identity(intervals, intervals);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(intervals);
  for (int i = 1; i <= intervals; ++i)
  {
    Eigen::VectorXd influence = Eigen::VectorXd::Zero(intervals);
    for (int j = 0; j < i; ++j)
      influence[j] = b * std::pow(a, i - 1 - j);
    const double weight = i == intervals ? terminal_weight : params.q[channel];
    const double baseline = std::pow(a, i) * (state[channel] - model.equilibrium[channel]) +
                            model.residual[channel] * (1. - std::pow(a, i)) / .7;
    const double error = baseline - (reference(channel, i) - model.equilibrium[channel]);
    hessian.noalias() += weight * influence * influence.transpose();
    gradient.noalias() += weight * error * influence;
  }

  const double lower = params.input_min[channel] - model.uff[channel];
  const double upper = params.input_max[channel] - model.uff[channel];
  double best_cost = std::numeric_limits<double>::infinity();
  Eigen::VectorXd best;
  int combinations = 1;
  for (int i = 0; i < intervals; ++i)
    combinations *= 3;
  for (int combination = 0; combination < combinations; ++combination)
  {
    int code = combination;
    Eigen::VectorXd candidate = Eigen::VectorXd::Zero(intervals);
    std::vector<int> free;
    for (int i = 0; i < intervals; ++i)
    {
      const int mode = code % 3;
      code /= 3;
      if (mode == 0)
        free.push_back(i);
      else
        candidate[i] = mode == 1 ? lower : upper;
    }
    if (!free.empty())
    {
      Eigen::MatrixXd reduced(free.size(), free.size());
      Eigen::VectorXd rhs(free.size());
      for (std::size_t i = 0; i < free.size(); ++i)
      {
        rhs[i] = -gradient[free[i]] - hessian.row(free[i]).dot(candidate);
        for (std::size_t j = 0; j < free.size(); ++j)
          reduced(i, j) = hessian(free[i], free[j]);
      }
      const Eigen::VectorXd solution = reduced.ldlt().solve(rhs);
      for (std::size_t i = 0; i < free.size(); ++i)
        candidate[free[i]] = solution[i];
    }
    if ((candidate.array() < lower - 1e-9).any() || (candidate.array() > upper + 1e-9).any())
      continue;
    const double cost = .5 * candidate.dot(hessian * candidate) + gradient.dot(candidate);
    if (cost < best_cost)
    {
      best_cost = cost;
      best = candidate;
    }
  }
  if (!std::isfinite(best_cost))
    throw std::runtime_error("Independent box QP has no candidate");
  return model.uff[channel] + best[0];
}

TEST(BipedalMpcBackend, MatchesIndependentQpWithNonzeroReferencesAndActiveTorqueBounds)
{
  const auto model = simpleModel();
  for (double limit : { 20., .25 })
    for (double rho : { .1, 2. })
    {
      auto params = parameters();
      params.rho = rho;
      params.input_min.setConstant(-limit);
      params.input_max.setConstant(limit);
      Backend backend;
      ASSERT_TRUE(backend.initialize(model, params));
      State10 state = model.equilibrium;
      state.head<4>() += (Input4() << 2., -2., 1., -1.).finished();
      const auto reference = trajectory(model, params.horizon);
      Input4 output;
      ASSERT_TRUE(backend.solve(state, reference, output));
      EXPECT_EQ(backend.report().status, Backend::Status::SUCCESS);
      EXPECT_GT(backend.report().iterations, 0);
      EXPECT_GE(backend.report().solve_time, 0.);
      for (int channel = 0; channel < 4; ++channel)
        EXPECT_NEAR(output[channel], scalarQp(model, params, state, reference, channel), 2e-7);
      EXPECT_TRUE((output.array() >= params.input_min.array()).all());
      EXPECT_TRUE((output.array() <= params.input_max.array()).all());
      if (limit < 1.)
      {
        EXPECT_NEAR(output[0], -limit, 1e-9);
        EXPECT_NEAR(output[1], limit, 1e-9);
      }
    }
}

TEST(BipedalMpcBackend, ReturnsFeedforwardOnceAtEquilibriumAndSupportsZeroStateWeights)
{
  const auto model = simpleModel();
  auto params = parameters();
  params.q.setZero();
  Backend backend;
  ASSERT_TRUE(backend.initialize(model, params));
  const Backend::StateTrajectory reference = model.equilibrium.replicate(1, params.horizon);
  Input4 output;
  ASSERT_TRUE(backend.solve(model.equilibrium, reference, output));
  EXPECT_TRUE(output.isApprox(model.uff, 1e-12));
  State10 state = model.equilibrium;
  state[0] += .5;
  ASSERT_TRUE(backend.solve(state, reference, output));
  EXPECT_NEAR(output[0], scalarQp(model, params, state, reference, 0), 2e-7);
}

TEST(BipedalMpcBackend, RejectsInvalidParametersAndInvalidatesModelOnFailedReinitialization)
{
  const auto model = simpleModel();
  const auto good = parameters();
  std::vector<Backend::Params, Eigen::aligned_allocator<Backend::Params>> invalid(12, good);
  invalid[0].sample_period = 0.;
  invalid[1].horizon = 1;
  invalid[2].q[0] = -1.;
  invalid[3].r[0] = 0.;
  invalid[4].rho = -1.;
  invalid[5].max_iter = 0;
  invalid[6].abs_pri_tol = 0.;
  invalid[7].abs_dua_tol = std::numeric_limits<double>::infinity();
  invalid[8].max_solve_time = -1.;
  invalid[9].input_min[0] = good.input_max[0] + 1.;
  invalid[10].q[0] = std::numeric_limits<double>::quiet_NaN();
  invalid[11].input_max[0] = std::numeric_limits<double>::infinity();
  Backend backend;
  for (const auto& params : invalid)
  {
    EXPECT_FALSE(backend.initialize(model, params));
    EXPECT_FALSE(backend.initialized());
    EXPECT_EQ(backend.report().status, Backend::Status::INVALID_INPUT);
  }
  ASSERT_TRUE(backend.initialize(model, good));
  auto bad_model = model;
  bad_model.residual[0] = .1;
  EXPECT_FALSE(backend.initialize(bad_model, good));
  EXPECT_FALSE(backend.initialized());
  bad_model = model;
  bad_model.A(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(backend.initialize(bad_model, good));
  bad_model = model;
  bad_model.B.setZero();
  bad_model.A.setIdentity();
  EXPECT_FALSE(backend.initialize(bad_model, good));
  EXPECT_FALSE(backend.initialized());
  EXPECT_EQ(backend.report().status, Backend::Status::SOLVER_FAILED);
}

TEST(BipedalMpcBackend, RejectsInvalidObservationAndReferenceWithoutOverwritingOutput)
{
  const auto model = simpleModel();
  const auto params = parameters();
  const auto reference = trajectory(model, params.horizon);
  Backend backend;
  Input4 output = Input4::Constant(123.);
  EXPECT_FALSE(backend.solve(model.equilibrium, reference, output));
  EXPECT_EQ(backend.report().status, Backend::Status::NOT_INITIALIZED);
  EXPECT_FALSE(backend.setInputBounds(params.input_min, params.input_max));
  ASSERT_TRUE(backend.initialize(model, params));
  State10 invalid_state = model.equilibrium;
  invalid_state[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(backend.solve(invalid_state, reference, output));
  EXPECT_EQ(backend.report().status, Backend::Status::INVALID_INPUT);
  EXPECT_TRUE(output.isConstant(123.));
  Backend::StateTrajectory short_reference = reference.leftCols(params.horizon - 1);
  EXPECT_FALSE(backend.solve(model.equilibrium, short_reference, output));
  auto invalid_reference = reference;
  invalid_reference(0, 1) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(backend.solve(model.equilibrium, invalid_reference, output));
  EXPECT_TRUE(output.isConstant(123.));
  ASSERT_TRUE(backend.solve(model.equilibrium, reference, output));
  for (int channel = 0; channel < 4; ++channel)
    EXPECT_NEAR(output[channel], scalarQp(model, params, model.equilibrium, reference, channel), 2e-7);
}

TEST(BipedalMpcBackend, KeepsValidBoundsOnRejectedUpdateAndClearsHistoryOnReset)
{
  const auto model = simpleModel();
  auto params = parameters();
  Backend backend;
  ASSERT_TRUE(backend.initialize(model, params));
  State10 state = model.equilibrium + State10::Constant(.3);
  const auto reference = trajectory(model, params.horizon);
  Input4 output;
  ASSERT_TRUE(backend.solve(state, reference, output));
  EXPECT_FALSE(backend.setInputBounds(Input4::Constant(2.), Input4::Constant(1.)));
  ASSERT_TRUE(backend.solve(state, reference, output));
  for (int channel = 0; channel < 4; ++channel)
    EXPECT_NEAR(output[channel], scalarQp(model, params, state, reference, channel), 2e-7);
  params.input_min.setConstant(.1);
  params.input_max.setConstant(.15);
  ASSERT_TRUE(backend.setInputBounds(params.input_min, params.input_max));
  EXPECT_EQ(backend.report().iterations, 0);
  ASSERT_TRUE(backend.solve(state, reference, output));
  for (int channel = 0; channel < 4; ++channel)
    EXPECT_NEAR(output[channel], scalarQp(model, params, state, reference, channel), 2e-7);
  const Input4 expected = output;
  backend.reset();
  EXPECT_TRUE(backend.initialized());
  EXPECT_EQ(backend.report().iterations, 0);
  ASSERT_TRUE(backend.solve(state, reference, output));
  EXPECT_TRUE(output.isApprox(expected, 1e-8));
}

TEST(BipedalMpcBackend, RejectsUnconvergedAndLateSolutionsWithoutOverwritingOutput)
{
  const auto model = simpleModel();
  auto params = parameters();
  params.max_iter = 1;
  params.abs_pri_tol = params.abs_dua_tol = 1e-14;
  Backend backend;
  ASSERT_TRUE(backend.initialize(model, params));
  const auto reference = trajectory(model, params.horizon);
  Input4 output = Input4::Constant(123.);
  EXPECT_FALSE(backend.solve(model.equilibrium + State10::Ones(), reference, output));
  EXPECT_EQ(backend.report().status, Backend::Status::SOLVER_FAILED);
  EXPECT_EQ(backend.report().iterations, 1);
  EXPECT_TRUE(output.isConstant(123.));
  params = parameters();
  params.max_solve_time = 1e-12;
  ASSERT_TRUE(backend.initialize(model, params));
  EXPECT_FALSE(backend.solve(model.equilibrium, reference, output));
  EXPECT_EQ(backend.report().status, Backend::Status::TIME_LIMIT);
  EXPECT_GT(backend.report().solve_time, params.max_solve_time);
  EXPECT_TRUE(output.isConstant(123.));
  params.max_solve_time = 0.;
  ASSERT_TRUE(backend.initialize(model, params));
  ASSERT_TRUE(backend.solve(model.equilibrium, reference, output));
}

TEST(BipedalMpcBackend, StabilizesPhysicalTenStateModelWithWheelTorqueBounds)
{
  PhysicalModel physical;
  physical.mb = 12.;
  physical.Ib = .25;
  physical.Izz = .4;
  physical.mw = .35;
  physical.Iw = .00112;
  physical.Rw = .08;
  physical.Rl = .223;
  physical.g = 9.81;
  physical.lc = .1;
  physical.phi_c = .05;
  physical.geometry_domain << .06, .40;
  physical.legs.fill(makePhysicalLeg(1.1));
  const auto model = linearModel(physical, Eigen::Vector2d::Constant(.22));
  auto params = parameters();
  params.sample_period = .01;
  params.horizon = 21;
  params.q << 5., 5., 10., 1., 50., 2., 50., 2., 100., 5.;
  params.r << .5, .5, 1., 1.;
  params.input_min << -.5, -.5, -5., -5.;
  params.input_max << .5, .5, 5., 5.;
  params.abs_pri_tol = params.abs_dua_tol = 1e-6;
  Backend backend;
  ASSERT_TRUE(backend.initialize(model, params));
  Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(15, 15);
  augmented.topLeftCorner(10, 10) = model.A;
  augmented.block(0, 10, 10, 4) = model.B;
  augmented.block(0, 14, 10, 1) = model.residual;
  const Eigen::MatrixXd discrete = (params.sample_period * augmented).exp();
  State10 state = model.equilibrium;
  state[BODY_PITCH] += .01;
  state[S] += .02;
  state[YAW] += .02;
  const double initial_error = (state - model.equilibrium).norm();
  const Backend::StateTrajectory reference = model.equilibrium.replicate(1, params.horizon);
  for (int i = 0; i < 250; ++i)
  {
    Input4 output;
    ASSERT_TRUE(backend.solve(state, reference, output)) << "step=" << i;
    ASSERT_TRUE(output.allFinite());
    ASSERT_TRUE((output.array() >= params.input_min.array()).all());
    ASSERT_TRUE((output.array() <= params.input_max.array()).all());
    state = model.equilibrium + discrete.topLeftCorner(10, 10) * (state - model.equilibrium) +
            discrete.block(0, 10, 10, 4) * (output - model.uff) + discrete.block(0, 14, 10, 1);
    ASSERT_TRUE(state.allFinite());
    ASSERT_LT((state - model.equilibrium).norm(), 1.);
  }
  EXPECT_LT((state - model.equilibrium).norm(), .5 * initial_error);
}
}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
