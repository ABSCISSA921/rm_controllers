#include "bipedal_wheel_controller/mpc/mpc_backend.h"
#include "bipedal_wheel_controller/dynamics/model10.h"

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <tinympc/tiny_api.hpp>

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

struct OfficialSolver
{
  TinySolver* value{ nullptr };

  ~OfficialSolver()
  {
    if (value)
    {
      delete value->solution;
      delete value->settings;
      delete value->cache;
      delete value->work;
      delete value;
    }
  }
};

Input4 officialOutput(const LinearModel& model, const Backend::Params& params, const State10& state,
                      const Backend::StateTrajectory& reference)
{
  const double a = std::exp(-.7 * params.sample_period), b = (1. - a) / .7;
  const tinyMatrix ad = a * tinyMatrix::Identity(10, 10);
  tinyMatrix bd = tinyMatrix::Zero(10, 4);
  bd.topRows(4) = b * tinyMatrix::Identity(4, 4);
  OfficialSolver owner;
  if (tiny_setup(&owner.value, ad, bd, b * model.residual, params.q.asDiagonal(), params.r.asDiagonal(), params.rho, 10,
                 4, params.horizon, 0) != 0)
    throw std::runtime_error("Official TinyMPC setup failed");
  auto* solver = owner.value;
  solver->settings->max_iter = params.max_iter;
  solver->settings->abs_pri_tol = params.abs_pri_tol;
  solver->settings->abs_dua_tol = params.abs_dua_tol;
  solver->settings->en_state_bound = 0;
  solver->settings->en_input_bound = 1;
  const tinyMatrix state_bounds = tinyMatrix::Zero(10, params.horizon);
  if (tiny_set_bound_constraints(solver, state_bounds, state_bounds,
                                 (params.input_min - model.uff).replicate(1, params.horizon - 1),
                                 (params.input_max - model.uff).replicate(1, params.horizon - 1)) != 0 ||
      tiny_set_x0(solver, state - model.equilibrium) != 0 ||
      tiny_set_x_ref(solver, reference.colwise() - model.equilibrium) != 0 ||
      tiny_set_u_ref(solver, tinyMatrix::Zero(4, params.horizon - 1)) != 0 || tiny_solve(solver) != 0 ||
      solver->solution->solved != 1)
    throw std::runtime_error("Official TinyMPC solve failed");
  return model.uff + solver->solution->u.col(0);
}

TEST(BipedalMpcBackend, MatchesOfficialSolverWithNonzeroReferencesAndActiveTorqueBounds)
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
      const Input4 expected = officialOutput(model, params, state, reference);
      EXPECT_TRUE(output.isApprox(expected, 2e-7));
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
  EXPECT_TRUE(output.isApprox(officialOutput(model, params, state, reference), 2e-7));
}

TEST(BipedalMpcBackend, LimitsWheelsWhileLeavingVirtualLegTorquesUnbounded)
{
  const auto model = simpleModel();
  auto params = parameters();
  const double inf = std::numeric_limits<double>::infinity();
  params.input_min << -3.58, -3.58, -inf, -inf;
  params.input_max << 3.58, 3.58, inf, inf;
  Backend backend;
  ASSERT_TRUE(backend.initialize(model, params));
  State10 state = model.equilibrium;
  state.head<4>() += (Input4() << 100., -100., 100., -100.).finished();
  const Backend::StateTrajectory reference = model.equilibrium.replicate(1, params.horizon);
  Input4 output;
  ASSERT_TRUE(backend.solve(state, reference, output));
  EXPECT_NEAR(output[0], -3.58, 1e-8);
  EXPECT_NEAR(output[1], 3.58, 1e-8);
  EXPECT_GT(std::abs(output[2]), 3.58);
  EXPECT_GT(std::abs(output[3]), 3.58);
  EXPECT_TRUE(output.isApprox(officialOutput(model, params, state, reference), 2e-7));
  EXPECT_FALSE(backend.setInputBounds(Input4::Constant(-inf), Input4::Constant(-inf)));
  Input4 invalid = params.input_max;
  invalid[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(backend.setInputBounds(params.input_min, invalid));
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
  invalid[11].input_min[0] = std::numeric_limits<double>::infinity();
  Backend backend;
  for (const auto& params : invalid)
  {
    EXPECT_FALSE(backend.initialize(model, params));
    EXPECT_FALSE(backend.initialized());
    EXPECT_EQ(backend.report().status, Backend::Status::INVALID_INPUT);
  }
  ASSERT_TRUE(backend.initialize(model, good));
  auto bad_model = model;
  bad_model.uff[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(backend.initialize(bad_model, good));
  EXPECT_FALSE(backend.initialized());
  bad_model = model;
  bad_model.A(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(backend.initialize(bad_model, good));
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
  EXPECT_TRUE(output.isApprox(officialOutput(model, params, model.equilibrium, reference), 2e-7));
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
  EXPECT_TRUE(output.isApprox(officialOutput(model, params, state, reference), 2e-7));
  params.input_min.setConstant(.1);
  params.input_max.setConstant(.15);
  ASSERT_TRUE(backend.setInputBounds(params.input_min, params.input_max));
  EXPECT_EQ(backend.report().iterations, 0);
  ASSERT_TRUE(backend.solve(state, reference, output));
  EXPECT_TRUE(output.isApprox(officialOutput(model, params, state, reference), 2e-7));
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
