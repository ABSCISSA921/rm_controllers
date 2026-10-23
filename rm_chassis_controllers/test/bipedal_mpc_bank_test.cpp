#include <gtest/gtest.h>
#include "bipedal_wheel_controller/controller.h"

#include <chrono>
#include <cmath>
#include <limits>

namespace rm_chassis_controllers
{
namespace
{
PhysicalModel physicalModel()
{
  PhysicalModel model;
  model.mb = 17.;
  model.Ib = .38;
  model.Izz = .5282;
  model.mw = .80357;
  model.Iw = .00336;
  model.Rw = .06;
  model.Rl = .223;
  model.g = 9.8;
  model.lc = .05;
  model.phi_c = .31;
  model.geometry_domain << .06, .40;
  model.legs.fill(makePhysicalLeg(.8378));
  return model;
}

mpc_backend::Params backendParams()
{
  mpc_backend::Params params;
  params.sample_period = .001;
  params.max_iter = 5000;
  params.abs_pri_tol = params.abs_dua_tol = 1e-8;
  params.q << 6800., 90., 5280., 512., 55000., 1112., 55000., 1112., 110000., 30.;
  params.r << 400., 400., 7.5, 7.5;
  params.input_min << -3.58, -3.58, -INFINITY, -INFINITY;
  params.input_max << 3.58, 3.58, INFINITY, INFINITY;
  return params;
}
}  // namespace

TEST(BipedalMpcBank, RejectsInvalidGridsAndFailedInitialization)
{
  mpc_bank bank;
  const auto model = physicalModel();
  auto params = backendParams();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto& grid : std::vector<std::vector<double>>{
           {}, { .06 }, { .12, .40 }, { .06, .22 }, { .06, .22, .12, .40 }, { .06, .12, .12, .40 }, { .06, nan, .40 } })
    EXPECT_FALSE(bank.initialize(model, params, grid, 0.));
  for (double hysteresis : { -1., nan, .03 })
    EXPECT_FALSE(bank.initialize(model, params, { .06, .12, .22, .40 }, hysteresis));
  ASSERT_TRUE(bank.initialize(model, params, { .06, .40 }, 0.));
  ASSERT_NE(nullptr, bank.select(Eigen::Vector2d(.06, .40)));
  params.max_iter = 0;
  EXPECT_FALSE(bank.initialize(model, params, { .06, .40 }, 0.));
  EXPECT_EQ(0u, bank.size());
  EXPECT_EQ(nullptr, bank.active());
  EXPECT_EQ(nullptr, bank.select(Eigen::Vector2d(.06, .40)));
}

TEST(BipedalMpcBank, MatchesDirectBackendAtSymmetricAndAsymmetricNodes)
{
  const auto model = physicalModel();
  const auto params = backendParams();
  const std::vector<double> grid{ .06, .12, .22, .40 };
  mpc_bank bank;
  ASSERT_TRUE(bank.initialize(model, params, grid, .001));
  ASSERT_EQ(16u, bank.size());
  for (double left : grid)
    for (double right : grid)
    {
      auto* entry = bank.select(Eigen::Vector2d(left, right));
      ASSERT_NE(nullptr, entry);
      const auto direct_model = linearModel(model, entry->lengths);
      EXPECT_TRUE(entry->equilibrium.isApprox(direct_model.equilibrium, 1e-12));
      EXPECT_TRUE(entry->feedforward.isApprox(direct_model.uff, 1e-12));
      State10 state = entry->equilibrium;
      state[YAW] = 7.;
      mpc_backend::StateTrajectory reference = state.replicate(1, params.horizon);
      Input4 output;
      ASSERT_TRUE(entry->backend.solve(state, reference, output));
      EXPECT_LT((output - entry->feedforward).norm(), 1e-7);

      state[BODY_PITCH] += .01;
      state[YAW] += .02;
      reference.row(V).setConstant(.1);
      for (int i = 0; i < params.horizon; ++i)
        reference(S, i) = .1 * params.sample_period * i;
      ASSERT_TRUE(entry->backend.solve(state, reference, output));
      mpc_backend direct;
      ASSERT_TRUE(direct.initialize(direct_model, params));
      Input4 expected;
      ASSERT_TRUE(direct.solve(state, reference, expected));
      EXPECT_LT((output - expected).norm(), 1e-6);
      EXPECT_LE(std::abs(output[0]), 3.58);
      EXPECT_LE(std::abs(output[1]), 3.58);
    }
}

TEST(BipedalMpcBank, AppliesIndependentAxisHysteresisAndResetsRevisitedBackend)
{
  mpc_bank bank;
  const auto params = backendParams();
  ASSERT_TRUE(bank.initialize(physicalModel(), params, { .06, .12, .22, .40 }, .001));
  auto* original = bank.select(Eigen::Vector2d(.12, .22));
  ASSERT_NE(nullptr, original);
  EXPECT_EQ(original, bank.select(Eigen::Vector2d(.1705, .22)));
  auto* switched = bank.select(Eigen::Vector2d(.1711, .22));
  ASSERT_NE(nullptr, switched);
  EXPECT_DOUBLE_EQ(.22, switched->lengths[0]);
  EXPECT_EQ(switched, bank.select(Eigen::Vector2d(.1695, .22)));
  EXPECT_EQ(original, bank.select(Eigen::Vector2d(.1689, .22)));
  EXPECT_EQ(original, bank.select(Eigen::Vector2d(.12, .3105)));
  EXPECT_DOUBLE_EQ(.40, bank.select(Eigen::Vector2d(.12, .3111))->lengths[1]);

  ASSERT_EQ(original, bank.select(Eigen::Vector2d(.12, .22)));
  auto reference = original->equilibrium.replicate(1, params.horizon).eval();
  Input4 output;
  ASSERT_TRUE(original->backend.solve(original->equilibrium, reference, output));
  EXPECT_GT(original->backend.report().iterations, 0);
  EXPECT_EQ(original, bank.select(Eigen::Vector2d(.12, .22)));
  EXPECT_GT(original->backend.report().iterations, 0);
  ASSERT_NE(nullptr, bank.select(Eigen::Vector2d(.40, .06)));
  EXPECT_EQ(original, bank.select(Eigen::Vector2d(.12, .22)));
  EXPECT_EQ(0, original->backend.report().iterations);
  EXPECT_EQ(16u, bank.size());
}

TEST(BipedalMpcBank, RejectsOutOfDomainAndRestartsSelectionAfterReset)
{
  mpc_bank bank;
  ASSERT_TRUE(bank.initialize(physicalModel(), backendParams(), { .06, .12, .22, .40 }, .001));
  for (const Eigen::Vector2d& lengths : { Eigen::Vector2d(.06, .40), Eigen::Vector2d(.40, .06) })
    EXPECT_NE(nullptr, bank.select(lengths));
  for (const Eigen::Vector2d& lengths : { Eigen::Vector2d(.059, .22), Eigen::Vector2d(.22, .401),
                                          Eigen::Vector2d(NAN, .22), Eigen::Vector2d(.22, INFINITY) })
  {
    EXPECT_EQ(nullptr, bank.select(lengths));
    EXPECT_EQ(nullptr, bank.active());
  }
  auto* previous = bank.select(Eigen::Vector2d(.12, .22));
  bank.reset();
  EXPECT_EQ(nullptr, bank.active());
  EXPECT_NE(previous, bank.select(Eigen::Vector2d(.1705, .22)));
}

class BipedalObservationTest : public ::testing::Test
{
protected:
  BipedalController controller_;
  std::unique_ptr<Normal> normal_;
  control_toolbox::Pid roll_, leg_l_, leg_r_, theta_l_, theta_r_;
  double positions_[6]{}, velocities_[6]{}, efforts_[6]{}, commands_[6]{};
  std::vector<hardware_interface::JointHandle> handles_;

  void SetUp() override
  {
    auto& config = controller_.lqr_config_;
    config.model = physicalModel();
    controller_.mpc_params_ = backendParams();
    config.q = controller_.mpc_params_.q;
    config.r = controller_.mpc_params_.r;
    config.uff = staticFeedforward(config.model);
    config.wheel_radius = config.model.Rw;
    config.domain << .06, .40, .06, .40;
    controller_.chassis_state_.dt = .001;
    controller_.chassis_state_.linear_acc.z = 9.81;
    controller_.balance_mode_ = NORMAL;
    controller_.state_ = BipedalController::FOLLOW;
    controller_.complete_stand_ = true;
    controller_.model_params_ = std::make_shared<ModelParams>();
    controller_.model_params_->f_gravity = 110.;
    controller_.chassis_geometry_params_ = std::make_shared<ChassisGeometryParams>();
    controller_.chassis_geometry_params_->wheel_track = .446;
    controller_.spring_params_ = std::make_shared<SpringParams>();
    controller_.spring_params_->s2 = .0486;
    controller_.spring_params_->s3 = .20;
    controller_.spring_params_->alpha_s = .2448;
    controller_.spring_params_->f_spring = 0.;
    for (int i = 0; i < 6; ++i)
      handles_.emplace_back(hardware_interface::JointStateHandle(std::to_string(i), &positions_[i], &velocities_[i],
                                                                 &efforts_[i]),
                            &commands_[i]);
    for (auto& handle : handles_)
      controller_.joint_handles_.push_back(&handle);
    for (auto& leg : controller_.leg_state_)
    {
      leg.vmc = std::make_shared<VMC>(.21, .248);
      leg.vmc->calc_jacobian(2.7, M_PI - 2.7);
      leg.vmc->leg_pos(2.7, M_PI - 2.7);
    }
    controller_.legCmd_ = .22;
    normal_.reset(new Normal(&controller_, controller_.joint_handles_, { &leg_l_, &leg_r_ }, nullptr, nullptr, &roll_,
                             nullptr, { &theta_l_, &theta_r_ }));
    ASSERT_TRUE(controller_.mpc_bank_.initialize(config.model, controller_.mpc_params_, { .06, .12, .22, .40 }, .001));
    controller_.mpc_reference_.resize(10, controller_.mpc_params_.horizon);
    setLengths(Eigen::Vector2d(.22, .22));
  }

  void setLengths(const Eigen::Vector2d& lengths)
  {
    auto& chassis = controller_.chassis_state_;
    chassis.length = lengths;
    chassis.x = staticEquilibrium(controller_.lqr_config_.model, lengths, controller_.lqr_config_.uff);
    chassis.x[YAW] = 7.;
  }

  void execute()
  {
    normal_->execute(ros::Time(10.), ros::Duration(controller_.chassis_state_.dt));
  }

  bool setupMpc(ros::NodeHandle& nh)
  {
    return controller_.setupMpc(nh);
  }

  void rejectWeights(LQRWeightConfig& weights)
  {
    controller_.reconfigCB(weights, 0);
  }

  NormalFeedback& feedback()
  {
    return controller_.normal_feedback_;
  }

  mpc_bank& bank()
  {
    return controller_.mpc_bank_;
  }

  ChassisState& chassis()
  {
    return controller_.chassis_state_;
  }

  void limitIterations()
  {
    controller_.mpc_params_.max_iter = 1;
    ASSERT_TRUE(bank().initialize(controller_.lqr_config_.model, controller_.mpc_params_, { .06, .12, .22, .40 }, .001));
  }
};

TEST_F(BipedalObservationTest, NormalUsesSelectedTrimAndSingleFeedforwardAcrossSwitches)
{
  for (const Eigen::Vector2d& lengths :
       { Eigen::Vector2d(.12, .40), Eigen::Vector2d(.22, .12), Eigen::Vector2d(.40, .22) })
  {
    setLengths(lengths);
    execute();
    ASSERT_TRUE(feedback().active);
    ASSERT_NE(nullptr, bank().active());
    EXPECT_TRUE(bank().active()->lengths.isApprox(lengths));
    for (int index : { THETA_L, THETA_R, BODY_PITCH })
      EXPECT_DOUBLE_EQ(bank().active()->equilibrium[index], feedback().reference[index]);
    EXPECT_LT((feedback().output - bank().active()->feedforward).norm(), 1e-7);
  }
}

TEST_F(BipedalObservationTest, FailedSolveRequestsProtectAndClearsCommands)
{
  execute();
  ASSERT_TRUE(feedback().active);
  limitIterations();
  chassis().x[BODY_PITCH] += .1;
  execute();
  EXPECT_FALSE(feedback().active);
  EXPECT_EQ(PROTECT, controller_.getBalanceMode());
  EXPECT_EQ(nullptr, bank().active());
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(0., command);
}

TEST_F(BipedalObservationTest, InvalidLengthRequestsProtectAndClearsSelection)
{
  execute();
  ASSERT_TRUE(feedback().active);
  chassis().length[0] = NAN;
  execute();
  EXPECT_FALSE(feedback().active);
  EXPECT_EQ(PROTECT, controller_.getBalanceMode());
  EXPECT_EQ(nullptr, bank().active());
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(0., command);
}

TEST_F(BipedalObservationTest, JumpEntryClearsGroundSelection)
{
  execute();
  ASSERT_TRUE(feedback().active);
  controller_.setJumpCmd(true);
  execute();
  EXPECT_FALSE(feedback().active);
  EXPECT_FALSE(feedback().reference_valid);
  EXPECT_EQ(nullptr, bank().active());
  EXPECT_EQ(NORMAL, controller_.getBalanceMode());
}

TEST_F(BipedalObservationTest, RejectsOnlineWeightsAndRestoresAppliedValues)
{
  LQRWeightConfig weights;
  weights.Q_s = -10.;
  weights.R_T_L = 999.;
  rejectWeights(weights);
  EXPECT_DOUBLE_EQ(6800., weights.Q_s);
  EXPECT_DOUBLE_EQ(400., weights.R_T_L);
  EXPECT_NE(std::string::npos, weights.update_status.find("read_only"));
  execute();
  EXPECT_TRUE(feedback().active);
}

TEST_F(BipedalObservationTest, BuildsDefaultPoolAndValidatesCustomGridParameters)
{
  ros::NodeHandle nh("~bank_config");
  ASSERT_TRUE(setupMpc(nh));
  EXPECT_EQ(169u, bank().size());
  auto* entry = bank().select(Eigen::Vector2d(.12, .12));
  ASSERT_NE(nullptr, entry);
  EXPECT_DOUBLE_EQ(.12, entry->lengths[0]);
  nh.setParam("mpc/leg_lengths", std::vector<double>{ .06, .12, .22, .40 });
  ASSERT_TRUE(setupMpc(nh));
  EXPECT_EQ(16u, bank().size());
  nh.setParam("mpc/leg_lengths", std::vector<double>{ .12, .22, .40 });
  EXPECT_FALSE(setupMpc(nh));
  nh.setParam("mpc/leg_lengths", std::string("invalid"));
  EXPECT_FALSE(setupMpc(nh));
}
}  // namespace rm_chassis_controllers

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bipedal_mpc_bank_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
