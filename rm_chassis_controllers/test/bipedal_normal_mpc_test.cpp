#include <gtest/gtest.h>
#include "bipedal_wheel_controller/controller.h"
#include <unsupported/Eigen/MatrixFunctions>

#include <limits>

namespace rm_chassis_controllers
{
class BipedalNormalMpcTest : public ::testing::Test
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
    config.domain << .06, .40, .06, .40;
    auto& physical = config.model;
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
    config.uff = staticFeedforward(physical);
    config.q << 5., 5., 10., 1., 50., 2., 50., 2., 100., 5.;
    config.r << .5, .5, 1., 1.;
    config.coeffs.setConstant(std::numeric_limits<double>::quiet_NaN());
    config.wheel_radius = physical.Rw;
    controller_.chassis_state_.length.setConstant(.22);
    controller_.chassis_state_.dt = .001;
    controller_.chassis_state_.x = linearModel(physical, controller_.chassis_state_.length).equilibrium;
    controller_.chassis_state_.x[BODY_PITCH] += .01;
    controller_.balance_mode_ = NORMAL;
    controller_.state_ = BipedalController::FOLLOW;
    controller_.complete_stand_ = true;
    controller_.balance_state_changed_ = true;
    controller_.model_params_ = std::make_shared<ModelParams>();
    controller_.model_params_->f_gravity = 110.;
    controller_.chassis_geometry_params_ = std::make_shared<ChassisGeometryParams>();
    controller_.chassis_geometry_params_->wheel_track = .446;
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
    startBackend();
  }

  void startBackend(double solve_budget = 0., int max_iter = 10000)
  {
    auto& params = controller_.mpc_params_;
    params.sample_period = state().dt;
    params.horizon = 21;
    params.max_iter = max_iter;
    params.abs_pri_tol = params.abs_dua_tol = 1e-7;
    params.max_solve_time = solve_budget;
    params.q = controller_.lqr_config_.q;
    params.r = controller_.lqr_config_.r;
    const double inf = std::numeric_limits<double>::infinity();
    params.input_min << -3.58, -3.58, -inf, -inf;
    params.input_max << 3.58, 3.58, inf, inf;
    controller_.mpc_.reset(new mpc_backend);
    controller_.mpc_lengths_ = state().length;
    ASSERT_TRUE(controller_.mpc_->initialize(linearModel(controller_.lqr_config_.model, state().length), params));
    controller_.mpc_model_valid_ = true;
    controller_.mpc_reference_.resize(10, params.horizon);
  }

  bool compute(double velocity = 0., double yaw_rate = 0.)
  {
    return normal_->computeFeedback(velocity, yaw_rate);
  }
  NormalFeedback& feedback()
  {
    return controller_.normal_feedback_;
  }
  ChassisState& state()
  {
    return controller_.chassis_state_;
  }
  void reset()
  {
    normal_->resetMpc();
  }
  void integrate()
  {
    controller_.integrateVelocity();
  }
  void execute()
  {
    normal_->execute(ros::Time(10.), ros::Duration(state().dt));
  }
  void requestMode(int mode)
  {
    normal_->requestMode(mode);
  }
  void reenter()
  {
    controller_.setMode(NORMAL);
    controller_.setStateChange(false);
  }
  void jump()
  {
    controller_.setJumpCmd(true);
  }
  void setVelocity(double v)
  {
    controller_.vel_cmd_.x = v;
  }
  void setWeights(const State10& q)
  {
    controller_.lqr_config_.q = q;
  }
  const mpc_backend::Params& params()
  {
    return controller_.mpc_params_;
  }
  bool configureDefaultBackend()
  {
    ros::NodeHandle nh("~mpc_defaults");
    return controller_.setupMpc(nh);
  }

  Input4 directOutput()
  {
    const auto model = linearModel(controller_.lqr_config_.model, state().length);
    auto params = controller_.mpc_params_;
    params.q = controller_.lqr_config_.q;
    params.r = controller_.lqr_config_.r;
    mpc_backend backend;
    EXPECT_TRUE(backend.initialize(model, params));
    mpc_backend::StateTrajectory trajectory = feedback().reference.replicate(1, params.horizon);
    for (int i = 0; i < params.horizon; ++i)
    {
      trajectory(S, i) += feedback().reference[V] * params.sample_period * i;
      trajectory(YAW, i) += feedback().reference[YAW_RATE] * params.sample_period * i;
    }
    Input4 output = Input4::Zero();
    EXPECT_TRUE(backend.solve(state().x, trajectory, output));
    return output;
  }
};

TEST_F(BipedalNormalMpcTest, UsesCurrentObservationAndReturnsFeedforwardExactlyOnce)
{
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
  EXPECT_GT((feedback().output - controller_.getLqrConfig().uff).norm(), 1e-3);
  state().x[BODY_PITCH] = -.02;
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
}

TEST_F(BipedalNormalMpcTest, DrivingKeepsZeroPositionOriginAndAdvancesOnlyPredictionAndYaw)
{
  for (int i = 0; i < 10; ++i)
    ASSERT_TRUE(compute(.4, -.3));
  EXPECT_DOUBLE_EQ(feedback().reference[S], 0.);
  EXPECT_NEAR(feedback().reference[YAW], -.003, 1e-12);
  EXPECT_DOUBLE_EQ(feedback().reference[V], .4);
  EXPECT_DOUBLE_EQ(feedback().reference[YAW_RATE], -.3);
  EXPECT_TRUE(feedback().active);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
  state().x[S] = .2;
  state().x[V] = 2.;
  setVelocity(2.);
  integrate();
  EXPECT_DOUBLE_EQ(state().x[S], 0.);
}

TEST_F(BipedalNormalMpcTest, ParkingIntegratesDriftWithoutMovingThePositionTarget)
{
  ASSERT_TRUE(compute());
  state().x[S] = .02;
  state().x[V] = .1;
  setVelocity(0.);
  integrate();
  EXPECT_NEAR(state().x[S], .0201, 1e-12);
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_DOUBLE_EQ(feedback().reference[S], 0.);
  EXPECT_NEAR(state().x[S], .0201, 1e-12);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
  state().x[V] = .6;
  integrate();
  EXPECT_DOUBLE_EQ(state().x[S], 0.);
  state().x[S] = .02;
  state().x[V] = .1;
  controller_.setMode(NORMAL);
  setVelocity(.02);
  integrate();
  EXPECT_DOUBLE_EQ(state().x[S], 0.);
}

TEST_F(BipedalNormalMpcTest, RebuildsForLengthsWeightsAndActualControlPeriod)
{
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  reset();
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
  state().length << .25, .24;
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
  const State10 q = 2. * controller_.getLqrConfig().q;
  setWeights(q);
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
  state().dt = .005;
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_DOUBLE_EQ(params().sample_period, .005);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
}

TEST_F(BipedalNormalMpcTest, DefaultSynchronousFeedbackStabilizesTheLinearPlant)
{
  ASSERT_TRUE(configureDefaultBackend());
  state().dt = .01;
  const auto model = linearModel(controller_.getLqrConfig().model, state().length);
  ASSERT_TRUE(compute());
  const State10 reference = feedback().reference;
  state().x[S] += .02;
  state().x[YAW] += .02;
  const double initial_error = (state().x - reference).norm();
  Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(15, 15);
  augmented.topLeftCorner(10, 10) = model.A;
  augmented.block(0, 10, 10, 4) = model.B;
  augmented.block(0, 14, 10, 1) = model.residual;
  const Eigen::MatrixXd discrete = (state().dt * augmented).exp();
  for (int i = 0; i < 250; ++i)
  {
    ASSERT_TRUE(compute()) << "step=" << i;
    ASSERT_TRUE(feedback().active) << "step=" << i;
    ASSERT_LE(feedback().output.head<2>().cwiseAbs().maxCoeff(), 3.58);
    state().x = model.equilibrium + discrete.topLeftCorner(10, 10) * (state().x - model.equilibrium) +
                discrete.block(0, 10, 10, 4) * (feedback().output - model.uff) + discrete.block(0, 14, 10, 1);
    ASSERT_TRUE(state().x.allFinite());
    ASSERT_LT((state().x - reference).norm(), 1.);
  }
  EXPECT_LT((state().x - reference).norm(), .5 * initial_error);
}

TEST_F(BipedalNormalMpcTest, LateSolveRequestsProtectionAndClearsAllJointCommands)
{
  startBackend(1e-12);
  feedback().output.setConstant(123.);
  EXPECT_FALSE(compute());
  EXPECT_FALSE(feedback().active);
  EXPECT_TRUE(feedback().output.isConstant(123.));
  for (double& command : commands_)
    command = 42.;
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), PROTECT);
  EXPECT_FALSE(feedback().active);
  EXPECT_FALSE(feedback().reference_valid);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
}

TEST_F(BipedalNormalMpcTest, UnconvergedSolveRequestsProtectionAndClearsAllJointCommands)
{
  startBackend(0., 1);
  state().x[BODY_PITCH] = .1;
  for (double& command : commands_)
    command = 42.;
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), PROTECT);
  EXPECT_FALSE(feedback().active);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
}

TEST_F(BipedalNormalMpcTest, FailedModelRebuildDoesNotUseThePreviousSolver)
{
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  state().dt = std::numeric_limits<double>::max();
  EXPECT_FALSE(compute());
  EXPECT_FALSE(feedback().active);
  state().dt = .001;
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  EXPECT_TRUE(feedback().output.isApprox(directOutput(), 1e-5));
}

TEST_F(BipedalNormalMpcTest, FailedModelInitializationRequestsProtection)
{
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  State10 invalid_q = controller_.getLqrConfig().q;
  invalid_q[0] = -1.;
  setWeights(invalid_q);
  for (double& command : commands_)
    command = 42.;
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), PROTECT);
  EXPECT_FALSE(feedback().active);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
}

TEST_F(BipedalNormalMpcTest, ModeExitReentryAndJumpResetGroundFeedback)
{
  ASSERT_TRUE(compute());
  ASSERT_TRUE(feedback().active);
  requestMode(SIT_DOWN);
  EXPECT_FALSE(feedback().active);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
  reenter();
  execute();
  EXPECT_TRUE(feedback().active);
  jump();
  execute();
  EXPECT_FALSE(feedback().active);
}

TEST_F(BipedalNormalMpcTest, InvalidObservationRequestsProtectionBeforeWritingVmcCommands)
{
  state().x[V] = std::numeric_limits<double>::quiet_NaN();
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), PROTECT);
  EXPECT_FALSE(feedback().active);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "bipedal_normal_mpc_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
