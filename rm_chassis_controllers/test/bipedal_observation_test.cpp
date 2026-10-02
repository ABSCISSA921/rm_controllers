#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <limits>
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
// Exercise production methods with in-memory handles and a private rostest master.
// No RobotHW, simulation, bus, or production diagnostic publisher is involved.
class BipedalObservationTest : public ::testing::Test
{
protected:
  BipedalController controller_;
  double positions_[6]{}, velocities_[6]{}, efforts_[6]{}, commands_[6]{};
  std::vector<hardware_interface::JointHandle> handles_;
  control_toolbox::Pid roll_{1500., 0., 35., 30., -30.};
  control_toolbox::Pid leg_l_{1000., 100., 60., 10., -10.}, leg_r_{1000., 100., 60., 10., -10.};
  control_toolbox::Pid theta_l_, theta_r_;
  std::unique_ptr<Normal> normal_;
  std::unique_ptr<RampFilter<double>> ramp_x_, ramp_y_, ramp_w_;

  void SetUp() override
  {
    ros::Time::init();
    for (int i = 0; i < 6; ++i)
      handles_.emplace_back(hardware_interface::JointStateHandle(std::to_string(i), &positions_[i],
                                                               &velocities_[i], &efforts_[i]), &commands_[i]);
    for (auto &handle : handles_)
      controller_.joint_handles_.push_back(&handle);
    for (int side = 0; side < 2; ++side)
    {
      auto &vmc = controller_.leg_state_[side].vmc;
      vmc = std::make_shared<VMC>(.21, .248);
      vmc->calc_jacobian(2.7, M_PI - 2.7);
      vmc->leg_pos(2.7, M_PI - 2.7);
    }
    auto &config = controller_.lqr_config_;
    config.wheel_radius = .08;
    config.domain << .06, .40, .06, .40;
    config.max_angle = .2;
    config.position_release_tau = .2;
    config.hold_capture_speed = .1;
    // Simple valid trim and constant gains, only for exercising the real Normal path.
    config.model.geometry_domain << .06, .40;
    config.model.mb = 10.;
    config.model.g = 9.8;
    for (auto &leg : config.model.legs)
    {
      leg.mass = 1.;
      leg.lb[0] = .1;
      leg.lw[0] = .2;
      leg.inertia[0] = .01;
    }
    config.coeffs(0, 4 * lqr10::V) = config.coeffs(0, 1 + 4 * lqr10::V) = 1.;
    controller_.model_params_ = std::make_shared<ModelParams>();
    controller_.model_params_->M = 12.;
    controller_.model_params_->f_gravity = 110.;
    controller_.chassis_geometry_params_ = std::make_shared<ChassisGeometryParams>();
    controller_.chassis_geometry_params_->wheel_track = .446;
    controller_.legCmd_ = .15;
    controller_.balance_mode_ = NORMAL;
    controller_.complete_stand_ = true;
    ASSERT_TRUE(controller_.velocity_kf_.configure(VelocityKalmanFilter::Params{}));
    normal_.reset(new Normal(&controller_, controller_.joint_handles_, {&leg_l_, &leg_r_},
                             nullptr, nullptr, &roll_, nullptr, {&theta_l_, &theta_r_}));
    state().linear_acc.z = 9.81;
  }

  ChassisState &state() { return controller_.chassis_state_; }
  lqr10::ObservationHistory &history() { return controller_.observation_history_; }
  lqr10::NormalFeedback &feedback() { return controller_.normal_feedback_; }
  bool observe(double time, double dt = .001)
  {
    controller_.resetObservation(ros::Time(time), ros::Duration(dt));
    state().raw_valid = true;
    for (int side = 0; side < 2; ++side)
      state().length[side] = controller_.leg_state_[side].vmc->getPos().L0;
    const double pitch_rate = state().angular_vel.y * std::cos(state().roll) -
                              state().angular_vel.z * std::sin(state().roll);
    return controller_.observe(pitch_rate, Eigen::Vector2d::Zero());
  }
  void integrate() { controller_.integrateVelocity(); }
  void finish() { controller_.finishObservation(); }
  bool ground() { return controller_.updateGroundVelocity(); }
  void execute() { normal_->execute(ros::Time(state().time), ros::Duration(state().dt)); }
  void reenter() { controller_.setStateChange(false); }
  void jump() { controller_.setJumpCmd(true); }
  void overturn() { controller_.overturn_ = true; }
  void invalidCycle()
  {
    controller_.command_authorized_ = true;
    controller_.last_status_time_ = 10.; // No publishers in this offline fixture.
    controller_.moveJoint(ros::Time(10.), ros::Duration(.001));
  }
  void restart()
  {
    ros::NodeHandle nh("~restart");
    controller_.mode_manager_ = std::make_shared<ModeManager>(&controller_, nh, controller_.joint_handles_);
    ramp_x_.reset(new RampFilter<double>(1., .001));
    ramp_y_.reset(new RampFilter<double>(1., .001));
    ramp_w_.reset(new RampFilter<double>(1., .001));
    controller_.ramp_x_ = ramp_x_.get();
    controller_.ramp_y_ = ramp_y_.get();
    controller_.ramp_w_ = ramp_w_.get();
    controller_.starting(ros::Time(20.));
    EXPECT_FALSE(controller_.command_authorized_);
    EXPECT_EQ(controller_.state_, rm_msgs::ChassisCmd::FALLEN);
  }
  double rollDerivative()
  {
    double p, i, d;
    roll_.getCurrentPIDErrors(&p, &i, &d);
    return d;
  }
};

TEST_F(BipedalObservationTest, JitterKeepsNormalReferencePidAndIntegration)
{
  velocities_[4] = velocities_[5] = 2.;
  ASSERT_TRUE(observe(10.));
  execute();
  ASSERT_TRUE(feedback().active);
  feedback().reference[lqr10::YAW] = .7;
  double t = 10.;
  for (double interval : {.001, .001023, .000977, .001023, .000977})
  {
    t += interval;
    ASSERT_TRUE(observe(t)); // Nominal host step stays 1 ms.
    const double previous_s = history().s;
    double p, old_i, d, new_i;
    leg_l_.getCurrentPIDErrors(&p, &old_i, &d);
    execute();
    leg_l_.getCurrentPIDErrors(&p, &new_i, &d);
    EXPECT_LT(new_i, old_i); // No spurious Normal reentry/PID reset.
    EXPECT_GT(history().s, previous_s);
    EXPECT_DOUBLE_EQ(feedback().reference[lqr10::YAW], .7);
    EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
    EXPECT_TRUE(feedback().active);
    EXPECT_FALSE(feedback().faulted);
    EXPECT_NE(commands_[4], 0.);
    finish();
  }
}

TEST_F(BipedalObservationTest, JitterDoesNotEraseCapturedPositionAnchor)
{
  ASSERT_TRUE(observe(10.));
  execute();
  ASSERT_TRUE(feedback().active);
  feedback().reference[lqr10::S] = -.01;
  ASSERT_TRUE(observe(10.001023));
  execute();
  EXPECT_DOUBLE_EQ(feedback().reference[lqr10::S], -.01);
  EXPECT_TRUE(controller_.getCompleteStand());
}

TEST_F(BipedalObservationTest, SuppliedStepControlsIntegralNotRosLabels)
{
  velocities_[4] = velocities_[5] = 2.;
  ASSERT_TRUE(observe(10.));
  integrate();
  double expected = 0., t = 10.;
  for (double dt : {.001, .001023, .000977})
  {
    t += .001; // A host may instead supply actual positive intervals.
    ASSERT_TRUE(observe(t, dt));
    integrate();
    expected += .16 * dt;
    EXPECT_NEAR(history().s, expected, 1e-12);
  }
}

TEST_F(BipedalObservationTest, JitterDoesNotReseedVelocityFilter)
{
  ASSERT_TRUE(observe(10.));
  ASSERT_TRUE(ground());
  velocities_[4] = velocities_[5] = 10.;
  ASSERT_TRUE(observe(10.001023));
  ASSERT_TRUE(ground());
  EXPECT_GT(state().x[lqr10::V], 0.);
  EXPECT_LT(state().x[lqr10::V], .8); // Reseeding would return the raw .8 m/s.
}

TEST_F(BipedalObservationTest, RepeatedAndBackwardLabelsKeepCurrentObservationAndSpatialHistory)
{
  velocities_[4] = velocities_[5] = 2.;
  state().yaw = 3.13;
  ASSERT_TRUE(observe(10.));
  execute();
  const double saved_s = history().s;
  feedback().reference[lqr10::YAW] = .7;
  for (double t : {10., 9.})
  {
    state().yaw = -3.13;
    ASSERT_TRUE(observe(t));
    EXPECT_FALSE(history().initialized);
    execute();
    EXPECT_DOUBLE_EQ(history().s, saved_s); // No trapezoid across the label discontinuity.
    EXPECT_NEAR(state().x[lqr10::YAW], 2 * M_PI - 3.13, 1e-12);
    EXPECT_DOUBLE_EQ(feedback().reference[lqr10::YAW], .7);
    EXPECT_TRUE(feedback().active);
    EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
  }
}

TEST_F(BipedalObservationTest, ForwardRosJumpIsNotIntegratedAsElapsedTime)
{
  velocities_[4] = velocities_[5] = 2.;
  ASSERT_TRUE(observe(10.));
  integrate();
  ASSERT_TRUE(observe(1000.));
  integrate();
  EXPECT_NEAR(history().s, .00016, 1e-12);
}

TEST_F(BipedalObservationTest, MissingObservationBreaksOnlyNecessaryHistory)
{
  velocities_[4] = velocities_[5] = 2.;
  ASSERT_TRUE(observe(10.));
  integrate();
  ASSERT_TRUE(observe(10.001));
  integrate();
  const double saved_s = history().s;
  velocities_[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(observe(10.002));
  finish();
  EXPECT_FALSE(history().initialized);
  EXPECT_TRUE(history().ever_initialized);
  EXPECT_DOUBLE_EQ(history().s, saved_s);
  velocities_[4] = 2.;
  ASSERT_TRUE(observe(30.)); // Explicitly observed feedback outage; no catch-up integration.
  integrate();
  EXPECT_DOUBLE_EQ(history().s, saved_s);
}

TEST_F(BipedalObservationTest, InvalidObservationLatchesInsteadOfRequestingProtect)
{
  velocities_[4] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_FALSE(observe(10.));
  execute();
  EXPECT_TRUE(feedback().faulted);
  EXPECT_EQ(feedback().reason, lqr10::Reason::InvalidSnapshot);
  EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
}

TEST_F(BipedalObservationTest, InvalidInputStillZerosAllSixCommands)
{
  for (double &command : commands_)
    command = 12.;
  velocities_[4] = std::numeric_limits<double>::quiet_NaN();
  invalidCycle(); // Production moveJoint, including the existing final zero-output path.
  EXPECT_TRUE(feedback().faulted);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
}

TEST_F(BipedalObservationTest, InvalidPeriodIsNotAPostureAction)
{
  for (double dt : {0., -.001})
  {
    ASSERT_FALSE(observe(10., dt));
    EXPECT_EQ(state().observation_reason, lqr10::Reason::InvalidTime);
    execute();
    EXPECT_TRUE(feedback().faulted);
    EXPECT_EQ(feedback().reason, lqr10::Reason::InvalidTime);
    EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
  }
}

TEST_F(BipedalObservationTest, GenuinePostureAndOverturnStillRouteToActions)
{
  state().pitch = 1.3;
  ASSERT_FALSE(observe(10.));
  EXPECT_EQ(state().observation_reason, lqr10::Reason::Posture);
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), PROTECT);
  EXPECT_FALSE(feedback().faulted);
  overturn();
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), RECOVER);
}

TEST_F(BipedalObservationTest, RollReentryHasProportionalForceWithoutFictitiousDerivative)
{
  state().roll = .06838551517;
  for (int entry = 0; entry < 3; ++entry)
  {
    reenter();
    ASSERT_TRUE(observe(10. + .001 * entry));
    execute();
    ASSERT_TRUE(feedback().active);
    EXPECT_NEAR(roll_.getCurrentCmd(), -102.578272755, 1e-9);
    EXPECT_DOUBLE_EQ(rollDerivative(), 0.);
  }
}

TEST_F(BipedalObservationTest, RealRollMotionKeepsDampingOnGroundAndJumpEntry)
{
  state().roll = .06838551517;
  state().angular_vel.x = .2;
  ASSERT_TRUE(observe(10.));
  execute();
  EXPECT_NEAR(roll_.getCurrentCmd(), -102.578272755 - 7., 1e-9);
  EXPECT_NEAR(rollDerivative(), -.2, 1e-12);
  reenter();
  jump();
  ASSERT_TRUE(observe(10.001));
  execute();
  EXPECT_NEAR(roll_.getCurrentCmd(), -102.578272755 - 7., 1e-9);
  EXPECT_NEAR(rollDerivative(), -.2, 1e-12);
}

TEST_F(BipedalObservationTest, ProtectToNormalResetsLegIntegralWithoutRollKick)
{
  state().roll = .06838551517;
  ASSERT_TRUE(observe(10.));
  execute();
  control_toolbox::Pid wheel_l, wheel_r, theta_diff, yaw;
  std::vector<hardware_interface::JointHandle *> joints;
  for (auto &handle : handles_)
    joints.push_back(&handle);
  Protect protect(&controller_, joints, {&leg_l_, &leg_r_}, {&theta_l_, &theta_r_},
                  {&wheel_l, &wheel_r}, &theta_diff, &yaw);
  controller_.setMode(PROTECT);
  reenter();
  ASSERT_TRUE(observe(10.001));
  protect.execute(ros::Time(10.001), ros::Duration(.001));
  ASSERT_EQ(controller_.getBalanceMode(), NORMAL);
  ASSERT_FALSE(controller_.getStateChange());
  // Deliberately leave a prior mechanical-task integral; Normal must reset it.
  leg_l_.computeCommand(-.1, 0., ros::Duration(.2));
  ASSERT_TRUE(observe(10.002));
  execute();
  EXPECT_TRUE(feedback().active);
  EXPECT_NEAR(roll_.getCurrentCmd(), -102.578272755, 1e-9);
  EXPECT_DOUBLE_EQ(rollDerivative(), 0.);
  double p, i, d;
  leg_l_.getCurrentPIDErrors(&p, &i, &d);
  EXPECT_NEAR(i, p * .001, 1e-12); // Only this Normal cycle's integral remains.
  EXPECT_DOUBLE_EQ(d, 0.); // Measured length derivative still in use.
  EXPECT_NEAR(feedback().axial_force[0] - feedback().axial_force[1], 2. * roll_.getCurrentCmd(), 1e-9);
  controller_.stopping(ros::Time(10.003));
  EXPECT_TRUE(feedback().faulted);
  EXPECT_EQ(feedback().reason, lqr10::Reason::Terminated);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
}

TEST_F(BipedalObservationTest, ExplicitStopStartStillClearsHistoryAndRequiresFreshCommand)
{
  state().roll = .06838551517;
  ASSERT_TRUE(observe(10.));
  execute();
  feedback().reference[lqr10::YAW] = .7;
  history().s = 1.;
  controller_.stopping(ros::Time(10.001));
  restart();
  EXPECT_FALSE(feedback().faulted);
  EXPECT_FALSE(feedback().reference_valid);
  EXPECT_FALSE(history().initialized);
  EXPECT_FALSE(history().ever_initialized);
  EXPECT_DOUBLE_EQ(history().s, 0.);
  EXPECT_EQ(controller_.getBalanceMode(), SIT_DOWN);
  EXPECT_FALSE(controller_.getCompleteStand());
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
  // Exercise the later Normal entry separately; this is not a stand-up simulation.
  controller_.setMode(NORMAL);
  ASSERT_TRUE(observe(20.001));
  execute();
  EXPECT_TRUE(feedback().reference_valid);
  EXPECT_DOUBLE_EQ(feedback().reference[lqr10::YAW], state().x[lqr10::YAW]);
  EXPECT_NEAR(roll_.getCurrentCmd(), -102.578272755, 1e-9);
  EXPECT_DOUBLE_EQ(rollDerivative(), 0.);
}

TEST_F(BipedalObservationTest, EulerRollRateMatchesRotatedBodyGyro)
{
  state().roll = .25;
  state().pitch = .3;
  state().yaw = -.6;
  const Eigen::Vector3d omega(.2, .3, -.4);
  state().angular_vel.x = omega.x();
  state().angular_vel.y = omega.y();
  state().angular_vel.z = omega.z();
  ASSERT_TRUE(observe(10.));
  const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(state().yaw, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(state().pitch, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(state().roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const double dt = 1e-7;
  const Eigen::Matrix3d next = rotation * Eigen::AngleAxisd(omega.norm() * dt, omega.normalized()).toRotationMatrix();
  const double numerical = (std::atan2(next(2, 1), next(2, 2)) - state().roll) / dt;
  EXPECT_NEAR(state().roll_rate, numerical, 1e-7);
  EXPECT_GT(std::abs(state().roll_rate - omega.x()), .05);
}
} // namespace rm_chassis_controllers

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bipedal_observation_test");
  ros::NodeHandle keep_alive;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
