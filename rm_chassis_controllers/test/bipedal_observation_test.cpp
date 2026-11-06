#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
// Exercise production methods with in-memory handles and a private rostest master.
// No RobotHW, simulation, or bus is involved. Status tests use only private ROS topics.
class BipedalObservationTest : public ::testing::Test
{
protected:
  BipedalController controller_;
  double positions_[6]{}, velocities_[6]{}, efforts_[6]{}, commands_[6]{};
  double orientation_[4]{0., 0., 0., 1.}, gyro_[3]{}, acceleration_[3]{0., 0., 9.81};
  tf2_ros::Buffer transforms_;
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
    controller_.left_hip_joint_handle_ = handles_[0];
    controller_.left_knee_joint_handle_ = handles_[1];
    controller_.right_hip_joint_handle_ = handles_[2];
    controller_.right_knee_joint_handle_ = handles_[3];
    controller_.left_wheel_joint_handle_ = handles_[4];
    controller_.right_wheel_joint_handle_ = handles_[5];
    controller_.imu_handle_ = hardware_interface::ImuSensorHandle(
        "test_imu", "test_imu_frame", orientation_, nullptr, gyro_, nullptr, acceleration_, nullptr);
    controller_.robot_state_handle_ = rm_control::RobotStateHandle("test_state", &transforms_);
    positions_[0] = positions_[2] = 2.7 - M_PI;
    positions_[1] = positions_[3] = M_PI - 2.7;
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
    config.coeffs(0, 4 * V) = config.coeffs(0, 1 + 4 * V) = 1.;
    controller_.model_params_ = std::make_shared<ModelParams>();
    controller_.model_params_->M = 12.;
    controller_.model_params_->f_gravity = 110.;
    controller_.chassis_geometry_params_ = std::make_shared<ChassisGeometryParams>();
    controller_.chassis_geometry_params_->wheel_track = .446;
    controller_.legCmd_ = .15;
    controller_.balance_mode_ = NORMAL;
    controller_.complete_stand_ = true;
    controller_.velocity_kf_.configure(VelocityKalmanFilter::Params{});
    stageGain(config.coeffs, config.q, config.r);
    normal_.reset(new Normal(&controller_, controller_.joint_handles_, {&leg_l_, &leg_r_},
                             nullptr, nullptr, &roll_, nullptr, {&theta_l_, &theta_r_}));
    state().linear_acc.z = 9.81;
    prepareCycle();
  }

  ChassisState &state() { return controller_.chassis_state_; }
  NormalFeedback &feedback() { return controller_.normal_feedback_; }
  Config &config() { return controller_.lqr_config_; }
  void legCommand(double length) { controller_.legCmd_ = length; }
  double observation_time_{0.};
  void observe(double time, double dt = .001)
  {
    observation_time_ = time;
    state().dt = dt;
    for (int side = 0; side < 2; ++side)
      state().length[side] = controller_.leg_state_[side].vmc->getPos().L0;
    const double pitch_rate = state().angular_vel.y * std::cos(state().roll) -
                              state().angular_vel.z * std::sin(state().roll);
    controller_.observe(pitch_rate, Eigen::Vector2d::Zero());
  }
  void baseMode(int mode) { controller_.state_ = mode; }
  void velocityCommand(double velocity) { controller_.vel_cmd_.x = velocity; }
  void integrate() { controller_.integrateVelocity(); }
  void ground() { controller_.updateGroundVelocity(); }
  void stageGain(const Eigen::Matrix<double, 10, 40> &coeffs, const State10 &q, const Input4 &r)
  {
    GainUpdate update;
    update.coeffs = coeffs;
    update.q = q;
    update.r = r;
    controller_.gain_update_buffer_.writeFromNonRT(update);
  }
  void execute() { normal_->execute(ros::Time(observation_time_), ros::Duration(state().dt)); }
  void reenter() { controller_.setStateChange(false); }
  void jump() { controller_.setJumpCmd(true); }
  void overturn() { controller_.overturn_ = true; }
  void prepareCycle()
  {
    ros::NodeHandle nh("~cycle");
    controller_.mode_manager_ = std::make_shared<ModeManager>(&controller_, nh, controller_.joint_handles_);
    ramp_x_.reset(new RampFilter<double>(1., .001));
    ramp_y_.reset(new RampFilter<double>(1., .001));
    ramp_w_.reset(new RampFilter<double>(1., .001));
    controller_.ramp_x_ = ramp_x_.get();
    controller_.ramp_y_ = ramp_y_.get();
    controller_.ramp_w_ = ramp_w_.get();
    controller_.legged_chassis_mode_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>(nh, "mode", 1));
    controller_.unstick_pub_ = nh.advertise<std_msgs::Bool>("unstick", 1);
    controller_.state_ = rm_msgs::ChassisCmd::FOLLOW;
  }
  void missingTfCycle()
  {
    controller_.last_status_time_ = 10.; // Compatibility publishers are prepared only by status tests.
    controller_.moveJoint(ros::Time(10.), ros::Duration(.001));
  }
  void restart()
  {
    prepareCycle();
    controller_.starting(ros::Time(20.));
    EXPECT_EQ(controller_.state_, rm_msgs::ChassisCmd::FALLEN);
  }
  void cycleWithPreStartCommand()
  {
    restart();
    ros::NodeHandle nh("~startup");
    controller_.legged_chassis_mode_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>(nh, "mode", 1));
    controller_.unstick_pub_ = nh.advertise<std_msgs::Bool>("unstick", 1);
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "test_imu_frame";
    transform.transform.rotation.w = 1.;
    ASSERT_TRUE(transforms_.setTransform(transform, "test", true));
    // ChassisBase has selected RAW from the existing command buffer.
    controller_.cmd_struct_.stamp_ = ros::Time(19.);
    controller_.cmd_struct_.cmd_chassis_.stamp = ros::Time(19.);
    controller_.cmd_struct_.cmd_chassis_.mode = rm_msgs::ChassisCmd::RAW;
    controller_.cmd_rt_buffer_.initRT(controller_.cmd_struct_);
    controller_.state_ = rm_msgs::ChassisCmd::RAW;
    controller_.last_status_time_ = 20.001;
    controller_.moveJoint(ros::Time(20.001), ros::Duration(.001));
  }
  double rollDerivative()
  {
    double p, i, d;
    roll_.getCurrentPIDErrors(&p, &i, &d);
    return d;
  }

  uint64_t cycleSeq() const { return controller_.status_cycle_seq_; }
  void statusSnapshot(const ros::Time &time, rm_msgs::LeggedChassisStatus &chassis, rm_msgs::LeggedLQRStatus &lqr)
  {
    ros::NodeHandle nh("~status");
    // Only the publisher storage is prepared here; all field projection uses production code.
    controller_.legged_chassis_status_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisStatus>(nh, "chassis", 1));
    controller_.lqr_status_pub_.reset(
        new realtime_tools::RealtimePublisher<rm_msgs::LeggedLQRStatus>(nh, "lqr", 1));
    controller_.legged_chassis_status_pub_->msg_.linear_acc_base.resize(3);
    auto &m = controller_.lqr_status_pub_->msg_;
    m.left_leg_ref.resize(6);
    m.right_leg_ref.resize(6);
    m.left_leg_error.resize(6);
    m.right_leg_error.resize(6);
    m.left_leg_u.resize(2);
    m.right_leg_u.resize(2);
    m.F_leg.resize(2);
    m.unstick.resize(2);
    bool got_chassis = false, got_lqr = false;
    auto chassis_sub = nh.subscribe<rm_msgs::LeggedChassisStatus>("chassis", 1,
        [&](const rm_msgs::LeggedChassisStatus::ConstPtr &msg) { chassis = *msg; got_chassis = true; });
    auto lqr_sub = nh.subscribe<rm_msgs::LeggedLQRStatus>("lqr", 1,
        [&](const rm_msgs::LeggedLQRStatus::ConstPtr &msg) { lqr = *msg; got_lqr = true; });
    const auto deadline = ros::WallTime::now() + ros::WallDuration(2.);
    while ((!got_chassis || !got_lqr) && ros::WallTime::now() < deadline)
    {
      controller_.publishCompatibilityStatus(time);
      ros::spinOnce();
      ros::WallDuration(.001).sleep();
    }
    ASSERT_TRUE(got_chassis);
    ASSERT_TRUE(got_lqr);
  }
};

TEST_F(BipedalObservationTest, StatusReconstructsFullStateReferenceAndErrorWithPitchSigns)
{
  static_assert(ros::message_traits::HasHeader<rm_msgs::LeggedChassisStatus>::value, "ROS tools need a leading header");
  static_assert(ros::message_traits::HasHeader<rm_msgs::LeggedLQRStatus>::value, "ROS tools need a leading header");
  missingTfCycle(); // Count a real moveJoint call using in-memory IMU/TF handles.
  state().x << 1.2, -.3, 7.4, -.8, .11, -.12, .13, -.14, .15, -.16;
  state().length << .18, .23;
  state().roll = -.07;
  state().linear_acc.x = 1.;
  state().linear_acc.y = -2.;
  state().linear_acc.z = 9.7;
  feedback().reference << .9, -.2, 7.1, -.6, .01, -.02, .03, -.04, .05, -.06;
  feedback().reference_valid = feedback().active = true;
  rm_msgs::LeggedChassisStatus c;
  rm_msgs::LeggedLQRStatus f;
  const ros::Time time(1790754500, 123456789);
  statusSnapshot(time, c, f);
  ASSERT_EQ(c.cycle_seq, cycleSeq());
  ASSERT_EQ(c.cycle_seq, f.cycle_seq);
  EXPECT_EQ(c.header.stamp, time);
  EXPECT_EQ(f.header.stamp, time); // Preserve integer nanoseconds, not a double round-trip.
  EXPECT_TRUE(f.reference_valid);
  EXPECT_TRUE(f.normal_active);
  State10 x, ref, error;
  x << c.x, c.x_dot, c.yaw, c.d_yaw, c.left_leg_theta, c.left_leg_theta_dot,
       c.right_leg_theta, c.right_leg_theta_dot, -c.pitch, -c.d_pitch;
  ref << f.left_leg_ref[2], f.left_leg_ref[3], f.yaw_ref, f.yaw_rate_ref,
         f.left_leg_ref[0], f.left_leg_ref[1], f.right_leg_ref[0], f.right_leg_ref[1],
         -f.left_leg_ref[4], -f.left_leg_ref[5];
  error << f.left_leg_error[2], f.left_leg_error[3], c.yaw - f.yaw_ref, c.d_yaw - f.yaw_rate_ref,
           f.left_leg_error[0], f.left_leg_error[1], f.right_leg_error[0], f.right_leg_error[1],
           -f.left_leg_error[4], -f.left_leg_error[5];
  EXPECT_TRUE(x.isApprox(state().x, 1e-14));
  EXPECT_TRUE(ref.isApprox(feedback().reference, 1e-14));
  EXPECT_TRUE(error.isApprox(state().x - feedback().reference, 1e-14));
  EXPECT_DOUBLE_EQ(c.roll, state().roll);
  EXPECT_DOUBLE_EQ(c.left_leg_length, .18);
  EXPECT_DOUBLE_EQ(c.right_leg_length, .23);
  ASSERT_EQ(c.linear_acc_base.size(), 3u);
  EXPECT_DOUBLE_EQ(c.linear_acc_base[2], 9.7);
}

TEST_F(BipedalObservationTest, StatusDistinguishesFallbackReferenceFromInactiveOutput)
{
  state().x.setConstant(.2);
  state().x[YAW] = 7.4;
  state().x[YAW_RATE] = -.8;
  feedback().reference.setConstant(99.); // Old reference must not leak through the invalid flag.
  feedback().output.setConstant(42.);
  feedback().axial_force.setConstant(123.);
  rm_msgs::LeggedChassisStatus c;
  rm_msgs::LeggedLQRStatus f;
  statusSnapshot(ros::Time(10.), c, f);
  EXPECT_FALSE(f.reference_valid);
  EXPECT_FALSE(f.normal_active);
  EXPECT_DOUBLE_EQ(f.yaw_ref, c.yaw);
  EXPECT_DOUBLE_EQ(f.yaw_rate_ref, c.d_yaw);
  for (double e : f.left_leg_error) EXPECT_DOUBLE_EQ(e, 0.);
  for (double u : f.left_leg_u) EXPECT_DOUBLE_EQ(u, 0.);
  for (double force : f.F_leg) EXPECT_DOUBLE_EQ(force, 0.);
  feedback().reference_valid = true; // A retained reference alone does not imply active U/F.
  statusSnapshot(ros::Time(11.), c, f);
  EXPECT_TRUE(f.reference_valid);
  EXPECT_FALSE(f.normal_active);
  EXPECT_DOUBLE_EQ(f.yaw_ref, 99.);
  EXPECT_DOUBLE_EQ(f.yaw_rate_ref, 99.);
  for (double u : f.left_leg_u) EXPECT_DOUBLE_EQ(u, 0.);
}

TEST_F(BipedalObservationTest, StatusCycleCountsCallsAndSurvivesStopStart)
{
  EXPECT_EQ(cycleSeq(), 0u);
  missingTfCycle();
  missingTfCycle(); // Same ROS time and no published status, still a new control cycle.
  EXPECT_EQ(cycleSeq(), 2u);
  controller_.stopping(ros::Time(11.));
  restart();
  EXPECT_EQ(cycleSeq(), 2u);
  missingTfCycle(); // ROS label 10 is now older than starting's label 20.
  EXPECT_EQ(cycleSeq(), 3u);
}

TEST_F(BipedalObservationTest, JitterKeepsNormalReferencePidAndIntegration)
{
  velocities_[4] = velocities_[5] = 2.;
  observe(10.);
  execute();
  ASSERT_TRUE(feedback().active);
  feedback().reference[YAW] = .7;
  double t = 10.;
  for (double interval : {.001, .001023, .000977, .001023, .000977})
  {
    t += interval;
    observe(t); // Nominal host step stays 1 ms.
    const double previous_s = state().x[S];
    double p, old_i, d, new_i;
    leg_l_.getCurrentPIDErrors(&p, &old_i, &d);
    execute();
    leg_l_.getCurrentPIDErrors(&p, &new_i, &d);
    EXPECT_LT(new_i, old_i); // No spurious Normal reentry/PID reset.
    EXPECT_GT(state().x[S], previous_s);
    EXPECT_DOUBLE_EQ(feedback().reference[YAW], .7);
    EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
    EXPECT_TRUE(feedback().active);
    EXPECT_NE(commands_[4], 0.);
  }
}

TEST_F(BipedalObservationTest, NormalAccumulatesSlowDriftWithZeroPositionReference)
{
  velocities_[4] = velocities_[5] = 2.;
  state().x[S] = 1.;
  observe(10.);
  execute();
  ASSERT_TRUE(feedback().active);
  EXPECT_DOUBLE_EQ(state().x[S], 0.); // First Normal feedback clears the integral.
  observe(10.001023);
  execute();
  EXPECT_NEAR(state().x[S], .00016, 1e-12);
  EXPECT_DOUBLE_EQ(feedback().reference[S], 0.);
  velocityCommand(.2);
  observe(10.002);
  execute();
  EXPECT_DOUBLE_EQ(state().x[S], 0.);
  EXPECT_DOUBLE_EQ(feedback().reference[S], 0.);
  velocityCommand(0.);
  observe(10.003);
  execute();
  EXPECT_NEAR(state().x[S], .00016, 1e-12);
  EXPECT_DOUBLE_EQ(feedback().reference[S], 0.);
}

TEST_F(BipedalObservationTest, PositionIntegralUsesOriginalSpeedCommandAndRawConditions)
{
  struct Sample
  {
    int mode;
    double speed, command, position;
  };
  const Sample samples[] = {
      {rm_msgs::ChassisCmd::FOLLOW, .5, .01, 1.0005},
      {rm_msgs::ChassisCmd::FOLLOW, -.5, -.01, .9995},
      {rm_msgs::ChassisCmd::FOLLOW, .5001, 0., 0.},
      {rm_msgs::ChassisCmd::FOLLOW, -.5001, 0., 0.},
      {rm_msgs::ChassisCmd::FOLLOW, .2, .0101, 0.},
      {rm_msgs::ChassisCmd::FOLLOW, -.2, -.0101, 0.},
      {rm_msgs::ChassisCmd::RAW, .2, 0., 0.}};
  for (const auto& sample : samples)
  {
    baseMode(sample.mode);
    velocityCommand(sample.command);
    state().x[S] = 1.;
    state().x[V] = sample.speed;
    state().dt = .001;
    integrate();
    EXPECT_NEAR(state().x[S], sample.position, 1e-12);
    EXPECT_DOUBLE_EQ(state().x[V], sample.speed);
    EXPECT_DOUBLE_EQ(state().x_vel, sample.speed);
  }
}

TEST_F(BipedalObservationTest, SuppliedStepControlsIntegralNotRosLabels)
{
  velocities_[4] = velocities_[5] = 2.;
  observe(10.);
  integrate();
  double expected = .00016, t = 10.;
  EXPECT_NEAR(state().x[S], expected, 1e-12); // The first sample contributes its supplied dt.
  for (double dt : {.001, .001023, .000977})
  {
    t += .001; // A host may instead supply actual positive intervals.
    observe(t, dt);
    integrate();
    expected += .16 * dt;
    EXPECT_NEAR(state().x[S], expected, 1e-12);
  }
}

TEST_F(BipedalObservationTest, JitterDoesNotReseedVelocityFilter)
{
  observe(10.);
  ground();
  velocities_[4] = velocities_[5] = 10.;
  observe(10.001023);
  ground();
  EXPECT_GT(state().x[V], 0.);
  EXPECT_LT(state().x[V], .8); // Reseeding would return the raw .8 m/s.
}

TEST_F(BipedalObservationTest, RepeatedAndBackwardLabelsDoNotInterruptIntegration)
{
  velocities_[4] = velocities_[5] = 2.;
  state().yaw = 3.13;
  observe(10.);
  execute();
  feedback().reference[YAW] = .7;
  for (double t : {10., 9.})
  {
    const double previous_s = state().x[S];
    state().yaw = -3.13;
    observe(t);
    execute();
    EXPECT_NEAR(state().x[S], previous_s + state().x[V] * .001, 1e-12);
    EXPECT_NEAR(state().x[YAW], 2 * M_PI - 3.13, 1e-12);
    EXPECT_DOUBLE_EQ(feedback().reference[YAW], .7);
    EXPECT_TRUE(feedback().active);
    EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
  }
}

TEST_F(BipedalObservationTest, RepeatedAndBackwardLabelsDoNotReseedVelocityFilter)
{
  observe(10.);
  ground();
  velocities_[4] = velocities_[5] = 10.;
  for (double t : {10., 9.})
  {
    observe(t);
    ground();
    EXPECT_GT(state().x[V], 0.);
    EXPECT_LT(state().x[V], .8); // A reseed would immediately use the raw speed.
  }
}

TEST_F(BipedalObservationTest, ForwardRosJumpIsNotIntegratedAsElapsedTime)
{
  velocities_[4] = velocities_[5] = 2.;
  observe(10.);
  integrate();
  observe(1000.);
  integrate();
  EXPECT_NEAR(state().x[S], .00032, 1e-12);
}

TEST_F(BipedalObservationTest, MissingTransformStillIntegratesTheCurrentCycle)
{
  velocities_[4] = velocities_[5] = 2.;
  observe(10.);
  missingTfCycle(); // Enter Normal through the real mode manager before accumulating S.
  observe(10.001);
  integrate();
  const double saved_s = state().x[S];
  missingTfCycle(); // The outer cycle continues through state calculation and Normal.
  EXPECT_NEAR(state().x[S], saved_s + .00016, 1e-12);
  observe(30.);
  integrate();
  EXPECT_NEAR(state().x[S], saved_s + .00032, 1e-12);
}

TEST_F(BipedalObservationTest, MissingTransformDoesNotReseedVelocityFilter)
{
  observe(10.);
  ground();
  velocities_[4] = velocities_[5] = 10.;
  observe(10.001);
  ground();
  missingTfCycle();
  observe(30.);
  ground();
  EXPECT_GT(state().x[V], 0.);
  EXPECT_LT(state().x[V], .8);
}

TEST_F(BipedalObservationTest, NormalPostureExitsClearPositionIntegral)
{
  velocities_[4] = velocities_[5] = 2.;
  for (const auto& exit : {std::make_pair(.9, SIT_DOWN), std::make_pair(.5, PROTECT)})
  {
    state().roll = exit.first;
    state().x[S] = .1;
    observe(10.);
    execute();
    EXPECT_EQ(controller_.getBalanceMode(), exit.second);
    EXPECT_DOUBLE_EQ(state().x[S], 0.);
    EXPECT_NEAR(state().x_vel, .16, 1e-12);
  }
}

TEST_F(BipedalObservationTest, JumpIntegratesKinematicVelocityOnce)
{
  velocities_[4] = velocities_[5] = 2.;
  jump();
  observe(10.);
  execute();
  EXPECT_FALSE(feedback().active);
  EXPECT_NEAR(state().x[S], .00016, 1e-12);
}

TEST_F(BipedalObservationTest, ObservationDoesNotRejectNonfiniteWheelFeedback)
{
  velocities_[4] = std::numeric_limits<double>::quiet_NaN();
  observe(10.);
  EXPECT_TRUE(std::isnan(state().x[V]));
}

TEST_F(BipedalObservationTest, MissingImuTransformContinuesNormalAndOverwritesTemporaryZeroCommands)
{
  velocities_[4] = velocities_[5] = 2.;
  for (double &command : commands_)
    command = 12.;
  missingTfCycle();
  EXPECT_TRUE(feedback().active);
  EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
  EXPECT_NE(commands_[0], 0.);
  EXPECT_NEAR(commands_[4], -.16, 1e-12);
  EXPECT_DOUBLE_EQ(state().x[S], 0.);
  missingTfCycle();
  EXPECT_TRUE(feedback().active);
  EXPECT_NEAR(state().x[S], .00016, 1e-12);
}

TEST_F(BipedalObservationTest, NonfiniteVelocityPropagatesWithoutFilterResetOrOutputZeroing)
{
  observe(10.);
  execute();
  ASSERT_TRUE(feedback().active);
  velocities_[4] = std::numeric_limits<double>::quiet_NaN();
  observe(10.001);
  execute();
  EXPECT_TRUE(feedback().active);
  EXPECT_TRUE(std::isnan(state().x[V]));
  EXPECT_DOUBLE_EQ(state().x[S], 0.); // The low-speed integration condition is false.
  EXPECT_TRUE(std::isnan(commands_[4]));
  velocities_[4] = 0.;
  observe(10.002);
  execute();
  EXPECT_TRUE(feedback().active);
  EXPECT_TRUE(std::isnan(state().x[V])); // The KF no longer resets on a nonfinite sample.
  EXPECT_TRUE(std::isnan(commands_[4]));
}

TEST_F(BipedalObservationTest, SuppliedZeroAndNegativeStepAreNotRejected)
{
  velocities_[4] = velocities_[5] = 2.;
  observe(10.);
  integrate();
  observe(10.001, 0.);
  integrate();
  EXPECT_NEAR(state().x[S], .00016, 1e-12);
  observe(10.002, -.001);
  integrate();
  EXPECT_NEAR(state().x[S], 0., 1e-12);
}

TEST_F(BipedalObservationTest, GenuinePostureAndOverturnStillRouteToActions)
{
  state().pitch = 1.3;
  observe(10.); // Euler observation has no 1.2 rad admission threshold.
  EXPECT_DOUBLE_EQ(state().x[BODY_PITCH], 1.3);
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), SIT_DOWN); // Original Normal posture action.
  overturn();
  execute();
  EXPECT_EQ(controller_.getBalanceMode(), RECOVER);
}

TEST_F(BipedalObservationTest, EquivalentLegEvaluationExtrapolatesWithoutDomainRejection)
{
  config().domain << .1, .2, .1, .2;
  config().model.geometry_domain << .1, .2;
  config().coeffs(1, 4 * V) = 2.;
  Gain10 gain;
  Input4 uff;
  State10 equilibrium;
  const Eigen::Vector2d inside(.15, .17), outside(.21, .24);
  evaluate(config(), inside, gain, uff, &equilibrium);
  EXPECT_NEAR((equilibrium - staticEquilibrium(config().model, inside, uff)).norm(), 0., 1e-12);
  EXPECT_NO_THROW(evaluate(config(), outside, gain, uff, &equilibrium));
  EXPECT_NEAR(gain(0, V), 3.4, 1e-12); // 1 + 2*x, with normalized x = 1.2 (unclamped).
  EXPECT_TRUE(equilibrium.allFinite());
  EXPECT_NEAR((equilibrium - staticEquilibrium(config().model, outside, uff)).norm(), 0., 1e-12);
  legCommand(.45); // Both the commanded length and measured VMC length exceed this domain.
  observe(10.);
  EXPECT_GT(state().length.minCoeff(), .2);
  execute();
  EXPECT_TRUE(feedback().active);
  EXPECT_EQ(controller_.getBalanceMode(), NORMAL);
}

TEST_F(BipedalObservationTest, RuntimeEquilibriumDoesNotClampInvalidTriangle)
{
  Gain10 gain;
  Input4 uff;
  State10 equilibrium;
  // The fixture's lb + lw = .3 m is shorter than the supplied .4 m chord.
  EXPECT_NO_THROW(evaluate(config(), Eigen::Vector2d(.4, .4), gain, uff, &equilibrium));
  EXPECT_TRUE(std::isnan(equilibrium[THETA_L]));
  EXPECT_TRUE(std::isnan(equilibrium[THETA_R]));
  // Non-RT model construction still rejects a trim that cannot be represented as real angles.
  EXPECT_THROW(staticEquilibrium(config().model, Eigen::Vector2d(.4, .4), uff), std::invalid_argument);
}

TEST_F(BipedalObservationTest, GainChangesAreConsumedByTheNextCycleWithoutRestart)
{
  restart();
  const auto changed_coeffs = (2. * config().coeffs).eval();
  const State10 changed_q = State10::Constant(3.);
  const Input4 changed_r = Input4::Constant(2.);
  stageGain(changed_coeffs, changed_q, changed_r);
  missingTfCycle(); // A TF failure also continues to consume the new gain data.
  EXPECT_TRUE(config().coeffs.isApprox(changed_coeffs));
  EXPECT_TRUE(config().q.isApprox(changed_q));
  EXPECT_TRUE(config().r.isApprox(changed_r));
  const auto next_coeffs = (3. * changed_coeffs).eval();
  stageGain(next_coeffs, changed_q, changed_r);
  missingTfCycle();
  EXPECT_TRUE(config().coeffs.isApprox(next_coeffs));
}

TEST(BipedalDesignTest, CareAcceptsZeroStateWeightWithAnObservablePositiveSemidefiniteQ)
{
  LinearModel model;
  for (int i = 0; i < 10; ++i)
    model.A(i, i) = -1. - .1 * i;
  model.A(0, 9) = .25; // The unweighted state remains observable through a weighted state.
  model.B.topRows<4>().setIdentity();
  State10 q = State10::Ones();
  q[9] = 0.;
  const Input4 r = Input4::Ones();
  DesignReport report;
  Gain10 gain;
  ASSERT_NO_THROW(gain = solveCare(model, q, r, report));
  EXPECT_TRUE(gain.allFinite());
  EXPECT_GT(gain.norm(), 0.);
  EXPECT_LT(report.max_real, 0.);
  q[9] = -1.;
  EXPECT_THROW(solveCare(model, q, r, report), std::invalid_argument);
  q[9] = 0.;
  Input4 invalid_r = r;
  invalid_r[0] = 0.;
  EXPECT_THROW(solveCare(model, q, invalid_r, report), std::invalid_argument);
}

TEST(BipedalDesignTest, StableUnweightedSystemHasZeroCostAndZeroGain)
{
  LinearModel model;
  for (int i = 0; i < 10; ++i)
    model.A(i, i) = -1. - .1 * i;
  model.B.topRows<4>().setIdentity();
  DesignReport report;
  Gain10 gain;
  // A is already stable and Q=0: the exact optimum is P=0, K=0.
  ASSERT_NO_THROW(gain = solveCare(model, State10::Zero(), Input4::Ones(), report));
  EXPECT_LT(gain.norm(), 1e-12);
  EXPECT_LT(report.care_residual, 1e-12);
  EXPECT_NEAR(report.max_real, -1., 1e-12);
}

TEST(BipedalDesignTest, StableUnweightedSlowModeDoesNotNeedAnExtraPoleMargin)
{
  LinearModel model;
  model.A = -Eigen::Matrix<double, 10, 10>::Identity();
  model.A(9, 9) = -1e-9;
  model.B.topRows<4>().setIdentity();
  State10 q = State10::Ones();
  q[9] = 0.;
  DesignReport report;
  Gain10 gain;
  ASSERT_NO_THROW(gain = solveCare(model, q, Input4::Ones(), report));
  Gain10 expected = Gain10::Zero();
  expected.leftCols<4>() = (std::sqrt(2.) - 1.) * Eigen::Matrix4d::Identity();
  EXPECT_LT((gain - expected).norm(), 1e-12);
  EXPECT_NEAR(report.max_real, -1e-9, 1e-15);
}

TEST(BipedalDesignTest, UncontrollableUnstableModeStillHasNoStabilizingSolution)
{
  LinearModel model;
  model.A = -Eigen::Matrix<double, 10, 10>::Identity();
  model.A(9, 9) = 1.;
  model.B.topRows<4>().setIdentity();
  DesignReport report;
  EXPECT_THROW(solveCare(model, State10::Ones(), Input4::Ones(), report), std::runtime_error);
}

TEST_F(BipedalObservationTest, NormalDoesNotRejectNonfiniteFeedbackOutput)
{
  config().coeffs(0, 4 * V) = std::numeric_limits<double>::quiet_NaN();
  observe(10.);
  execute();
  EXPECT_TRUE(feedback().active);
  EXPECT_TRUE(std::isnan(feedback().output[0]));
  EXPECT_TRUE(std::isnan(commands_[4]));
}

TEST_F(BipedalObservationTest, RollReentryHasProportionalForceWithoutFictitiousDerivative)
{
  state().roll = .06838551517;
  for (int entry = 0; entry < 3; ++entry)
  {
    reenter();
    observe(10. + .001 * entry);
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
  observe(10.);
  execute();
  EXPECT_NEAR(roll_.getCurrentCmd(), -102.578272755 - 7., 1e-9);
  EXPECT_NEAR(rollDerivative(), -.2, 1e-12);
  reenter();
  jump();
  observe(10.001);
  execute();
  EXPECT_NEAR(roll_.getCurrentCmd(), -102.578272755 - 7., 1e-9);
  EXPECT_NEAR(rollDerivative(), -.2, 1e-12);
}

TEST_F(BipedalObservationTest, ProtectToNormalResetsLegIntegralWithoutRollKick)
{
  state().roll = .06838551517;
  observe(10.);
  execute();
  control_toolbox::Pid wheel_l, wheel_r, theta_diff, yaw;
  std::vector<hardware_interface::JointHandle *> joints;
  for (auto &handle : handles_)
    joints.push_back(&handle);
  Protect protect(&controller_, joints, {&leg_l_, &leg_r_}, {&theta_l_, &theta_r_},
                  {&wheel_l, &wheel_r}, &theta_diff, &yaw);
  controller_.setMode(PROTECT);
  reenter();
  observe(10.001);
  protect.execute(ros::Time(10.001), ros::Duration(.001));
  ASSERT_EQ(controller_.getBalanceMode(), NORMAL);
  ASSERT_FALSE(controller_.getStateChange());
  // Deliberately leave a prior mechanical-task integral; Normal must reset it.
  leg_l_.computeCommand(-.1, 0., ros::Duration(.2));
  observe(10.002);
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
  EXPECT_FALSE(feedback().active);
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
}

TEST_F(BipedalObservationTest, StartupDoesNotRequireNewCommandTimestamps)
{
  velocities_[4] = velocities_[5] = 2.;
  cycleWithPreStartCommand();
  EXPECT_DOUBLE_EQ(state().x[S], 0.);
  EXPECT_EQ(controller_.getBaseState(), rm_msgs::ChassisCmd::RAW);
  EXPECT_EQ(controller_.getBalanceMode(), STAND_UP);
}

TEST_F(BipedalObservationTest, StopStartPreservesObservationPositionAndHeading)
{
  state().roll = .06838551517;
  observe(10.);
  execute();
  feedback().reference[YAW] = .7;
  state().x[S] = 1.;
  state().x[YAW] = 2 * M_PI + .3;
  state().yaw = .3;
  controller_.stopping(ros::Time(10.001));
  restart();
  EXPECT_FALSE(feedback().reference_valid);
  EXPECT_DOUBLE_EQ(state().x[S], 1.);
  EXPECT_DOUBLE_EQ(state().x[YAW], 2 * M_PI + .3);
  EXPECT_EQ(controller_.getBalanceMode(), SIT_DOWN);
  EXPECT_FALSE(controller_.getCompleteStand());
  for (double command : commands_)
    EXPECT_DOUBLE_EQ(command, 0.);
  // Exercise the later Normal entry separately; this is not a stand-up simulation.
  controller_.setMode(NORMAL);
  observe(20.001);
  execute();
  EXPECT_TRUE(feedback().reference_valid);
  EXPECT_DOUBLE_EQ(state().x[S], 0.);
  EXPECT_DOUBLE_EQ(feedback().reference[S], 0.);
  EXPECT_DOUBLE_EQ(feedback().reference[YAW], state().x[YAW]);
  EXPECT_NEAR(state().x[YAW], 2 * M_PI + .3, 1e-12);
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
  observe(10.);
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
