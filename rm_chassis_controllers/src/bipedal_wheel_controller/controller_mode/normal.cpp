#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller.h"
namespace rm_chassis_controllers
{
Normal::Normal(BipedalControllerInterface *c, const std::vector<hardware_interface::JointHandle *> &joints,
               const std::vector<control_toolbox::Pid *> &legs, control_toolbox::Pid *yaw,
               control_toolbox::Pid *theta_diff, control_toolbox::Pid *roll, control_toolbox::Pid *wheel_diff,
               const std::vector<control_toolbox::Pid *> &thetas)
    : ModeBase(c), thetas_(thetas), yaw_(yaw), theta_diff_(theta_diff), wheel_diff_(wheel_diff), joints_(joints),
      legs_(legs), roll_(roll)
{
}
void Normal::execute(const ros::Time &time, const ros::Duration &period)
{
  auto &feedback = controller->getNormalFeedback();
  const auto &chassis = controller->getChassisState();
  const auto &config = controller->getLqrConfig();
  auto &left = controller->getLegState(LEFT);
  auto &right = controller->getLegState(RIGHT);
  const auto &lp = left.vmc->getPos();
  const auto &rp = right.vmc->getPos();
  if (!controller->getStateChange())
  {
    jump_phase_ = IDLE;
    jump_dwell_ = 0.;
    length_reference_.reset((lp.L0 + rp.L0) / 2.);
    roll_->reset();
    for (auto *pid : legs_)
      pid->reset();
    feedback.active = false;
    feedback.reference_valid = false;
    feedback.reason = lqr10::Reason::NotNormal;
    controller->setStateChange(true);
    ROS_INFO("[balance] Enter NORMAL with ten-state feedback");
  }
  if (static_cast<BipedalController *>(controller)->turn_debug_capture_)
    static_cast<BipedalController *>(controller)->turn_debug_[182] = jump_phase_;
  if (feedback.faulted)
    return;
  const auto &action = controller->getActionParams();
  if (!chassis.valid && chassis.raw_valid)
  {
    requestMode(controller->getOverturn() ? RECOVER : PROTECT);
    return;
  }
  if (controller->getOverturn())
  {
    requestMode(RECOVER);
    return;
  }
  if (controller->getDown5cmStairFlag() &&
      (std::abs(chassis.x[lqr10::BODY_PITCH]) > action.down5cmStairPitchThreshold ||
       std::abs(chassis.x[lqr10::THETA_L]) > action.down5cmStairThetaThreshold ||
       std::abs(chassis.x[lqr10::THETA_R]) > action.down5cmStairThetaThreshold))
  {
    controller->setDown5cmStairFlag(false);
    requestMode(SIT_DOWN);
    return;
  }
  if (jump_phase_ == IDLE && controller->getJumpCmd() && time.toSec() - last_jump_time_ > action.jumpOverTime_)
  {
    jump_phase_ = LEG_RETRACTION;
    jump_dwell_ = 0.;
    feedback.active = false;
    feedback.reference_valid = false;
    ROS_INFO("[balance] Jump start: original axial stages with mechanical swing PID");
  }
  if (jump_phase_ != IDLE)
  {
    if (static_cast<BipedalController *>(controller)->turn_debug_capture_)
      static_cast<BipedalController *>(controller)->turn_debug_[182] = jump_phase_;
    executeJump(time, period);
    return;
  }
  const auto command = controller->getVelCmd();
  // Original automatic stair trigger, evaluated before ground-LQR feedback.
  if (controller->getCompleteStand() &&
      std::abs((chassis.x[lqr10::THETA_L] + chassis.x[lqr10::THETA_R]) / 2.) > .50 && std::abs(command.x) > .1 &&
      std::abs(chassis.x[lqr10::V]) > .1 && (lp.L0 + rp.L0) / 2. > .30 && controller->getLegCmd() > .30)
  {
    controller->pubLegLenStatus(false);
    requestMode(UPSTAIRS);
    return;
  }
  // Match A's action-stage condition exactly: abs(theta_L + theta_R) / 2,
  // rather than the mean of the two angle magnitudes. Independent posture
  // protection remains in computeFeedback().
  if (!controller->getCompleteStand() && chassis.valid && std::abs(chassis.x[lqr10::BODY_PITCH]) < .2 &&
      std::abs(chassis.x[lqr10::THETA_L] + chassis.x[lqr10::THETA_R]) / 2. < .2)
  {
    controller->setCompleteStand(true);
    ROS_INFO("[balance] Original Normal standing condition reached");
  }
  const bool complete_stand = controller->getCompleteStand();
  const double requested_length = complete_stand ? controller->getLegCmd() : controller->getDefaultLegLength();
  auto *recorder = static_cast<BipedalController *>(controller);
  if (recorder->turn_debug_capture_)
    recorder->turn_debug_[18] = requested_length;
  if (!std::isfinite(requested_length) || requested_length < config.domain.col(0).maxCoeff() ||
      requested_length > config.domain.col(1).minCoeff())
  {
    ROS_ERROR("[balance] Normal leg command is outside the loaded gain domain");
    controller->latchControlFault(lqr10::Reason::OutsideDomain);
    return;
  }
  // The previous migration gate (.2 rad/.15 rad/.6 rad/s) was a near-static
  // candidate-admission check. It conflicted with the original StandUp->Normal
  // handoff. Keep A's action-stage posture meaning and the existing Normal
  // input/axial-force bounds here.
  auto posture_exit = [&](int mode) {
    requestMode(mode);
    feedback.reason = lqr10::Reason::Posture;
  };
  if (std::abs(chassis.x[lqr10::THETA_L]) > 1.0)
  {
    posture_exit(SIT_DOWN);
    return;
  }
  if (std::abs(chassis.x[lqr10::THETA_R]) > 1.0)
  {
    posture_exit(SIT_DOWN);
    return;
  }
  if (std::abs(chassis.x[lqr10::BODY_PITCH]) > .6)
  {
    posture_exit(SIT_DOWN);
    return;
  }
  if (std::abs(chassis.roll) > .8)
  {
    posture_exit(SIT_DOWN);
    return;
  }
  if (requested_length < .22)
  {
    if (std::abs(chassis.x[lqr10::THETA_L]) > 1.0)
    {
      posture_exit(PROTECT);
      return;
    }
    if (std::abs(chassis.x[lqr10::THETA_R]) > 1.0)
    {
      posture_exit(PROTECT);
      return;
    }
    if (std::abs(chassis.x[lqr10::BODY_PITCH]) > .4)
    {
      posture_exit(PROTECT);
      return;
    }
    if (std::abs(chassis.roll) > .4)
    {
      posture_exit(PROTECT);
      return;
    }
  }
  if (!recorder->updateGroundVelocity())
  {
    reject(lqr10::Reason::InvalidSnapshot);
    return;
  }
  const double circle = chassis.x[lqr10::V] * chassis.x[lqr10::YAW_RATE];
  const double alpha = std::abs(circle) > 10. ? 10. / std::abs(circle) : 1.;
  const double velocity_reference = complete_stand ? alpha * command.x : 0.;
  if (recorder->turn_debug_capture_)
  {
    recorder->turn_debug_[20] = circle;
    recorder->turn_debug_[21] = alpha;
  }
  if (!computeFeedback(velocity_reference, alpha * command.z))
    return;
  if (feedback.faulted || !controller->getStateChange())
    return;
  // Second-order length shaping is inactive; retain the former path for reference.
  // if (complete_stand)
  //   length_reference_.update(requested_length, chassis.dt, config.length_reference_tau);
  // else
  //   length_reference_.reset(requested_length);
  // Pass the checked effective target to the existing PID on this cycle.
  // This resets only the unused length generator, not PID/KF/longitudinal history.
  length_reference_.reset(requested_length);
  if (recorder->turn_debug_capture_)
    recorder->turn_debug_[19] = length_reference_.position;
  const double current_length = (lp.L0 + rp.L0) / 2.;
  const double roll_force = roll_->computeCommand(-chassis.roll, period);
  const auto &model = controller->getModelParams();
  const double track = controller->getChassisGeometryParams()->wheel_track;
  auto &force = feedback.axial_force;
  for (int side = 0; side < 2; ++side)
  {
    const auto &pos = side == LEFT ? lp : rp;
    // Differentiate the measured mean leg length, not a discrete setpoint
    // change. This keeps the existing P/I/D gains while avoiding a derivative
    // kick when complete_stand opens the .20 m command.
    const double length_error = length_reference_.position - current_length;
    const double length_error_dot = -chassis.dlength.mean();
    double pid = legs_[side]->computeCommand(length_error, length_error_dot, period);
    if (recorder->turn_debug_capture_)
      recorder->turn_debug_[59 + side] = pid;
    clamp(pid, -150., 150.);
    const double turning = model->M * circle * pos.L0 / track;
    const double turning_force = side == LEFT ? -turning : turning;
    const double gravity_force = model->f_gravity / std::cos(pos.theta);
    const double roll_component = side == LEFT ? roll_force : -roll_force;
    const double spring_force = -controller->f_spring_force(pos.L0);
    force[side] = pid + turning_force + gravity_force + roll_component + spring_force;
    if (recorder->turn_debug_capture_)
    {
      recorder->turn_debug_[61 + side] = pid;
      recorder->turn_debug_[63 + side] = turning_force;
      recorder->turn_debug_[65 + side] = gravity_force;
      recorder->turn_debug_[67 + side] = roll_component;
      recorder->turn_debug_[69 + side] = spring_force;
      recorder->turn_debug_[71 + side] = force[side];
    }
  }
  if (!force.allFinite())
  {
    controller->latchControlFault(lqr10::Reason::NonfiniteOutput);
    return;
  }
  force = force.cwiseMax(Eigen::Vector2d::Constant(-config.max_axial_force))
              .cwiseMin(Eigen::Vector2d::Constant(config.max_axial_force));
  if (recorder->turn_debug_capture_)
    for (int i = 0; i < 2; ++i)
      recorder->turn_debug_[73 + i] = force[i];
  auto map = [&](const lqr10::Input4 &input) {
    double lt[2], rt[2];
    left.vmc->leg_conv(force[0], input[2], lt);
    right.vmc->leg_conv(force[1], input[3], rt);
    lqr10::Joint6 out;
    out << lt[0], lt[1], rt[0], rt[1], input[0], input[1];
    return out;
  };
  const auto commands = map(feedback.output);
  for (int i = 0; i < 6; ++i)
    joints_[i]->setCommand(commands[i]);
  if (recorder->turn_debug_capture_)
  {
    recorder->turn_debug_[13] = 1.;
    for (int i = 0; i < 6; ++i)
      recorder->turn_debug_[75 + i] = commands[i];
  }
}

void Normal::requestMode(int mode)
{
  auto &result = controller->getNormalFeedback();
  result.active = result.reference_valid = false;
  controller->setMode(mode);
  controller->setStateChange(false);
  controller->setCompleteStand(false);
  controller->setJumpCmd(false);
  jump_phase_ = IDLE;
  if (!joints_.empty())
    setJointCommands(joints_, {0, 0, {0., 0.}}, {0, 0, {0., 0.}});
}
void Normal::executeJump(const ros::Time &time, const ros::Duration &period)
{
  const auto &action = controller->getActionParams();
  const auto &body = controller->getChassisState();
  auto &left = controller->getLegState(LEFT);
  auto &right = controller->getLegState(RIGHT);
  const double length = (left.vmc->getPos().L0 + right.vmc->getPos().L0) / 2.;
  const double gravity = controller->getModelParams()->f_gravity;
  const double roll_force = roll_->computeCommand(-body.roll, period);
  LegCommand commands[2]{};
  for (int side = 0; side < 2; ++side)
  {
    auto &leg = side == 0 ? left : right;
    const auto &pos = leg.vmc->getPos();
    const double x = (pos.L0 - .12) / (.35 - .11);
    const double shape = 1. - 3. * x * x + 2. * x * x * x;
    double force = 0.;
    if (jump_phase_ == LEG_RETRACTION)
      force = legs_[side]->computeCommand(jumpLengthDes[LEG_RETRACTION].second - .02 - length, period) +
              gravity / std::cos(pos.theta) + (side == 0 ? roll_force : -roll_force);
    else if (jump_phase_ == JUMP_UP)
      force = action.jump_up_force * shape + gravity;
    else
    {
      const double flipped = 1. - x;
      force = -action.off_ground_force * (1. - 3. * flipped * flipped + 2. * flipped * flipped * flipped);
    }
    commands[side].force = force - controller->f_spring_force(pos.L0);
    // The ground K10 is not used during flight or outside its validated length domain.
    // This is the original action's mechanical PID port, replacing only the old K6 flight row.
    commands[side].torque = thetas_[side]->computeCommand(-pos.theta, period);
    clamp(commands[side].torque, -controller->getLqrConfig().input_limits[2 + side],
          controller->getLqrConfig().input_limits[2 + side]);
    leg.vmc->leg_conv(commands[side].force, commands[side].torque, commands[side].input);
  }
  setJointCommands(joints_, commands[0], commands[1]);
  const bool arrived =
      jump_phase_ == LEG_RETRACTION ? length < .13 : jump_phase_ == JUMP_UP ? length > .33 : length < .14;
  if (arrived)
    jump_dwell_ += period.toSec();
  const double dwell = jump_phase_ == LEG_RETRACTION ? .010 : jump_phase_ == JUMP_UP ? .004 : .100;
  if (jump_dwell_ >= dwell)
  {
    jump_dwell_ = 0.;
    if (jump_phase_ == LEG_RETRACTION)
      jump_phase_ = JUMP_UP;
    else if (jump_phase_ == JUMP_UP)
      jump_phase_ = OFF_GROUND;
    else
    {
      jump_phase_ = IDLE;
      last_jump_time_ = time.toSec();
      controller->setJumpCmd(false);
      controller->setStateChange(false);
      controller->setCompleteStand(false);
      auto &result = controller->getNormalFeedback();
      result.reference_valid = false;
      ROS_INFO("[balance] Jump end; re-enter Normal ten-state feedback");
    }
  }
}

using namespace lqr10;
bool Normal::reject(Reason reason)
{
  auto &feedback = controller->getNormalFeedback();
  feedback.active = false;
  if (reason == Reason::Posture)
  {
    requestMode(controller->getOverturn() ? RECOVER : PROTECT);
    feedback.reason = reason;
  }
  else
    controller->latchControlFault(reason);
  return false;
}

bool Normal::computeFeedback(double velocity, double yaw_rate)
{
  auto &feedback = controller->getNormalFeedback();
  const auto &chassis = controller->getChassisState();
  const auto &config = controller->getLqrConfig();
  if (feedback.faulted)
    return false;
  if (!chassis.valid || !chassis.x.allFinite() || !chassis.length.allFinite() || !chassis.dlength.allFinite() ||
      !std::isfinite(chassis.roll) || !std::isfinite(velocity) || !std::isfinite(yaw_rate))
    return reject(Reason::InvalidSnapshot);
  Gain10 gain;
  Input4 feedforward;
  State10 equilibrium;
  if (!lqr10::evaluate(config, chassis.length, gain, feedforward, &equilibrium))
    return reject(Reason::OutsideDomain);
  if (!std::isfinite(chassis.command_age) || chassis.command_age < 0. ||
      chassis.command_age > config.max_command_age)
    return reject(Reason::CommandStale);
  auto *recorder = static_cast<BipedalController *>(controller);
  if (recorder->turn_debug_capture_)
    recorder->turn_debug_[155] = !feedback.reference_valid;
  if (!feedback.reference_valid)
  {
    feedback.reference.setZero();
    feedback.reference[S] = chassis.x[S];
    feedback.reference[YAW] = chassis.x[YAW];
    feedback.reference_valid = true;
    longitudinal_reference_.reset(controller->getCompleteStand());
  }
  // Absolute physical angles remain in the observation. Only the commanded static
  // equilibrium is scheduled; s/yaw history is initialized once per Normal entry.
  for (int i : {THETA_L, THETA_R, BODY_PITCH})
    feedback.reference[i] = equilibrium[i];
  feedback.reference[V] = velocity;
  feedback.reference[YAW_RATE] = yaw_rate;
  longitudinal_reference_.update(chassis.x[S], chassis.x[V], velocity, controller->getCompleteStand(),
                                 recorder->translation_source_active_, recorder->translation_ramp_zero_, chassis.dt,
                                 config.position_release_tau, config.hold_capture_speed, feedback.reference[S]);
  feedback.reference[YAW] += yaw_rate * chassis.dt;
  feedback.output = feedforward - gain * (chassis.x - feedback.reference);
  if (recorder->turn_debug_capture_)
  {
    recorder->turn_debug_[12] = 1.;
    for (int j = 0; j < 10; ++j)
      recorder->turn_debug_[33 + j] = feedback.reference[j];
    for (int i = 0; i < 4; ++i)
    {
      recorder->turn_debug_[47 + i] = feedback.output[i];
      recorder->turn_debug_[55 + i] = feedforward[i];
      // Direct gain samples at 100 Hz for signed same-cycle decomposition.
      if (recorder->turn_debug_cycle_ % 10 == 0)
        for (int j = 0; j < 10; ++j)
          recorder->turn_debug_[115 + 10 * i + j] = gain(i, j);
    }
  }
  if (!feedback.output.allFinite())
    return reject(Reason::NonfiniteOutput);
  feedback.output = feedback.output.cwiseMax(-config.input_limits).cwiseMin(config.input_limits);
  if (recorder->turn_debug_capture_)
    for (int i = 0; i < 4; ++i)
      recorder->turn_debug_[51 + i] = feedback.output[i];
  feedback.active = true;
  feedback.reason = Reason::None;
  return true;
}

} // namespace rm_chassis_controllers
