#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller.h"
#include <exception>
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
    resetMpc();
    controller->setStateChange(true);
    ROS_INFO("[balance] Enter NORMAL with ten-state feedback");
  }
  const auto &action = controller->getActionParams();
  if (controller->getOverturn())
  {
    requestMode(RECOVER);
    return;
  }
  if (controller->getDown5cmStairFlag() &&
      (std::abs(chassis.x[BODY_PITCH]) > action.down5cmStairPitchThreshold ||
       std::abs(chassis.x[THETA_L]) > action.down5cmStairThetaThreshold ||
       std::abs(chassis.x[THETA_R]) > action.down5cmStairThetaThreshold))
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
    resetMpc();
    ROS_INFO("[balance] Jump start: original axial stages with mechanical swing PID");
  }
  if (jump_phase_ != IDLE)
  {
    executeJump(time, period);
    return;
  }
  const auto command = controller->getVelCmd();
  // Original automatic stair trigger, evaluated before ground-LQR feedback.
  if (controller->getCompleteStand() &&
      std::abs((chassis.x[THETA_L] + chassis.x[THETA_R]) / 2.) > .50 && std::abs(command.x) > .1 &&
      std::abs(chassis.x[V]) > .1 && (lp.L0 + rp.L0) / 2. > .30 && controller->getLegCmd() > .30)
  {
    controller->pubLegLenStatus(false);
    requestMode(UPSTAIRS);
    return;
  }
  // Match A's action-stage condition exactly: abs(theta_L + theta_R) / 2,
  // rather than the mean of the two angle magnitudes.
  if (!controller->getCompleteStand() && std::abs(chassis.x[BODY_PITCH]) < .2 &&
      std::abs(chassis.x[THETA_L] + chassis.x[THETA_R]) / 2. < .2)
  {
    controller->setCompleteStand(true);
    ROS_INFO("[balance] Original Normal standing condition reached");
  }
  const bool complete_stand = controller->getCompleteStand();
  const double requested_length = complete_stand ? controller->getLegCmd() : controller->getDefaultLegLength();
  auto *bipedal_controller = static_cast<BipedalController *>(controller);
  // Keep the original action-stage posture routing, independent of model trim checks.
  if (std::abs(chassis.x[THETA_L]) > 1.0)
  {
    requestMode(SIT_DOWN);
    return;
  }
  if (std::abs(chassis.x[THETA_R]) > 1.0)
  {
    requestMode(SIT_DOWN);
    return;
  }
  if (std::abs(chassis.x[BODY_PITCH]) > .6)
  {
    requestMode(SIT_DOWN);
    return;
  }
  if (std::abs(chassis.roll) > .8)
  {
    requestMode(SIT_DOWN);
    return;
  }
  if (requested_length < .22)
  {
    if (std::abs(chassis.x[THETA_L]) > 1.0)
    {
      requestMode(PROTECT);
      return;
    }
    if (std::abs(chassis.x[THETA_R]) > 1.0)
    {
      requestMode(PROTECT);
      return;
    }
    if (std::abs(chassis.x[BODY_PITCH]) > .4)
    {
      requestMode(PROTECT);
      return;
    }
    if (std::abs(chassis.roll) > .4)
    {
      requestMode(PROTECT);
      return;
    }
  }
  bipedal_controller->updateGroundVelocity();
  const double circle = chassis.x[V] * chassis.x[YAW_RATE];
  const double alpha = std::abs(circle) > 10. ? 10. / std::abs(circle) : 1.;
  const double velocity_reference = complete_stand ? alpha * command.x : 0.;
  if (!computeFeedback(velocity_reference, alpha * command.z))
  {
    requestMode(PROTECT);
    return;
  }
  // Second-order length shaping is inactive; retain the former path for reference.
  // if (complete_stand)
  //   length_reference_.update(requested_length, chassis.dt, config.length_reference_tau);
  // else
  //   length_reference_.reset(requested_length);
  // Pass the effective target to the existing PID on this cycle.
  // This resets only the unused length generator.
  length_reference_.reset(requested_length);
  const double current_length = (lp.L0 + rp.L0) / 2.;
  const double roll_force = roll_->computeCommand(-chassis.roll, -chassis.roll_rate, period);
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
    clamp(pid, -150., 150.);
    const double turning = model->M * circle * pos.L0 / track;
    const double turning_force = side == LEFT ? -turning : turning;
    const double gravity_force = model->f_gravity / std::cos(pos.theta);
    const double roll_component = side == LEFT ? roll_force : -roll_force;
    const double spring_force = -controller->f_spring_force(pos.L0);
    force[side] = pid + turning_force + gravity_force + roll_component + spring_force;
  }
  auto map = [&](const Input4 &input) {
    double lt[2], rt[2];
    left.vmc->leg_conv(force[0], input[2], lt);
    right.vmc->leg_conv(force[1], input[3], rt);
    Joint6 out;
    out << lt[0], lt[1], rt[0], rt[1], input[0], input[1];
    return out;
  };
  const auto commands = map(feedback.output);
  for (int i = 0; i < 6; ++i)
    joints_[i]->setCommand(commands[i]);
}

void Normal::requestMode(int mode)
{
  // All Normal action exits occur before the ground KF/feedback branch.
  auto* bipedal_controller = static_cast<BipedalController*>(controller);
  bipedal_controller->integrateVelocity();
  if (mode == PROTECT || mode == SIT_DOWN)
    bipedal_controller->chassis_state_.x[S] = 0.;
  auto &result = controller->getNormalFeedback();
  result.active = result.reference_valid = false;
  resetMpc();
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
  static_cast<BipedalController *>(controller)->integrateVelocity();
  const auto &action = controller->getActionParams();
  const auto &body = controller->getChassisState();
  auto &left = controller->getLegState(LEFT);
  auto &right = controller->getLegState(RIGHT);
  const double length = (left.vmc->getPos().L0 + right.vmc->getPos().L0) / 2.;
  const double gravity = controller->getModelParams()->f_gravity;
  const double roll_force = roll_->computeCommand(-body.roll, -body.roll_rate, period);
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
    commands[side].torque = thetas_[side]->computeCommand(-pos.theta, period);
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

void Normal::resetMpc()
{
  auto* bipedal_controller = static_cast<BipedalController*>(controller);
  if (bipedal_controller->mpc_)
    bipedal_controller->mpc_->reset();
}

bool Normal::computeFeedback(double velocity, double yaw_rate)
{
  auto& feedback = controller->getNormalFeedback();
  const auto& chassis = controller->getChassisState();
  const auto& config = controller->getLqrConfig();
  auto* bipedal_controller = static_cast<BipedalController*>(controller);
  auto* mpc = bipedal_controller->mpc_.get();
  feedback.active = false;
  if (!mpc || !mpc->initialized() || !chassis.x.allFinite() || !chassis.length.allFinite() ||
      (chassis.length.array() < config.model.geometry_domain[0]).any() ||
      (chassis.length.array() > config.model.geometry_domain[1]).any() || !std::isfinite(chassis.dt) ||
      chassis.dt <= 0. || !std::isfinite(velocity) || !std::isfinite(yaw_rate))
    return false;
  State10 equilibrium;
  try
  {
    equilibrium = staticEquilibrium(config.model, chassis.length, config.uff);
  }
  catch (const std::exception&)
  {
    return false;
  }
  if (!feedback.reference_valid)
  {
    feedback.reference.setZero();
    bipedal_controller->chassis_state_.x[S] = 0.;
    feedback.reference[YAW] = chassis.x[YAW];
    feedback.reference_valid = true;
  }
  // Absolute physical angles remain in the observation. Only the commanded static
  // equilibrium is scheduled; S is cleared and yaw reference initialized on Normal entry.
  for (int i : { THETA_L, THETA_R, BODY_PITCH })
    feedback.reference[i] = equilibrium[i];
  feedback.reference[V] = velocity;
  feedback.reference[YAW_RATE] = yaw_rate;
  feedback.reference[YAW] += yaw_rate * chassis.dt;
  if (!feedback.reference.allFinite())
    return false;
  Input4 output;
  const auto& params = bipedal_controller->mpc_params_;
  auto& trajectory = bipedal_controller->mpc_reference_;
  for (int i = 0; i < params.horizon; ++i)
  {
    trajectory.col(i) = feedback.reference;
    trajectory(S, i) += velocity * params.sample_period * i;
    trajectory(YAW, i) += yaw_rate * params.sample_period * i;
  }
  if (!mpc->solve(chassis.x, trajectory, output))
    return false;
  feedback.output = output;
  feedback.active = true;
  return true;
}

} // namespace rm_chassis_controllers
