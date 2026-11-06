#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/core/ground_core.h"
namespace rm_chassis_controllers
{
Normal::Normal(BipedalControllerInterface* c, const std::vector<hardware_interface::JointHandle*>& joints,
               const std::vector<control_toolbox::Pid*>& legs, control_toolbox::Pid* yaw,
               control_toolbox::Pid* theta_diff, control_toolbox::Pid* roll, control_toolbox::Pid* wheel_diff,
               const std::vector<control_toolbox::Pid*>& thetas)
  : ModeBase(c)
  , thetas_(thetas)
  , yaw_(yaw)
  , theta_diff_(theta_diff)
  , wheel_diff_(wheel_diff)
  , joints_(joints)
  , legs_(legs)
  , roll_(roll)
{
}
void Normal::execute(const ros::Time& time, const ros::Duration& period)
{
  auto& feedback = controller->getNormalFeedback();
  const auto& chassis = controller->getChassisState();
  auto& left = controller->getLegState(LEFT);
  auto& right = controller->getLegState(RIGHT);
  const auto& lp = left.vmc->getPos();
  const auto& rp = right.vmc->getPos();
  if (!controller->getStateChange())
  {
    jump_phase_ = IDLE;
    jump_dwell_ = 0.;
    length_reference_.reset((lp.L0 + rp.L0) / 2.);
    roll_->reset();
    for (auto* pid : legs_)
      pid->reset();
    feedback.active = false;
    feedback.reference_valid = false;
    controller->setStateChange(true);
    ROS_INFO("[balance] Enter NORMAL with base-task ten-state feedback");
  }
  const auto& action = controller->getActionParams();
  if (controller->getOverturn())
  {
    requestMode(RECOVER);
    return;
  }
  if (controller->getDown5cmStairFlag() && (std::abs(chassis.x[BODY_PITCH]) > action.down5cmStairPitchThreshold ||
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
    ROS_INFO("[balance] Jump start: original axial stages with mechanical swing PID");
  }
  if (jump_phase_ != IDLE)
  {
    executeJump(time, period);
    return;
  }
  const auto command = controller->getVelCmd();
  // Original automatic stair trigger, evaluated before ground-LQR feedback.
  if (controller->getCompleteStand() && std::abs((chassis.x[THETA_L] + chassis.x[THETA_R]) / 2.) > .50 &&
      std::abs(command.x) > .1 && std::abs(chassis.x[V]) > .1 && (lp.L0 + rp.L0) / 2. > .30 &&
      controller->getLegCmd() > .30)
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
  auto* bipedal_controller = static_cast<BipedalController*>(controller);
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
  GroundCommand ground_command;
  ground_command.forward_velocity = command.x;
  ground_command.yaw_rate = command.z;
  ground_command.leg_length = requested_length;
  ground_command.complete_stand = complete_stand;
  const auto commands =
      ground_core::computeGroundTorques(controller->getLqrConfig(), *controller->getModelParams(),
                                        controller->getChassisGeometryParams()->wheel_track,
                                        bipedal_controller->chassis_state_, bipedal_controller->leg_state_, feedback,
                                        length_reference_, { { legs_[LEFT], legs_[RIGHT] } }, *roll_, ground_command);
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
  auto& result = controller->getNormalFeedback();
  result.active = result.reference_valid = false;
  controller->setMode(mode);
  controller->setStateChange(false);
  controller->setCompleteStand(false);
  controller->setJumpCmd(false);
  jump_phase_ = IDLE;
  if (!joints_.empty())
    setJointCommands(joints_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
}
void Normal::executeJump(const ros::Time& time, const ros::Duration& period)
{
  static_cast<BipedalController*>(controller)->integrateVelocity();
  const auto& action = controller->getActionParams();
  const auto& body = controller->getChassisState();
  auto& left = controller->getLegState(LEFT);
  auto& right = controller->getLegState(RIGHT);
  const double length = (left.vmc->getPos().L0 + right.vmc->getPos().L0) / 2.;
  const double gravity = controller->getModelParams()->f_gravity;
  const double roll_force = roll_->computeCommand(-body.roll, -body.roll_rate, period);
  LegCommand commands[2]{};
  for (int side = 0; side < 2; ++side)
  {
    auto& leg = side == 0 ? left : right;
    const auto& pos = leg.vmc->getPos();
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
    // The ground K10 is not used during the jump action.
    // This is the original action's mechanical PID port, replacing only the old K6 flight row.
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
      auto& result = controller->getNormalFeedback();
      result.reference_valid = false;
      ROS_INFO("[balance] Jump end; re-enter Normal ten-state feedback");
    }
  }
}

}  // namespace rm_chassis_controllers
