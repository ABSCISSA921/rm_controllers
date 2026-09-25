//
// Created by guanlin on 25-9-4.
//

#include "bipedal_wheel_controller/controller_mode/mode_manager.h"
#include <rm_msgs/ChassisCmd.h>

namespace rm_chassis_controllers
{
ModeManager::ModeManager(BipedalControllerInterface *controller, ros::NodeHandle &controller_nh,
                         const std::vector<hardware_interface::JointHandle *> &joint_handles)
    : controller_(controller)
{
  const std::pair<const char *, control_toolbox::Pid *> pids[] = {
      {"pid_theta_diff", &pid_theta_diff_},
      {"pid_yaw_vel", &pid_yaw_vel_},
      {"pid_wheel_vel_diff", &pid_wheel_vel_diff_},
      {"pid_left_leg", &pid_left_leg_},
      {"pid_right_leg", &pid_right_leg_},
      {"pid_roll", &pid_roll_},
      {"pid_left_leg_theta", &pid_left_leg_theta_},
      {"pid_right_leg_theta", &pid_right_leg_theta_},
      {"pid_left_leg_theta_vel", &pid_left_leg_theta_vel_},
      {"pid_right_leg_theta_vel", &pid_right_leg_theta_vel_},
      {"pid_left_wheel_vel", &pid_left_wheel_vel_},
      {"pid_right_wheel_vel", &pid_right_wheel_vel_},
      {"pid_left_leg_stand_up", &pid_left_leg_stand_up_},
      {"pid_right_leg_stand_up", &pid_right_leg_stand_up_},
  };
  for (const auto &e : pids)
    if (controller_nh.hasParam(e.first) && !e.second->init(ros::NodeHandle(controller_nh, e.first)))
      ROS_ERROR("Failed to load pid %s", e.first);
  pid_wheels_.push_back(&pid_left_wheel_vel_);
  pid_wheels_.push_back(&pid_right_wheel_vel_);
  pid_legs_.push_back(&pid_left_leg_);
  pid_legs_.push_back(&pid_right_leg_);
  pid_thetas_.push_back(&pid_left_leg_theta_);
  pid_thetas_.push_back(&pid_right_leg_theta_);
  pid_thetas_.push_back(&pid_left_leg_theta_vel_);
  pid_thetas_.push_back(&pid_right_leg_theta_vel_);
  pid_legs_stand_up_.push_back(&pid_left_leg_stand_up_);
  pid_legs_stand_up_.push_back(&pid_right_leg_stand_up_);

  mode_map_.emplace(BalanceMode::NORMAL,
                    std::make_shared<Normal>(controller, joint_handles, pid_legs_, &pid_yaw_vel_, &pid_theta_diff_,
                                             &pid_roll_, &pid_wheel_vel_diff_, pid_thetas_));
  mode_map_.emplace(BalanceMode::STAND_UP,
                    std::make_shared<StandUp>(controller, joint_handles, pid_legs_stand_up_, pid_thetas_));
  mode_map_.emplace(BalanceMode::SIT_DOWN, std::make_shared<SitDown>(controller, joint_handles, pid_wheels_));
  mode_map_.emplace(
      RECOVER, std::make_shared<Recover>(controller, joint_handles, pid_legs_stand_up_, pid_thetas_, &pid_theta_diff_));
  mode_map_.emplace(PROTECT, std::make_shared<Protect>(controller, joint_handles, pid_legs_, pid_thetas_, pid_wheels_,
                                                       &pid_theta_diff_, &pid_yaw_vel_));
  mode_map_.emplace(UPSTAIRS, std::make_shared<Upstairs>(controller, joint_handles, pid_legs_stand_up_, pid_thetas_));
}
void ModeManager::route()
{
  const bool request = controller_->getBaseState() == rm_msgs::ChassisCmd::RECOVERY;
  int mode = controller_->getBalanceMode();
  if (controller_->getBaseState() == rm_msgs::ChassisCmd::FALLEN)
    mode = SIT_DOWN;
  else if (request && !recovery_requested_)
    mode = RECOVER;
  recovery_requested_ = request;
  if (mode != controller_->getBalanceMode())
  {
    controller_->setMode(mode);
    controller_->setStateChange(false);
    controller_->setCompleteStand(false);
  }
  if (!controller_->getStateChange())
    switchMode(mode);
}
} // namespace rm_chassis_controllers
