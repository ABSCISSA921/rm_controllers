//
// Created by qiayuan on 2022/7/29.
//

#include "rm_chassis_controllers/active_suspension.h"

namespace rm_chassis_controllers
{
bool ActiveSuspensionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                      ros::NodeHandle& controller_nh)
{
  OmniController::init(robot_hw, root_nh, controller_nh);

  active_suspension_sub_ = controller_nh.subscribe("/cmd_active_suspension", 10,
                                                   &ActiveSuspensionController::ActiveSuspensionCallBack, this);

  XmlRpc::XmlRpcValue suspension_legs;
  controller_nh.getParam("suspension_leg", suspension_legs);
  stretch_coff_A_ = getParam(controller_nh, "stretch_coff_A_", 0.);
  stretch_coff_k_ = getParam(controller_nh, "stretch_coff_k_", 0.);
  shrink_coff_A_ = getParam(controller_nh, "shrink_coff_A_", 0.);
  shrink_coff_k_ = getParam(controller_nh, "shrink_coff_k_", 0.);
  feedforward_offset = getParam(controller_nh, "feed_forward_offset", 0.);
  feedforward_effect_time_ = getParam(controller_nh, "feedforward_effect_time_", 0.);

  for (const auto& suspension_leg : suspension_legs)
  {
    ros::NodeHandle nh_suspension = ros::NodeHandle(controller_nh, "suspension_leg/" + suspension_leg.first);
    active_suspension_joints_.push_back(std::make_shared<effort_controllers::JointPositionController>());
    if (!active_suspension_joints_.back()->init(effort_joint_interface_, nh_suspension))
    {
      ROS_ERROR("Failed to init active_suspension_joint");
      return false;
    }
  }
  if (!controller_nh.hasParam("imu_name"))
    has_imu_ = false;
  if (has_imu_)
  {
    imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("base_imu"));
    hardware_interface::ImuSensorInterface* imu_sensor_interface =
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_);
  }
  return true;
}
void ActiveSuspensionController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  OmniController::moveJoint(time, period);
  if (has_imu_)
  {
    odom2base_q.setValue(imu_sensor_handle_.getOrientation()[0], imu_sensor_handle_.getOrientation()[1],
                  imu_sensor_handle_.getOrientation()[2], imu_sensor_handle_.getOrientation()[3]);
    odom2base.setRotation(odom2base_q);
    odom2base.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Matrix3x3(odom2base.getRotation()).getRPY(base_roll_, base_pitch_, base_yaw_);
    base_roll_rate_ = imu_sensor_handle_.getAngularVelocity()[0];
    base_pitch_rate_ = imu_sensor_handle_.getAngularVelocity()[1];
    base_yaw_rate_ = imu_sensor_handle_.getAngularVelocity()[2];

    if (base_pitch_rate_ < -0.5 || base_pitch_ < -0.5)
    {
      current_state_ = State::UP;
    }
  }

  switch (current_state_)
  {
    case State::DOWN:
      target_pos_ = -0.05;
      static_effort = -6.0;
      break;
    case State::MID:
      target_pos_ = 0.30;
      static_effort = 9.0;
      break;
    case State::UP:
      target_pos_ = 0.80;
      static_effort = 8.0;
      break;
  }
  if (current_state_ != last_state_)
  {
    last_state_ = current_state_;
    feedforward_timer = ros::Time::now();
  }
  if ((ros::Time::now() - feedforward_timer).toSec() < feedforward_effect_time_)
  {
    if (current_state_ == State::MID)
      feedforward_effort = 1.2 * (stretch_coff_A_ - 45.0 * current_pos_);
    else if (current_state_ == State::UP)
      feedforward_effort = 2 * (stretch_coff_A_ + stretch_coff_k_ * current_pos_);
    else if (current_state_ == State::DOWN)
      feedforward_effort = shrink_coff_A_ + shrink_coff_k_ * current_pos_;
  }
  else
    feedforward_effort = 0.0;

  for (auto& joint : active_suspension_joints_)
  {
    joint->setCommand(target_pos_);
    joint->update(time, period);
    current_pos_ = joint->getPosition();
    auto cmd = joint->joint_.getCommand();
    cmd += feedforward_effort + feedforward_offset + static_effort;
    joint->joint_.setCommand(cmd);
  }
}
void ActiveSuspensionController::ActiveSuspensionCallBack(const rm_msgs::ChassisActiveSusCmd& msg)
{
  current_state_ = static_cast<State>(msg.mode);
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::ActiveSuspensionController, controller_interface::ControllerBase)
