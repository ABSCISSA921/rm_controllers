//
// Created by guanlin on 25-8-28.
//

#include "bipedal_wheel_controller/controller.h"
#include <urdf/model.h>
#include <algorithm>
#include <limits>

#include <angles/angles.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pluginlib/class_list_macros.hpp>
#include <unsupported/Eigen/MatrixFunctions>

namespace rm_chassis_controllers
{
bool BipedalController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                             ros::NodeHandle &controller_nh)
{
  // Stay seated until manual supplies an explicit chassis mode.
  cmd_struct_.cmd_chassis_.mode = rm_msgs::ChassisCmd::FALLEN;
  cmd_rt_buffer_.initRT(cmd_struct_);
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  //  gimbal_imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("gimbal_imu");
  const std::pair<const char *, hardware_interface::JointHandle *> table[] = {
      {"left_hip_joint", &left_hip_joint_handle_},     {"left_knee_joint", &left_knee_joint_handle_},
      {"right_hip_joint", &right_hip_joint_handle_},   {"right_knee_joint", &right_knee_joint_handle_},
      {"left_wheel_joint", &left_wheel_joint_handle_}, {"right_wheel_joint", &right_wheel_joint_handle_}};
  auto *joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  for (const auto &t : table)
  {
    *t.second = joint_interface->getHandle(t.first);
    joint_handles_.push_back(t.second);
  }

  if (!setupParams(controller_nh))
  {
    ROS_ERROR("[balance] Failed to setup parameters");
    return false;
  }

  mode_manager_ = std::make_shared<ModeManager>(this, controller_nh, joint_handles_);

  auto legCmdCallback = [this](const rm_msgs::LegCmd::ConstPtr &msg) {
    LegSetpoint command;
    command.length = msg->leg_length;
    command.jump = msg->jump;
    command.received = ros::Time::now();
    leg_setpoint_buffer_.writeFromNonRT(command);
  };
  auto recoveryLegSpdTurnbackCb = [this](const std_msgs::Bool::ConstPtr &msg) { setRecoveryLegSpdTurnback(msg->data); };
  leg_cmd_sub_ = controller_nh.subscribe<rm_msgs::LegCmd>("/leg_cmd", 5, legCmdCallback);
  recovery_leg_spd_turnback_sub_ =
      controller_nh.subscribe<std_msgs::Bool>("/recovery_leg_spd_turnback", 1, recoveryLegSpdTurnbackCb);
  unstick_pub_ = controller_nh.advertise<std_msgs::Bool>("unstick", 1);
  upstair_status_pub_ = controller_nh.advertise<rm_msgs::LeggedUpstairStatus>("upstair_status", 1);

  legged_chassis_mode_pub_.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>(controller_nh, "legged_chassis_mode", 1));
  legged_chassis_status_pub_.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisStatus>(controller_nh, "legged_chassis_status", 1));
  legged_chassis_status_pub_->msg_.linear_acc_base.resize(3);
  lqr_status_pub_.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::LeggedLQRStatus>(controller_nh, "lqr_status", 1));
  auto &status = lqr_status_pub_->msg_;
  status.left_leg_error.resize(6);
  status.right_leg_error.resize(6);
  status.left_leg_ref.resize(6);
  status.right_leg_ref.resize(6);
  status.left_leg_u.resize(2);
  status.right_leg_u.resize(2);
  status.F_leg.resize(2);
  status.unstick.resize(2);
  down_5cm_stair_srv_ =
      controller_nh.advertiseService("/down_5cm_stair", &BipedalController::down5cmStairSrvCallback, this);

  configureLqrInterface(controller_nh);
  return true;
}

bool BipedalController::down5cmStairSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  (void)req;
  triggerDown5cmStairAction();
  res.message = "Original down-5cm-stair posture thresholds armed";
  res.success = true;
  return true;
}

void BipedalController::moveJoint(const ros::Time &time, const ros::Duration &period)
{
  ++status_cycle_seq_;
  const bool status_due = time.toSec() - last_status_time_ >= .01 || time.toSec() < last_status_time_;
  if (status_due)
    last_status_time_ = time.toSec();
  const auto leg_setpoint = *leg_setpoint_buffer_.readFromRT();
  if (leg_setpoint.received != last_leg_setpoint_)
  {
    legCmd_ = leg_setpoint.length;
    jumpCmd_ = leg_setpoint.jump;
    last_leg_setpoint_ = leg_setpoint.received;
  }
  updateEstimation(time, period);
  chassis_state_.dt = period.toSec();
  updateChassisState();
  mode_manager_->route();
  publishMode(); // Report the mode selected for this cycle, before execute() can request the next one.
  if (balance_mode_ != NORMAL)
  {
    normal_feedback_.active = false;
    normal_feedback_.reference_valid = false;
    integrateVelocity(); // Mechanical modes use the current kinematic speed.
  }
  if (state_ == rm_msgs::ChassisCmd::FALLEN)
  {
    mode_manager_->getModeImpl()->execute(time, period);
    setJointCommands(joint_handles_, {0, 0, {0., 0.}}, {0, 0, {0., 0.}});
    if (status_due)
      publishCompatibilityStatus(time);
    return;
  }
  mode_manager_->getModeImpl()->execute(time, period);
  if (status_due)
    publishCompatibilityStatus(time);
  pubState();
}

void BipedalController::updateEstimation(const ros::Time &time, const ros::Duration &period)
{
  geometry_msgs::Vector3 gyro, acc;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  acc.x = imu_handle_.getLinearAcceleration()[0];
  acc.y = imu_handle_.getLinearAcceleration()[1];
  acc.z = imu_handle_.getLinearAcceleration()[2];
  tf2::Transform odom2imu, imu2base, odom2base;
  geometry_msgs::Vector3 angular_vel_base{}, linear_acc_base{};
  double roll{}, pitch{}, yaw{};
  try
  {
    const auto imu_to_base = robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time);
    tf2::doTransform(gyro, angular_vel_base, imu_to_base);
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
    tf2::Quaternion odom2imu_quaternion;
    tf2::Vector3 odom2imu_origin;
    odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                                 imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
    odom2imu_origin.setValue(0, 0, 0);
    odom2imu.setOrigin(odom2imu_origin);
    odom2imu.setRotation(odom2imu_quaternion);
    odom2base = odom2imu * imu2base;
    quatToRPY(toMsg(odom2base).rotation, roll, pitch, yaw);

    // IMU vectors are expressed in the sensor frame, including Gazebo's RelativeLinearAccel.
    tf2::doTransform(acc, linear_acc_base, imu_to_base);

    tf2::Vector3 z_body(0, 0, 1);
    tf2::Vector3 z_world = tf2::quatRotate(odom2base.getRotation(), z_body);
    overturn_ = (abs(pitch) > 0.65 || abs(roll) > 0.8) && z_world.z() < 0.0;
    const auto gravity_body = tf2::quatRotate(odom2base.getRotation().inverse(), tf2::Vector3(0, 0, 1));
    chassis_state_.recovery_pitch = std::atan2(-gravity_body.x(), gravity_body.z());
    // Side inclination from gravity stays zero for a sagittal flip beyond 90 degrees.
    chassis_state_.recovery_roll = std::atan2(gravity_body.y(), std::hypot(gravity_body.x(), gravity_body.z()));
    chassis_state_.upright_z = z_world.z();

    chassis_state_.angular_vel = angular_vel_base;
    chassis_state_.linear_acc = linear_acc_base;
    chassis_state_.roll = roll;
    chassis_state_.pitch = pitch;
    chassis_state_.yaw = yaw;
    const auto &imu_translation = imu_to_base.transform.translation;
    imu_position_base_ << imu_translation.x, imu_translation.y, imu_translation.z;
    const auto &rotation = odom2base.getBasis();
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        rotation_world_base_(row, col) = rotation[row][col];
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_ONCE("%s", ex.what());
    setJointCommands(joint_handles_, {0, 0, {0., 0.}}, {0, 0, {0., 0.}});
    return;
  }

  // vmc
  double left_angle[2]{}, right_angle[2]{};

  //  double left_pos[2]{}, left_spd[2]{}, right_pos[2]{}, right_spd[2]{};
  // [0]:hip_vmc_joint [1]:knee_vmc_joint
  left_angle[0] = left_hip_joint_handle_.getPosition() + M_PI;
  left_angle[1] = left_knee_joint_handle_.getPosition();
  right_angle[0] = right_hip_joint_handle_.getPosition() + M_PI;
  right_angle[1] = right_knee_joint_handle_.getPosition();

  //  gazebo
  //  left_angle[0] = left_hip_joint_handle_.getPosition() + M_PI_2;
  //  left_angle[1] = left_knee_joint_handle_.getPosition() - M_PI_2;
  //  right_angle[0] = right_hip_joint_handle_.getPosition() + M_PI_2;
  //  right_angle[1] = right_knee_joint_handle_.getPosition() - M_PI_2;

  // left vmc calc
  leg_state_[LEFT].vmc->calc_jacobian(left_angle[0], left_angle[1]);
  leg_state_[LEFT].vmc->leg_pos(left_angle[0], left_angle[1]);
  leg_state_[LEFT].vmc->leg_spd(left_hip_joint_handle_.getVelocity(), left_knee_joint_handle_.getVelocity());

  // right vmc calc
  leg_state_[RIGHT].vmc->calc_jacobian(right_angle[0], right_angle[1]);
  leg_state_[RIGHT].vmc->leg_pos(right_angle[0], right_angle[1]);
  leg_state_[RIGHT].vmc->leg_spd(right_hip_joint_handle_.getVelocity(), right_knee_joint_handle_.getVelocity());

}

void BipedalController::updateChassisState()
{
  auto &chassis = chassis_state_;
  const double pitch_rate = chassis.angular_vel.y * std::cos(chassis.roll) -
                            chassis.angular_vel.z * std::sin(chassis.roll);
  Eigen::Vector2d carrier_rates;
  for (int side = 0; side < 2; ++side)
    readVirtualLeg(side, pitch_rate, carrier_rates[side]);
  observe(pitch_rate, carrier_rates);
  for (int side = 0; side < 2; ++side)
  {
    leg_state_[side].posture.theta = chassis.x[THETA_L + 2 * side];
    leg_state_[side].posture.rate = chassis.x[DTHETA_L + 2 * side];
  }
}

void BipedalController::publishMode()
{
  if (legged_chassis_mode_pub_->trylock())
  {
    legged_chassis_mode_pub_->msg_.mode = balance_mode_;
    legged_chassis_mode_pub_->msg_.mode_name = mode_manager_->getModeImpl()->name();
    legged_chassis_mode_pub_->unlockAndPublish();
  }
}

void BipedalController::publishCompatibilityStatus(const ros::Time &time)
{
  const auto &x = chassis_state_.x;
  if (legged_chassis_status_pub_->trylock())
  {
    auto &m = legged_chassis_status_pub_->msg_;
    m.header.stamp = time;
    m.cycle_seq = status_cycle_seq_;
    m.roll = chassis_state_.roll;
    m.pitch = -x[BODY_PITCH];
    m.d_pitch = -x[BODY_RATE];
    m.yaw = x[YAW];
    m.d_yaw = x[YAW_RATE];
    m.x = x[S];
    m.x_dot = x[V];
    m.left_leg_length = chassis_state_.length[0];
    m.right_leg_length = chassis_state_.length[1];
    m.left_leg_theta = x[THETA_L];
    m.left_leg_theta_dot = x[DTHETA_L];
    m.right_leg_theta = x[THETA_R];
    m.right_leg_theta_dot = x[DTHETA_R];
    m.linear_acc_base[0] = chassis_state_.linear_acc.x;
    m.linear_acc_base[1] = chassis_state_.linear_acc.y;
    m.linear_acc_base[2] = chassis_state_.linear_acc.z;
    legged_chassis_status_pub_->unlockAndPublish();
  }
  if (lqr_status_pub_->trylock())
  {
    // Read-only original six-field display projection; not six-state feedback or K6.
    // [theta, dtheta, s, v, negative ROS pitch, negative ROS pitch rate].
    auto &m = lqr_status_pub_->msg_;
    const auto &f = normal_feedback_;
    m.header.stamp = time;
    m.cycle_seq = status_cycle_seq_;
    m.normal_active = f.active;
    m.reference_valid = f.reference_valid;
    m.yaw_ref = f.reference_valid ? f.reference[YAW] : x[YAW];
    m.yaw_rate_ref = f.reference_valid ? f.reference[YAW_RATE] : x[YAW_RATE];
    for (int side = 0; side < 2; ++side)
    {
      const int index[] = {THETA_L + 2 * side, DTHETA_L + 2 * side, S, V, BODY_PITCH, BODY_RATE};
      auto &reference = side == 0 ? m.left_leg_ref : m.right_leg_ref;
      auto &error = side == 0 ? m.left_leg_error : m.right_leg_error;
      auto &input = side == 0 ? m.left_leg_u : m.right_leg_u;
      for (int i = 0; i < 6; ++i)
      {
        const double sign = i < 4 ? 1. : -1.;
        reference[i] = sign * (f.reference_valid ? f.reference[index[i]] : x[index[i]]);
        error[i] = sign * x[index[i]] - reference[i];
      }
      input[0] = f.active ? f.output[side] : 0.;
      input[1] = f.active ? f.output[2 + side] : 0.;
      m.F_leg[side] = f.active ? f.axial_force[side] : 0.;
      // The original three interfaces do not provide per-wheel contact truth.
      m.unstick[side] = false;
    }
    lqr_status_pub_->unlockAndPublish();
  }
}

void BipedalController::pubState()
{
  std_msgs::Bool msg;
  msg.data = mode_manager_->getModeImpl()->getUnstick();
  unstick_pub_.publish(msg);
}

void BipedalController::starting(const ros::Time &time)
{
  mpc_bank_.reset();
  normal_feedback_ = NormalFeedback{};
  velocity_kf_.reset();
  balance_mode_ = SIT_DOWN;
  state_ = rm_msgs::ChassisCmd::FALLEN;
  complete_stand_ = overturn_ = false;
  setStateChange(false);
  mode_manager_->reset();
  mode_manager_->switchMode(SIT_DOWN);
  vel_cmd_ = geometry_msgs::Vector3{};
  ramp_x_->clear(0.);
  ramp_y_->clear(0.);
  ramp_w_->clear(0.);
  ROS_INFO("[lqr10] Started");
}

void BipedalController::stopping(const ros::Time &time)
{
  mpc_bank_.reset();
  velocity_kf_.reset();
  normal_feedback_ = NormalFeedback{};
  balance_mode_ = BalanceMode::SIT_DOWN;
  setStateChange(false);
  setJointCommands(joint_handles_, {0, 0, {0., 0.}}, {0, 0, {0., 0.}});

  ROS_INFO("[balance] Controller Stop");
}

bool BipedalController::setupParams(ros::NodeHandle &controller_nh)
{
  controller_nh.param("recovery_leg_speed", action_params_.recovery_leg_speed, action_params_.recovery_leg_speed);
  const std::pair<const char *, double *> action_parameters[] = {
      {"jumpOverTime", &action_params_.jumpOverTime_},
      {"down5cmStairPitchThreshold", &action_params_.down5cmStairPitchThreshold},
      {"down5cmStairThetaThreshold", &action_params_.down5cmStairThetaThreshold},
      {"jump_up_force", &action_params_.jump_up_force},
      {"off_ground_force", &action_params_.off_ground_force}};
  for (const auto &p : action_parameters)
    if (!controller_nh.getParam(p.first, *p.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", p.first, controller_nh.getNamespace().c_str());
      return false;
    }
  model_params_ = std::make_shared<ModelParams>();
  leg_threshold_params_ = std::make_shared<LegStateThresholdParams>();
  spring_params_ = std::make_shared<SpringParams>();
  chassis_geometry_params_ = std::make_shared<ChassisGeometryParams>();

  if (!setupModelParams(controller_nh) || !setupThresholdParams(controller_nh) || !setupSpringParams(controller_nh) ||
      !setupChassisGeometryParams(controller_nh))
    return false;
  setupVelocityEstimator(controller_nh);
  return setupFeedbackParams(controller_nh) && setupMpc(controller_nh);
}

bool BipedalController::setupModelParams(ros::NodeHandle &controller_nh)
{
  const std::pair<const char *, double *> tbl[] = {{"M", &model_params_->M},
                                                   {"gravity_force", &model_params_->f_gravity}};

  for (const auto &e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }

  double l1, l2;
  if (!controller_nh.getParam("l1", l1) || !controller_nh.getParam("l2", l2))
  {
    ROS_ERROR("Param %s or %s not given (namespace: %s)", "l1", "l2", controller_nh.getNamespace().c_str());
    return false;
  }
  leg_state_[LEFT].vmc = std::make_shared<VMC>(l1, l2);
  leg_state_[RIGHT].vmc = std::make_shared<VMC>(l1, l2);

  if (!controller_nh.getParam("default_leg_length", default_leg_length_))
  {
    ROS_ERROR("Param %s not given (namespace: %s)", "default_leg_length", controller_nh.getNamespace().c_str());
    return false;
  }

  return true;
}

bool BipedalController::setupChassisGeometryParams(ros::NodeHandle &controller_nh)
{
  const std::pair<const char *, double *> tbl[] = {{"wheel_track", &chassis_geometry_params_->wheel_track},
                                                   {"chassis_high", &chassis_geometry_params_->chassis_height}};

  for (const auto &e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }

  return true;
}

// [will unused]
bool BipedalController::setupThresholdParams(ros::NodeHandle &controller_nh)
{
  const std::pair<const char *, double *> tbl[] = {
      {"under_lower_threshold", &leg_threshold_params_->under_lower},
      {"under_upper_threshold", &leg_threshold_params_->under_upper},
      {"front_lower_threshold", &leg_threshold_params_->front_lower},
      {"front_upper_threshold", &leg_threshold_params_->front_upper},
      {"behind_lower_threshold", &leg_threshold_params_->behind_lower},
      {"behind_upper_threshold", &leg_threshold_params_->behind_upper},
      {"upstair_exit_theta_threshold", &leg_threshold_params_->upstair_exit_theta_threshold},
      {"upstair_exit_length_threshold", &leg_threshold_params_->upstair_exit_length_threshold},
      {"upstair_des_theta", &leg_threshold_params_->upstair_des_theta},
      {"upstair_des_length", &leg_threshold_params_->upstair_des_length},
      {"unstick_threshold", &leg_threshold_params_->unstick_threshold},
      {"arrive_time_threshold", &leg_threshold_params_->arrive_time_threshold}};
  for (const auto &e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

bool BipedalController::setupSpringParams(ros::NodeHandle &controller_nh)
{
  const std::pair<const char *, double *> tbl[] = {
      {"spring_s2", &spring_params_->s2},
      {"spring_s3", &spring_params_->s3},
      {"spring_alpha_s", &spring_params_->alpha_s},
      {"spring_force", &spring_params_->f_spring},
  };

  for (const auto &e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

geometry_msgs::Twist BipedalController::odometry()
{
  geometry_msgs::Twist twist;
  if (mode_manager_->getModeImpl() != nullptr)
  {
    twist.linear.x = chassis_state_.x_vel;
    twist.angular.z = chassis_state_.angular_vel.z;
  }
  else
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
  }
  return twist;
}

void BipedalController::pubLegLenStatus(const bool &upstair_flag)
{
  rm_msgs::LeggedUpstairStatus msg;
  msg.upstair_flag = upstair_flag;
  upstair_status_pub_.publish(msg);
}

double BipedalController::f_spring_force(double L0)
{
  // return 0.;
  return ((2094.45f * L0 - 3091.28f) * L0 + 1408.375f) * L0 - 80.91f;
}

void BipedalController::readVirtualLeg(int side, double pitch_rate, double &carrier_rate)
{
  auto &chassis = chassis_state_;
  auto &vmc = *leg_state_[side].vmc;
  const double hip = joint_handles_[2 * side]->getPosition();
  const double hip_rate = joint_handles_[2 * side]->getVelocity();
  const double knee_rate = joint_handles_[2 * side + 1]->getVelocity();
  const auto &pos = vmc.getPos();
  double force_column[2], torque_column[2];
  vmc.leg_conv(1., 0., force_column);
  vmc.leg_conv(0., 1., torque_column);
  Eigen::Matrix2d jacobian;
  jacobian << force_column[0], force_column[1], torque_column[0], torque_column[1];
  const Eigen::Vector2d velocity = jacobian * Eigen::Vector2d(hip_rate, knee_rate);
  chassis.length[side] = pos.L0;
  chassis.dlength[side] = velocity[0];
  chassis.x[THETA_L + 2 * side] = pos.theta + chassis.pitch;
  chassis.x[DTHETA_L + 2 * side] = velocity[1] + pitch_rate;
  const double s = std::sin(pos.theta), c = std::cos(pos.theta), phi = hip + M_PI;
  const Eigen::Vector2d point(-pos.L0 * s, pos.L0 * c);
  const Eigen::Vector2d point_rate(-velocity[0] * s - pos.L0 * c * velocity[1],
                                   velocity[0] * c - pos.L0 * s * velocity[1]);
  const Eigen::Vector2d b(vmc.getL1() * std::cos(phi), vmc.getL1() * std::sin(phi));
  const Eigen::Vector2d b_rate(-b[1] * hip_rate, b[0] * hip_rate);
  const Eigen::Vector2d calf = point - b, calf_rate = point_rate - b_rate;
  const double calf_norm_sq = calf.squaredNorm();
  carrier_rate = (calf[0] * calf_rate[1] - calf[1] * calf_rate[0]) / calf_norm_sq;
}

void BipedalController::observe(double pitch_rate, const Eigen::Vector2d &carrier_rates)
{
  auto &out = chassis_state_;
  const Eigen::Vector2d wheel_rates(joint_handles_[4]->getVelocity(), joint_handles_[5]->getVelocity());
  const Eigen::Vector3d rpy(out.roll, out.pitch, out.yaw);
  const Eigen::Vector3d omega_base(out.angular_vel.x, out.angular_vel.y, out.angular_vel.z);
  const double radius = lqr_config_.wheel_radius;
  const double lateral_rate = omega_base[1] * std::sin(rpy[0]) + omega_base[2] * std::cos(rpy[0]);
  const double yaw_rate = lateral_rate / std::cos(rpy[1]);
  out.roll_rate = omega_base[0] + std::tan(rpy[1]) * lateral_rate;
  double v = 0.;
  for (int side = 0; side < 2; ++side)
    v += radius * (wheel_rates[side] + carrier_rates[side] + pitch_rate) / 2.;
  out.x[V] = v;
  // Keep the ten-state heading continuous across the IMU's +/-pi wrap.
  out.x[YAW] += std::remainder(rpy[2] - out.x[YAW], 2 * M_PI);
  out.x[YAW_RATE] = yaw_rate;
  out.x[BODY_PITCH] = rpy[1];
  out.x[BODY_RATE] = pitch_rate;
  out.roll = rpy[0];
}
// Called only by the ordinary Normal ground branch, before circle and K10.
void BipedalController::updateGroundVelocity()
{
  auto &chassis = chassis_state_;
  VelocityKalmanFilter::Input input;
  input.rotation_world_base = rotation_world_base_;
  input.yaw = chassis.yaw;
  input.yaw_rate = chassis.x[YAW_RATE];
  input.specific_force_base << chassis.linear_acc.x, chassis.linear_acc.y, chassis.linear_acc.z;
  input.angular_velocity_base << chassis.angular_vel.x, chassis.angular_vel.y, chassis.angular_vel.z;
  input.imu_position_base = imu_position_base_;
  // VMC endpoint approximation of the physical wheel midpoint. Keep this
  // geometry boundary outside the KF; it does not change VMC's force mapping.
  input.wheel_midpoint_base = hip_midpoint_base_;
  input.wheel_midpoint_rate_base.setZero();
  for (int side = 0; side < 2; ++side)
  {
    const double length = chassis.length[side], length_rate = chassis.dlength[side];
    const double angle = leg_state_[side].vmc->getPos().theta;
    const double angle_rate = chassis.x[DTHETA_L + 2 * side] - chassis.x[BODY_RATE];
    const double sine = std::sin(angle), cosine = std::cos(angle);
    input.wheel_midpoint_base += .5 * Eigen::Vector3d(-length * sine, 0., -length * cosine);
    input.wheel_midpoint_rate_base += .5 * Eigen::Vector3d(
        -length_rate * sine - length * cosine * angle_rate, 0.,
        -length_rate * cosine + length * sine * angle_rate);
  }
  input.kinematic_velocity = chassis.x[V];
  input.dt = chassis.dt;
  VelocityKalmanFilter::Output output;
  velocity_kf_.update(input, output);
  chassis.x[V] = output.velocity;
  integrateVelocity();
}

void BipedalController::integrateVelocity()
{
  if (state_ != RAW && std::abs(chassis_state_.x[V]) <= 0.5 && std::abs(vel_cmd_.x) <= 0.01)
    chassis_state_.x[S] += chassis_state_.x[V] * chassis_state_.dt;
  else
    chassis_state_.x[S] = 0.;
  chassis_state_.x_vel = chassis_state_.x[V];
}

namespace
{
template <int Rows>
bool getVectorParam(const ros::NodeHandle &nh, const std::string &name, Eigen::Matrix<double, Rows, 1> &out)
{
  std::vector<double> values;
  if (!nh.getParam(name, values))
  {
    ROS_ERROR("Param %s not given or not numeric (namespace: %s)", name.c_str(), nh.getNamespace().c_str());
    return false;
  }
  if (values.size() != Rows)
  {
    ROS_ERROR("Param %s must contain %d values (namespace: %s)", name.c_str(), Rows, nh.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < Rows; ++i)
    out[i] = values[i];
  return true;
}
} // namespace

void BipedalController::setupVelocityEstimator(ros::NodeHandle &controller_nh)
{
  VelocityKalmanFilter::Params params;
  const std::pair<const char *, double *> parameters[] = {
      {"velocity_estimator/velocity_process_density", &params.velocity_process_density},
      {"velocity_estimator/jerk_density", &params.jerk_density},
      {"velocity_estimator/velocity_stddev", &params.velocity_stddev},
      {"velocity_estimator/acceleration_stddev", &params.acceleration_stddev},
      {"velocity_estimator/velocity_filter_tau", &params.velocity_filter_tau},
      {"velocity_estimator/acceleration_filter_tau", &params.acceleration_filter_tau},
      {"velocity_estimator/velocity_residual_threshold", &params.velocity_residual_threshold},
      {"velocity_estimator/velocity_noise_multiplier", &params.velocity_noise_multiplier}};
  for (const auto &parameter : parameters)
    controller_nh.getParam(parameter.first, *parameter.second);
  velocity_kf_.configure(params);
  // Read only installation origins at init; runtime wheel geometry comes from the existing VMC.
  urdf::Model robot;
  robot.initParam("robot_description");
  hip_midpoint_base_.setZero();
  for (const char *name : {"left_hip_joint", "right_hip_joint"})
  {
    const auto joint = robot.getJoint(name);
    const auto &position = joint->parent_to_joint_origin_transform.position;
    hip_midpoint_base_ += .5 * Eigen::Vector3d(position.x, position.y, position.z);
  }
}

bool BipedalController::loadLqrParams(ros::NodeHandle &controller_nh, Config &config)
{
  auto &model = config.model;
  double leg_mass{};
  const std::pair<const char *, double *> parameters[] = {
      {"lqr_model/gravity", &model.g},
      {"lqr_model/yaw_inertia", &model.Izz},
      {"lqr_model/body/mass", &model.mb},
      {"lqr_model/body/pitch_inertia", &model.Ib},
      {"lqr_model/body/com_distance", &model.lc},
      {"lqr_model/body/com_angle", &model.phi_c},
      {"lqr_model/wheel/mass", &model.mw},
      {"lqr_model/wheel/spin_inertia", &model.Iw},
      {"lqr_model/leg/mass", &leg_mass},
      {"normal_reference/length_tau", &config.length_reference_tau}};
  for (const auto &entry : parameters)
    if (!controller_nh.getParam(entry.first, *entry.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", entry.first, controller_nh.getNamespace().c_str());
      return false;
    }

  Eigen::Vector2d length_domain;
  if (!getVectorParam(controller_nh, "q", config.q) || !getVectorParam(controller_nh, "r", config.r) ||
      !getVectorParam(controller_nh, "lqr10/length_domain", length_domain))
    return false;
  config.domain.row(0) = length_domain.transpose();
  config.domain.row(1) = length_domain.transpose();
  model.Rw = wheel_radius_;
  model.Rl = 0.5 * chassis_geometry_params_->wheel_track;
  model.geometry_domain = length_domain;
  model.legs.fill(makePhysicalLeg(leg_mass));
  config.uff = staticFeedforward(model);
  config.wheel_radius = wheel_radius_;
  return true;
}

bool BipedalController::setupFeedbackParams(ros::NodeHandle& controller_nh)
{
  Config config;
  if (!loadLqrParams(controller_nh, config))
    return false;
  if (!config.q.allFinite() || !config.r.allFinite())
    return false;
  if ((config.q.array() < 0.).any() || (config.r.array() <= 0.).any())
  {
    ROS_ERROR("[mpc] Q must be positive semidefinite and R positive definite");
    return false;
  }

  lqr_config_ = config;
  return true;
}

bool BipedalController::setupMpc(ros::NodeHandle& nh)
{
  auto& params = mpc_params_;
  params.sample_period = .001;
  params.max_solve_time = .008;
  params.q = lqr_config_.q;
  params.r = lqr_config_.r;
  double wheel_limit = 3.58;
  nh.param("mpc/wheel_torque_limit", wheel_limit, wheel_limit);
  nh.param("mpc/horizon", params.horizon, params.horizon);
  nh.param("mpc/rho", params.rho, params.rho);
  nh.param("mpc/max_iter", params.max_iter, params.max_iter);
  nh.param("mpc/abs_pri_tol", params.abs_pri_tol, params.abs_pri_tol);
  nh.param("mpc/abs_dua_tol", params.abs_dua_tol, params.abs_dua_tol);
  nh.param("mpc/max_solve_time", params.max_solve_time, params.max_solve_time);
  if (!std::isfinite(wheel_limit) || wheel_limit <= 0.)
  {
    ROS_ERROR("[mpc] wheel_torque_limit must be finite and positive");
    return false;
  }
  const double unlimited = std::numeric_limits<double>::infinity();
  params.input_min << -wheel_limit, -wheel_limit, -unlimited, -unlimited;
  params.input_max << wheel_limit, wheel_limit, unlimited, unlimited;
  const auto& domain = lqr_config_.model.geometry_domain;
  if (!domain.allFinite() || domain[0] >= domain[1] || !std::isfinite(default_leg_length_) ||
      default_leg_length_ < domain[0] || default_leg_length_ > domain[1])
  {
    ROS_ERROR("[mpc] Invalid length domain or default_leg_length outside domain");
    return false;
  }
  std::vector<double> lengths;
  if (nh.hasParam("mpc/leg_lengths"))
  {
    if (!nh.getParam("mpc/leg_lengths", lengths))
    {
      ROS_ERROR("[mpc] leg_lengths must be an increasing numeric array covering the length domain");
      return false;
    }
  }
  else
  {
    for (int i = 0; i < 12; ++i)
      lengths.push_back(domain[0] + (domain[1] - domain[0]) * i / 11.);
    lengths.back() = domain[1];
    if (std::none_of(lengths.begin(), lengths.end(),
                     [&](double length) { return std::abs(length - default_leg_length_) < 1e-9; }))
      lengths.push_back(default_leg_length_);
    std::sort(lengths.begin(), lengths.end());
  }
  double hysteresis = .0005;
  nh.param("mpc/length_hysteresis", hysteresis, hysteresis);
  const auto start = ros::WallTime::now();
  try
  {
    if (!mpc_bank_.initialize(lqr_config_.model, params, lengths, hysteresis))
    {
      ROS_ERROR("[mpc] Bank initialization failed: check model/settings, increasing leg_lengths with domain endpoints, "
                "and nonnegative length_hysteresis smaller than half the minimum grid spacing");
      return false;
    }
    mpc_reference_.resize(10, params.horizon);
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("[mpc] Bank initialization failed: %s", ex.what());
    return false;
  }
  ROS_INFO("[mpc] Built %zu backends (%zu x %zu) in %.3f s: fixed 1 kHz, N=%d, "
           "hysteresis=%g m, wheel bounds=+-%g Nm, virtual legs unbounded; Q/R require reload",
           mpc_bank_.size(), lengths.size(), lengths.size(), (ros::WallTime::now() - start).toSec(), params.horizon,
           hysteresis, wheel_limit);
  return true;
}

namespace
{
LQRWeightConfig fixedMpcWeights(const Config& config)
{
  LQRWeightConfig weights;
  weights.Q_s = config.q[0];
  weights.Q_v = config.q[1];
  weights.Q_yaw = config.q[2];
  weights.Q_yaw_rate = config.q[3];
  weights.Q_theta_L = config.q[4];
  weights.Q_dtheta_L = config.q[5];
  weights.Q_theta_R = config.q[6];
  weights.Q_dtheta_R = config.q[7];
  weights.Q_phi = config.q[8];
  weights.Q_phi_dot = config.q[9];
  weights.R_T_L = config.r[0];
  weights.R_T_R = config.r[1];
  weights.R_Tp_L = config.r[2];
  weights.R_Tp_R = config.r[3];
  weights.update_status = "read_only: reload controller to rebuild MPC bank";
  return weights;
}
}  // namespace

void BipedalController::configureLqrInterface(ros::NodeHandle &nh)
{
  lqr_server_ = std::make_unique<dynamic_reconfigure::Server<LQRWeightConfig>>(nh);
  auto weights = fixedMpcWeights(lqr_config_);
  lqr_server_->updateConfig(weights);
  lqr_server_->setCallback(boost::bind(&BipedalController::reconfigCB, this, _1, _2));
}
void BipedalController::reconfigCB(LQRWeightConfig &request, uint32_t /*level*/)
{
  State10 q;
  Input4 r;
  q << request.Q_s, request.Q_v, request.Q_yaw, request.Q_yaw_rate, request.Q_theta_L, request.Q_dtheta_L,
      request.Q_theta_R, request.Q_dtheta_R, request.Q_phi, request.Q_phi_dot;
  r << request.R_T_L, request.R_T_R, request.R_Tp_L, request.R_Tp_R;
  if ((q.array() != lqr_config_.q.array()).any() || (r.array() != lqr_config_.r.array()).any())
    ROS_WARN("[mpc] Q/R are read-only; reload the controller to rebuild the backend bank with new weights");
  request = fixedMpcWeights(lqr_config_);
}

} // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BipedalController, controller_interface::ControllerBase)
