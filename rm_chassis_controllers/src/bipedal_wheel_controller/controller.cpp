//
// Created by guanlin on 25-8-28.
//

#include "bipedal_wheel_controller/controller.h"
#include <stdexcept>
#include <urdf/model.h>

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
  if (!ChassisBase::init(robot_hw, root_nh, controller_nh))
    return false;

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
  auto zero_output = [this, status_due, &time]() {
    finishObservation();
    setJointCommands(joint_handles_, {0, 0, {0., 0.}}, {0, 0, {0., 0.}});
    if (status_due)
      publishCompatibilityStatus(time);
  };
  resetObservation(time, period);
  if (!std::isfinite(period.toSec()) || period.toSec() <= 0.)
  {
    latchControlFault(lqr10::Reason::InvalidTime);
    zero_output();
    return;
  }
  if (!updateEstimation(time, period))
  {
    if (command_authorized_)
      latchControlFault(lqr10::Reason::InvalidSnapshot);
    zero_output();
    return;
  }
  updateChassisState();
  if (normal_feedback_.faulted)
  {
    publishMode();
    zero_output();
    return;
  }
  const Command command = *cmd_rt_buffer_.readFromRT();
  // Reuse this derived controller snapshot and the already evaluated XY ramps.
  // Capture also requires zero Normal Vref; no changes to ChassisBase processing.
  translation_source_active_ = command.cmd_vel_.linear.x != 0. || command.cmd_vel_.linear.y != 0.;
  translation_ramp_zero_ = ramp_x_->output() == 0. && ramp_y_->output() == 0.;
  command_authorized_ =
      command_authorized_ || (command.stamp_ > start_time_ && command.cmd_chassis_.stamp > start_time_);
  if (!command_authorized_)
    state_ = rm_msgs::ChassisCmd::FALLEN;
  mode_manager_->route();
  publishMode(); // Report the mode selected for this cycle, before execute() can request the next one.
  if (balance_mode_ != NORMAL)
  {
    normal_feedback_.active = false;
    normal_feedback_.reference_valid = false;
    normal_feedback_.reason = lqr10::Reason::NotNormal;
  }
  if (state_ == rm_msgs::ChassisCmd::FALLEN)
  {
    mode_manager_->getModeImpl()->execute(time, period);
    zero_output();
    return;
  }
  const auto &chassis = chassis_state_;
  if (!chassis.raw_valid)
  {
    latchControlFault(lqr10::Reason::InvalidSnapshot);
    zero_output();
    return;
  }
  mode_manager_->getModeImpl()->execute(time, period);
  bool finite = true;
  for (const auto *handle : joint_handles_)
    finite = finite && std::isfinite(handle->getCommand());
  if (!finite || normal_feedback_.faulted)
  {
    if (!normal_feedback_.faulted)
      latchControlFault(lqr10::Reason::NonfiniteOutput);
    zero_output();
    return;
  }
  finishObservation();
  if (status_due)
    publishCompatibilityStatus(time);
  pubState();
}

bool BipedalController::updateEstimation(const ros::Time &time, const ros::Duration &period)
{
  for (const auto *joint : joint_handles_)
    if (!std::isfinite(joint->getPosition()) || !std::isfinite(joint->getVelocity()) ||
        !std::isfinite(joint->getEffort()))
      return false;
  double quaternion_norm = 0.;
  for (int i = 0; i < 4; ++i)
  {
    const double value = imu_handle_.getOrientation()[i];
    if (!std::isfinite(value))
      return false;
    quaternion_norm += value * value;
  }
  if (std::abs(quaternion_norm - 1.) > 1e-3)
    return false;
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(imu_handle_.getAngularVelocity()[i]) || !std::isfinite(imu_handle_.getLinearAcceleration()[i]))
      return false;
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
    return false;
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

  return true;
}

void BipedalController::resetObservation(const ros::Time &time, const ros::Duration &period)
{
  velocity_integrated_ = false;
  // Invalidate this cycle before reading feedback; do not clear the original action/odometry state.
  chassis_state_.x.setZero();
  chassis_state_.length.setZero();
  chassis_state_.dlength.setZero();
  chassis_state_.time = time.toSec();
  chassis_state_.dt = period.toSec();
  chassis_state_.valid = chassis_state_.raw_valid = false;
  chassis_state_.observation_reason = lqr10::Reason::InvalidSnapshot;
  chassis_state_.roll_rate = 0.;
}

void BipedalController::updateChassisState()
{
  auto &chassis = chassis_state_;
  const double pitch_rate = chassis.angular_vel.y * std::cos(chassis.roll) -
                            chassis.angular_vel.z * std::sin(chassis.roll);
  Eigen::Vector2d carrier_rates;
  bool valid = true;
  for (int side = 0; side < 2; ++side)
    valid = readVirtualLeg(side, pitch_rate, carrier_rates[side]) && valid;
  try
  {
    if (state_ == FOLLOW)
    {
      robot_state_handle_.lookupTransform("base_link", follow_source_frame_, ros::Time(0));
      robot_state_handle_.lookupTransform("base_link", command_source_frame_, ros::Time(0));
    }
  }
  catch (const tf2::TransformException &)
  {
    valid = false;
  }
  chassis.raw_valid = valid;
  if (valid)
    observe(pitch_rate, carrier_rates);
  if (chassis.valid)
  {
    for (int side = 0; side < 2; ++side)
    {
      leg_state_[side].posture.theta = chassis.x[lqr10::THETA_L + 2 * side];
      leg_state_[side].posture.rate = chassis.x[lqr10::DTHETA_L + 2 * side];
    }
  }
}

void BipedalController::publishMode()
{
  if (legged_chassis_mode_pub_->trylock())
  {
    legged_chassis_mode_pub_->msg_.mode = balance_mode_;
    static const char *mode_names[] = {"NORMAL", "STAND_UP", "SIT_DOWN", "RECOVER", "UPSTAIRS", "PROTECT"};
    legged_chassis_mode_pub_->msg_.mode_name =
        balance_mode_ >= 0 && balance_mode_ < 6 ? mode_names[balance_mode_] : "INVALID";
    legged_chassis_mode_pub_->unlockAndPublish();
  }
}

void BipedalController::publishCompatibilityStatus(const ros::Time &time)
{
  const auto &x = chassis_state_.x;
  if (chassis_state_.valid && legged_chassis_status_pub_->trylock())
  {
    auto &m = legged_chassis_status_pub_->msg_;
    m.header.stamp = time;
    m.cycle_seq = status_cycle_seq_;
    m.roll = chassis_state_.roll;
    m.pitch = -x[lqr10::BODY_PITCH];
    m.d_pitch = -x[lqr10::BODY_RATE];
    m.yaw = x[lqr10::YAW];
    m.d_yaw = x[lqr10::YAW_RATE];
    m.x = x[lqr10::S];
    m.x_dot = x[lqr10::V];
    m.left_leg_length = chassis_state_.length[0];
    m.right_leg_length = chassis_state_.length[1];
    m.left_leg_theta = x[lqr10::THETA_L];
    m.left_leg_theta_dot = x[lqr10::DTHETA_L];
    m.right_leg_theta = x[lqr10::THETA_R];
    m.right_leg_theta_dot = x[lqr10::DTHETA_R];
    m.linear_acc_base[0] = chassis_state_.linear_acc.x;
    m.linear_acc_base[1] = chassis_state_.linear_acc.y;
    m.linear_acc_base[2] = chassis_state_.linear_acc.z;
    legged_chassis_status_pub_->unlockAndPublish();
  }
  if (chassis_state_.valid && lqr_status_pub_->trylock())
  {
    // Read-only original six-field display projection; not six-state feedback or K6.
    // [theta, dtheta, s, v, negative ROS pitch, negative ROS pitch rate].
    auto &m = lqr_status_pub_->msg_;
    const auto &f = normal_feedback_;
    m.header.stamp = time;
    m.cycle_seq = status_cycle_seq_;
    m.normal_active = f.active;
    m.reference_valid = f.reference_valid;
    m.yaw_ref = f.reference_valid ? f.reference[lqr10::YAW] : x[lqr10::YAW];
    m.yaw_rate_ref = f.reference_valid ? f.reference[lqr10::YAW_RATE] : x[lqr10::YAW_RATE];
    for (int side = 0; side < 2; ++side)
    {
      const int index[] = {lqr10::THETA_L + 2 * side, lqr10::DTHETA_L + 2 * side, lqr10::S, lqr10::V, lqr10::BODY_PITCH,
                           lqr10::BODY_RATE};
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
  control_running_.store(true, std::memory_order_release);
  // One complete fixed-size transaction, only before the new start is authorized.
  const auto pending = *gain_update_buffer_.readFromRT();
  if (pending.revision > applied_gain_revision_.load(std::memory_order_acquire))
  {
    lqr_config_.coeffs = pending.coeffs;
    lqr_config_.q = pending.q;
    lqr_config_.r = pending.r;
    applied_gain_revision_.store(pending.revision, std::memory_order_release);
  }
  normal_feedback_ = lqr10::NormalFeedback{};
  resetObservation(ros::Time(0), ros::Duration(0));
  observation_history_ = lqr10::ObservationHistory{};
  velocity_kf_.reset();
  start_time_ = time;
  command_authorized_ = false;
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
  ROS_INFO("[lqr10] Started; waiting for a fresh chassis command");
}

void BipedalController::stopping(const ros::Time &time)
{
  velocity_kf_.reset();
  latchControlFault(lqr10::Reason::Terminated);
  balance_mode_ = BalanceMode::SIT_DOWN;
  setStateChange(false);
  setJointCommands(joint_handles_, {0, 0, {0., 0.}}, {0, 0, {0., 0.}});

  control_running_.store(false, std::memory_order_release);
  ROS_INFO("[balance] Controller Stop");
}

bool BipedalController::setupParams(ros::NodeHandle &controller_nh)
{
  const std::pair<const char *, double *> action_parameters[] = {
      {"recovery_leg_speed", &action_params_.recovery_leg_speed},
      {"jumpOverTime", &action_params_.jumpOverTime_},
      {"down5cmStairPitchThreshold", &action_params_.down5cmStairPitchThreshold},
      {"down5cmStairThetaThreshold", &action_params_.down5cmStairThetaThreshold},
      {"jump_up_force", &action_params_.jump_up_force},
      {"off_ground_force", &action_params_.off_ground_force}};
  for (const auto &p : action_parameters)
  {
    controller_nh.param(p.first, *p.second, *p.second);
    if (!std::isfinite(*p.second) || *p.second <= 0.)
      return false;
  }
  model_params_ = std::make_shared<ModelParams>();
  leg_threshold_params_ = std::make_shared<LegStateThresholdParams>();
  spring_params_ = std::make_shared<SpringParams>();
  chassis_geometry_params_ = std::make_shared<ChassisGeometryParams>();

  if (!setupModelParams(controller_nh) || !setupThresholdParams(controller_nh) || !setupSpringParams(controller_nh) ||
      !setupChassisGeometryParams(controller_nh))
    return false;
  return setupVelocityEstimator(controller_nh) && setupLQR(controller_nh);
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
  if (!std::isfinite(l1) || !std::isfinite(l2) || l1 <= 0. || l2 <= 0.)
  {
    ROS_ERROR("VMC l1/l2 must be finite and positive (namespace: %s)", controller_nh.getNamespace().c_str());
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

namespace lqr10
{
void Config::validateParameters() const
{
  if (!uff.allFinite() || !domain.allFinite() ||
      (domain.col(0).array() <= 0.).any() || (domain.col(1).array() <= domain.col(0).array()).any())
    throw std::invalid_argument("lqr10: invalid finite matrix/domain");
  if (!model.geometry_domain.allFinite() || model.geometry_domain[0] <= 0. ||
      model.geometry_domain[1] <= model.geometry_domain[0])
    throw std::invalid_argument("lqr10: invalid equivalent-leg domain");
  for (double value : {model.mb, model.Ib, model.Izz, model.mw, model.Iw, model.Rw, model.Rl, model.g})
    if (!std::isfinite(value) || value <= 0.)
      throw std::invalid_argument("lqr10: invalid physical model scalar");
  if (!std::isfinite(model.lc) || model.lc < 0. || !std::isfinite(model.phi_c))
    throw std::invalid_argument("lqr10: invalid body COM");
  for (const auto &leg : model.legs)
  {
    if (!std::isfinite(leg.mass) || leg.mass <= 0.)
      throw std::invalid_argument("lqr10: invalid equivalent-leg mass");
    for (const auto *curve : {&leg.offset, &leg.lb, &leg.lw, &leg.inertia})
      if (!curve->allFinite())
        throw std::invalid_argument("lqr10: nonfinite equivalent-leg coefficient");
  }
  const double positive[] = {wheel_radius,
                             length_reference_tau,
                             position_release_tau,
                             hold_capture_speed};
  for (double value : positive)
    if (!std::isfinite(value) || value <= 0.)
      throw std::invalid_argument("lqr10: missing/nonpositive numeric contract");
}
} // namespace lqr10

using namespace lqr10;
void BipedalController::latchControlFault(Reason reason)
{
  if (normal_feedback_.faulted)
    return; // Preserve the first cause until an explicit controller restart.
  normal_feedback_.faulted = true;
  normal_feedback_.active = false;
  normal_feedback_.reason = reason;
  normal_feedback_.output.setZero();
}

bool BipedalController::readVirtualLeg(int side, double pitch_rate, double &carrier_rate)
{
  auto &chassis = chassis_state_;
  auto &posture = leg_state_[side].posture;
  auto &vmc = *leg_state_[side].vmc;
  const double hip = joint_handles_[2 * side]->getPosition();
  const double hip_rate = joint_handles_[2 * side]->getVelocity();
  const double knee_rate = joint_handles_[2 * side + 1]->getVelocity();
  posture.theta = chassis.pitch;
  posture.rate = chassis.angular_vel.y;
  if (!std::isfinite(hip) || !std::isfinite(hip_rate) || !std::isfinite(knee_rate))
    return false;
  const auto &pos = vmc.getPos();
  double force_column[2], torque_column[2];
  vmc.leg_conv(1., 0., force_column);
  vmc.leg_conv(0., 1., torque_column);
  Eigen::Matrix2d jacobian;
  jacobian << force_column[0], force_column[1], torque_column[0], torque_column[1];
  if (!jacobian.allFinite() || !std::isfinite(pos.L0) || !std::isfinite(pos.theta) || pos.L0 <= 0.)
    return false;
  const Eigen::Vector2d velocity = jacobian * Eigen::Vector2d(hip_rate, knee_rate);
  chassis.length[side] = pos.L0;
  chassis.dlength[side] = velocity[0];
  chassis.x[THETA_L + 2 * side] = pos.theta + chassis.pitch;
  chassis.x[DTHETA_L + 2 * side] = velocity[1] + pitch_rate;
  // Original actions retain raw body-axis feedback when the full Euler observation is unavailable.
  posture.theta = chassis.x[THETA_L + 2 * side];
  posture.rate = velocity[1] + chassis.angular_vel.y;
  const double s = std::sin(pos.theta), c = std::cos(pos.theta), phi = hip + M_PI;
  const Eigen::Vector2d point(-pos.L0 * s, pos.L0 * c);
  const Eigen::Vector2d point_rate(-velocity[0] * s - pos.L0 * c * velocity[1],
                                   velocity[0] * c - pos.L0 * s * velocity[1]);
  const Eigen::Vector2d b(vmc.getL1() * std::cos(phi), vmc.getL1() * std::sin(phi));
  const Eigen::Vector2d b_rate(-b[1] * hip_rate, b[0] * hip_rate);
  const Eigen::Vector2d calf = point - b, calf_rate = point_rate - b_rate;
  const double calf_norm_sq = calf.squaredNorm();
  if (calf_norm_sq < 1e-12)
    return false;
  carrier_rate = (calf[0] * calf_rate[1] - calf[1] * calf_rate[0]) / calf_norm_sq;
  return std::isfinite(carrier_rate) && velocity.allFinite();
}

bool BipedalController::observe(double pitch_rate, const Eigen::Vector2d &carrier_rates)
{
  auto &out = chassis_state_;
  auto &history = observation_history_;
  const Eigen::Vector2d wheel_rates(joint_handles_[4]->getVelocity(), joint_handles_[5]->getVelocity());
  const Eigen::Vector3d rpy(out.roll, out.pitch, out.yaw);
  const Eigen::Vector3d omega_base(out.angular_vel.x, out.angular_vel.y, out.angular_vel.z);
  const double radius = lqr_config_.wheel_radius, time = out.time, dt = out.dt;
  out.valid = false;
  out.observation_reason = Reason::InvalidSnapshot;
  if (!std::isfinite(time) || !std::isfinite(dt) || dt <= 0.)
  {
    out.observation_reason = Reason::InvalidTime;
    return false;
  }
  if (!wheel_rates.allFinite() || !carrier_rates.allFinite() || !rpy.allFinite() || !omega_base.allFinite() ||
      !std::isfinite(pitch_rate) || !std::isfinite(radius) || radius <= 0.)
    return false;
  if (std::abs(rpy[1]) >= 1.2)
  {
    out.observation_reason = Reason::Posture;
    return false;
  }
  // ROS time labels and the host's computation step have different semantics:
  // rm_ecat currently supplies a nominal Worker period, not measured elapsed time.
  // Scheduling jitter (or a forward ROS-clock adjustment) must not invalidate sensors.
  // A non-advancing ROS label starts a new integration/filter segment only; keep
  // physical S/yaw and Normal/PID references, and still use the current observation.
  if (history.initialized && time <= history.last_time)
  {
    history.initialized = false;
    velocity_kf_.reset();
  }
  const double lateral_rate = omega_base[1] * std::sin(rpy[0]) + omega_base[2] * std::cos(rpy[0]);
  const double yaw_rate = lateral_rate / std::cos(rpy[1]);
  out.roll_rate = omega_base[0] + std::tan(rpy[1]) * lateral_rate;
  double v = 0.;
  for (int side = 0; side < 2; ++side)
    v += radius * (wheel_rates[side] + carrier_rates[side] + pitch_rate) / 2.;
  if (!history.initialized)
  {
    if (!history.ever_initialized)
    {
      history.s = 0.;
      history.yaw = rpy[2];
      history.ever_initialized = true;
    }
    else
      history.yaw += std::remainder(rpy[2] - history.wrapped_yaw, 2 * M_PI);
  }
  else
  {
    history.yaw += std::remainder(rpy[2] - history.wrapped_yaw, 2 * M_PI);
  }
  history.wrapped_yaw = rpy[2];
  out.x[S] = history.s;
  out.x[V] = v;
  out.x[YAW] = history.yaw;
  out.x[YAW_RATE] = yaw_rate;
  out.x[BODY_PITCH] = rpy[1];
  out.x[BODY_RATE] = pitch_rate;
  out.roll = rpy[0];
  out.valid = out.x.allFinite() && std::isfinite(out.roll_rate);
  if (out.valid)
    out.observation_reason = Reason::None;
  return out.valid;
}
// Called only by the ordinary Normal ground branch, before circle and K10.
bool BipedalController::updateGroundVelocity()
{
  auto &chassis = chassis_state_;
  if (!chassis.valid)
    return false;
  VelocityKalmanFilter::Input input;
  input.rotation_world_base = rotation_world_base_;
  input.yaw = chassis.yaw;
  input.yaw_rate = chassis.x[lqr10::YAW_RATE];
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
    const double angle_rate = chassis.x[lqr10::DTHETA_L + 2 * side] - chassis.x[lqr10::BODY_RATE];
    const double sine = std::sin(angle), cosine = std::cos(angle);
    input.wheel_midpoint_base += .5 * Eigen::Vector3d(-length * sine, 0., -length * cosine);
    input.wheel_midpoint_rate_base += .5 * Eigen::Vector3d(
        -length_rate * sine - length * cosine * angle_rate, 0.,
        -length_rate * cosine + length * sine * angle_rate);
  }
  input.kinematic_velocity = chassis.x[lqr10::V];
  input.dt = chassis.dt;
  VelocityKalmanFilter::Output output;
  if (!velocity_kf_.update(input, output))
  {
    chassis.valid = false;
    chassis.observation_reason = Reason::InvalidSnapshot;
    velocity_kf_.reset();
    return false;
  }
  chassis.x[lqr10::V] = output.velocity;
  integrateVelocity();
  return true;
}

void BipedalController::integrateVelocity()
{
  if (!chassis_state_.valid || velocity_integrated_)
    return;
  auto &history = observation_history_;
  auto &chassis = chassis_state_;
  if (history.initialized)
    history.s += (history.last_v + chassis.x[lqr10::V]) * chassis.dt / 2.;
  history.last_v = chassis.x[lqr10::V];
  history.last_time = chassis.time;
  history.initialized = true;
  chassis.x[lqr10::S] = history.s;
  chassis.x_vel = chassis.x[lqr10::V];
  velocity_integrated_ = true;
}

void BipedalController::finishObservation()
{
  // No ground update this frame: retain the original kinematic observation.
  if (!velocity_integrated_)
  {
    velocity_kf_.reset();
    integrateVelocity();
  }
  if (!chassis_state_.valid || normal_feedback_.faulted)
  {
    // A failed observation is a genuine missing integration sample. Do not bridge
    // it on recovery, or erase the position and unwrapped attitude already known.
    observation_history_.initialized = false;
    velocity_kf_.reset();
  }
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

bool BipedalController::setupVelocityEstimator(ros::NodeHandle &controller_nh)
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
    if (!controller_nh.getParam(parameter.first, *parameter.second))
    {
      ROS_ERROR("Missing velocity estimator parameter: %s", parameter.first);
      return false;
    }
  if (!velocity_kf_.configure(params))
  {
    ROS_ERROR("Invalid or missing velocity_estimator parameters");
    return false;
  }
  // Read only installation origins at init; runtime wheel geometry comes from the existing VMC.
  urdf::Model robot;
  if (!robot.initParam("robot_description"))
    return false;
  hip_midpoint_base_.setZero();
  for (const char *name : {"left_hip_joint", "right_hip_joint"})
  {
    const auto joint = robot.getJoint(name);
    if (!joint || joint->parent_link_name != "base_link")
    {
      ROS_ERROR("Velocity estimator requires hip installation origins relative to base_link");
      return false;
    }
    const auto &position = joint->parent_to_joint_origin_transform.position;
    hip_midpoint_base_ += .5 * Eigen::Vector3d(position.x, position.y, position.z);
  }
  return true;
}

bool BipedalController::loadLqrParams(ros::NodeHandle &controller_nh, lqr10::Config &config)
{
  auto &model = config.model;
  PhysicalLeg common_leg;
  const std::pair<const char *, double *> parameters[] = {
      {"lqr_model/gravity", &model.g},
      {"lqr_model/yaw_inertia", &model.Izz},
      {"lqr_model/body/mass", &model.mb},
      {"lqr_model/body/pitch_inertia", &model.Ib},
      {"lqr_model/body/com_distance", &model.lc},
      {"lqr_model/body/com_angle", &model.phi_c},
      {"lqr_model/wheel/mass", &model.mw},
      {"lqr_model/wheel/spin_inertia", &model.Iw},
      {"lqr_model/leg/mass", &common_leg.mass},
      {"normal_reference/length_tau", &config.length_reference_tau},
      {"normal_reference/position_release_tau", &config.position_release_tau},
      {"normal_reference/hold_capture_speed", &config.hold_capture_speed}};
  for (const auto &entry : parameters)
    if (!controller_nh.getParam(entry.first, *entry.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", entry.first, controller_nh.getNamespace().c_str());
      return false;
    }

  const std::pair<const char *, Curve8 *> curves[] = {
      {"lqr_model/leg/angle_offset_coefficients", &common_leg.offset},
      {"lqr_model/leg/lb_coefficients", &common_leg.lb},
      {"lqr_model/leg/lw_coefficients", &common_leg.lw},
      {"lqr_model/leg/inertia_coefficients", &common_leg.inertia}};
  for (const auto &entry : curves)
    if (!getVectorParam(controller_nh, entry.first, *entry.second))
      return false;

  Eigen::Vector2d length_domain;
  if (!getVectorParam(controller_nh, "q", config.q) || !getVectorParam(controller_nh, "r", config.r) ||
      !getVectorParam(controller_nh, "lqr10/length_domain", length_domain))
    return false;
  config.domain.row(0) = length_domain.transpose();
  config.domain.row(1) = length_domain.transpose();
  model.Rw = wheel_radius_;
  model.Rl = 0.5 * chassis_geometry_params_->wheel_track;
  model.geometry_domain = length_domain;
  model.legs[0] = common_leg;
  model.legs[1] = common_leg;
  config.uff = staticFeedforward(model);
  config.wheel_radius = wheel_radius_;
  return true;
}

bool BipedalController::setupLQR(ros::NodeHandle &controller_nh)
{
  try
  {
    // Read and assemble the single source configuration, then validate it once.
    lqr10::Config config;
    if (!loadLqrParams(controller_nh, config))
      return false;
    if (!config.q.allFinite() || !config.r.allFinite() ||
        (config.q.array() <= 0.).any() || (config.r.array() <= 0.).any())
      throw std::invalid_argument("lqr10 Q/R must be finite and strictly positive");
    config.validateParameters();
    if ((Eigen::Vector2d::Constant(default_leg_length_).array() < config.domain.col(0).array()).any() ||
        (Eigen::Vector2d::Constant(default_leg_length_).array() > config.domain.col(1).array()).any())
      throw std::invalid_argument("default leg length outside validated gain domain");

    // Generate the complete gain table on the non-realtime side.
    design_config_ = config;
    lqr10::DesignReport report;
    const auto generated = polyfit(config.q, config.r, report);
    if (!generated.coeffs.allFinite())
      throw std::runtime_error("lqr10: nonfinite generated gain coefficients");
    config.coeffs = generated.coeffs;

    // Publish the complete initial configuration only after successful generation.
    lqr_config_ = config;
    ROS_INFO("[lqr10] generated from controller source: %u CARE, %u validation points, fit=%g trim=%g pole=%g",
             report.nodes, report.validation_points, report.fit_error, report.equilibrium_error, report.max_real);
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR("[lqr10] Initialization rejected: %s", ex.what());
    return false;
  }
  return true;
}

lqr10::GainUpdate BipedalController::polyfit(const lqr10::State10 &q, const lqr10::Input4 &r,
                                             lqr10::DesignReport &report) const
{
  return lqr10::designTable(design_config_, q, r, report);
}
void BipedalController::configureLqrInterface(ros::NodeHandle &nh)
{
  accepted_weights_.Q_s = lqr_config_.q[0];
  accepted_weights_.Q_v = lqr_config_.q[1];
  accepted_weights_.Q_yaw = lqr_config_.q[2];
  accepted_weights_.Q_yaw_rate = lqr_config_.q[3];
  accepted_weights_.Q_theta_L = lqr_config_.q[4];
  accepted_weights_.Q_dtheta_L = lqr_config_.q[5];
  accepted_weights_.Q_theta_R = lqr_config_.q[6];
  accepted_weights_.Q_dtheta_R = lqr_config_.q[7];
  accepted_weights_.Q_phi = lqr_config_.q[8];
  accepted_weights_.Q_phi_dot = lqr_config_.q[9];
  accepted_weights_.R_T_L = lqr_config_.r[0];
  accepted_weights_.R_T_R = lqr_config_.r[1];
  accepted_weights_.R_Tp_L = lqr_config_.r[2];
  accepted_weights_.R_Tp_R = lqr_config_.r[3];
  lqr_server_.reset(new dynamic_reconfigure::Server<LQRWeightConfig>(gain_server_mutex_, nh));
  lqr_server_->updateConfig(accepted_weights_);
  lqr_server_->setCallback(boost::bind(&BipedalController::reconfigCB, this, _1, _2));
  gain_status_timer_ = nh.createTimer(ros::Duration(.1), &BipedalController::publishGainStatus, this);
}
void BipedalController::reconfigCB(LQRWeightConfig &request, uint32_t)
{
  std::lock_guard<std::mutex> lock(gain_callback_mutex_);
  if (first_reconfigure_)
  {
    first_reconfigure_ = false;
    request = accepted_weights_;
    return;
  }
  const auto revision = ++requested_gain_revision_;
  lqr10::State10 q;
  lqr10::Input4 r;
  q << request.Q_s, request.Q_v, request.Q_yaw, request.Q_yaw_rate, request.Q_theta_L, request.Q_dtheta_L,
      request.Q_theta_R, request.Q_dtheta_R, request.Q_phi, request.Q_phi_dot;
  r << request.R_T_L, request.R_T_R, request.R_Tp_L, request.R_Tp_R;
  try
  {
    if (control_running_.load(std::memory_order_acquire))
      throw std::invalid_argument("rejected: controller running; stop before requesting Q/R");
    lqr10::DesignReport report;
    auto update = polyfit(q, r, report);
    update.revision = revision;
    gain_update_buffer_.writeFromNonRT(update);
    staged_gain_revision_ = revision;
    accepted_weights_ = request;
    gain_status_ = "validated; pending next controller start (no active table switch)";
    ROS_INFO("[lqr10] requested=%lu validated: CARE=%g pole=%g", revision, report.care_residual, report.max_real);
  }
  catch (const std::exception &error)
  {
    gain_status_ = std::string("rejected; previous valid snapshot retained: ") + error.what();
    ROS_ERROR("[lqr10] Q/R request %lu: %s", revision, gain_status_.c_str());
  }
  accepted_weights_.requested_version = static_cast<int>(requested_gain_revision_);
  accepted_weights_.applied_version = static_cast<int>(applied_gain_revision_.load(std::memory_order_acquire));
  accepted_weights_.update_status = gain_status_;
  request = accepted_weights_;
}
void BipedalController::publishGainStatus(const ros::TimerEvent &)
{
  // Match dynamic_reconfigure callback lock order, so a status timer cannot publish stale weights.
  boost::recursive_mutex::scoped_lock server_lock(gain_server_mutex_);
  std::lock_guard<std::mutex> lock(gain_callback_mutex_);
  const auto applied = applied_gain_revision_.load(std::memory_order_acquire);
  accepted_weights_.applied_version = static_cast<int>(applied);
  if (staged_gain_revision_ != 0 && applied == staged_gain_revision_ && gain_status_.find("validated; pending") == 0)
    gain_status_ = "applied at controller start; normal feedback uses this complete snapshot";
  accepted_weights_.update_status = gain_status_;
  lqr_server_->updateConfig(accepted_weights_);
}

} // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BipedalController, controller_interface::ControllerBase)
