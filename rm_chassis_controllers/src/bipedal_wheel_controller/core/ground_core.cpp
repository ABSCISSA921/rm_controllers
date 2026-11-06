#include "bipedal_wheel_controller/core/ground_core.h"
#include "bipedal_wheel_controller/dynamics/model10.h"
#include "bipedal_wheel_controller/helper_functions.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_chassis_controllers
{
namespace ground_core
{
bool transformImu(const GroundSample& sample, const GroundInstallation& installation, ChassisState& state,
                  Eigen::Matrix3d& rotation_world_base)
{
  const auto& measured = sample.orientation_world_imu;
  const auto& mounting = installation.rotation_base_imu;
  const tf2::Quaternion world_imu(measured.x(), measured.y(), measured.z(), measured.w());
  const tf2::Quaternion base_imu(mounting.x(), mounting.y(), mounting.z(), mounting.w());
  // Match the former TF transform product, including its quaternion convention.
  const tf2::Transform world_base = tf2::Transform(world_imu) * tf2::Transform(base_imu).inverse();
  const tf2::Matrix3x3 base_imu_basis(base_imu);
  const tf2::Vector3 angular =
      base_imu_basis * tf2::Vector3(sample.gyro_imu.x(), sample.gyro_imu.y(), sample.gyro_imu.z());
  const tf2::Vector3 acceleration =
      base_imu_basis *
      tf2::Vector3(sample.specific_force_imu.x(), sample.specific_force_imu.y(), sample.specific_force_imu.z());
  geometry_msgs::Quaternion orientation = tf2::toMsg(world_base.getRotation());
  quatToRPY(orientation, state.roll, state.pitch, state.yaw);
  const tf2::Vector3 z_world = tf2::quatRotate(world_base.getRotation(), tf2::Vector3(0., 0., 1.));
  const tf2::Vector3 gravity_body = tf2::quatRotate(world_base.getRotation().inverse(), tf2::Vector3(0., 0., 1.));
  state.recovery_pitch = std::atan2(-gravity_body.x(), gravity_body.z());
  state.recovery_roll = std::atan2(gravity_body.y(), std::hypot(gravity_body.x(), gravity_body.z()));
  state.upright_z = z_world.z();
  state.angular_vel.x = angular.x();
  state.angular_vel.y = angular.y();
  state.angular_vel.z = angular.z();
  state.linear_acc.x = acceleration.x();
  state.linear_acc.y = acceleration.y();
  state.linear_acc.z = acceleration.z();
  const auto& rotation = world_base.getBasis();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      rotation_world_base(row, col) = rotation[row][col];
  return (std::abs(state.pitch) > .65 || std::abs(state.roll) > .8) && z_world.z() < 0.;
}

void updateVirtualLegs(LegState legs[2], const Joint6& position, const Joint6& velocity)
{
  for (int side = 0; side < 2; ++side)
  {
    const double hip = position[2 * side] + M_PI, knee = position[2 * side + 1];
    auto& vmc = *legs[side].vmc;
    vmc.calc_jacobian(hip, knee);
    vmc.leg_pos(hip, knee);
    vmc.leg_spd(velocity[2 * side], velocity[2 * side + 1]);
  }
}

double readVirtualLeg(ChassisState& state, LegState& leg, int side, double hip, double hip_rate, double knee_rate,
                      double pitch_rate)
{
  auto& vmc = *leg.vmc;
  const auto& pos = vmc.getPos();
  double force_column[2], torque_column[2];
  vmc.leg_conv(1., 0., force_column);
  vmc.leg_conv(0., 1., torque_column);
  Eigen::Matrix2d jacobian;
  jacobian << force_column[0], force_column[1], torque_column[0], torque_column[1];
  const Eigen::Vector2d velocity = jacobian * Eigen::Vector2d(hip_rate, knee_rate);
  state.length[side] = pos.L0;
  state.dlength[side] = velocity[0];
  state.x[THETA_L + 2 * side] = pos.theta + state.pitch;
  state.x[DTHETA_L + 2 * side] = velocity[1] + pitch_rate;
  const double s = std::sin(pos.theta), c = std::cos(pos.theta), phi = hip + M_PI;
  const Eigen::Vector2d point(-pos.L0 * s, pos.L0 * c);
  const Eigen::Vector2d point_rate(-velocity[0] * s - pos.L0 * c * velocity[1],
                                   velocity[0] * c - pos.L0 * s * velocity[1]);
  const Eigen::Vector2d b(vmc.getL1() * std::cos(phi), vmc.getL1() * std::sin(phi));
  const Eigen::Vector2d b_rate(-b[1] * hip_rate, b[0] * hip_rate);
  const Eigen::Vector2d calf = point - b, calf_rate = point_rate - b_rate;
  return (calf[0] * calf_rate[1] - calf[1] * calf_rate[0]) / calf.squaredNorm();
}

void observe(ChassisState& out, double pitch_rate, const Eigen::Vector2d& carrier_rates,
             const Eigen::Vector2d& wheel_rates, double wheel_radius)
{
  const Eigen::Vector3d rpy(out.roll, out.pitch, out.yaw);
  const Eigen::Vector3d omega_base(out.angular_vel.x, out.angular_vel.y, out.angular_vel.z);
  const double lateral_rate = omega_base[1] * std::sin(rpy[0]) + omega_base[2] * std::cos(rpy[0]);
  const double yaw_rate = lateral_rate / std::cos(rpy[1]);
  out.roll_rate = omega_base[0] + std::tan(rpy[1]) * lateral_rate;
  double v = 0.;
  for (int side = 0; side < 2; ++side)
    v += wheel_radius * (wheel_rates[side] + carrier_rates[side] + pitch_rate) / 2.;
  out.x[V] = v;
  out.x[YAW] += std::remainder(rpy[2] - out.x[YAW], 2 * M_PI);
  out.x[YAW_RATE] = yaw_rate;
  out.x[BODY_PITCH] = rpy[1];
  out.x[BODY_RATE] = pitch_rate;
  out.roll = rpy[0];
}

void updateGroundVelocity(ChassisState& chassis, LegState legs[2], VelocityKalmanFilter& filter,
                          const Eigen::Matrix3d& rotation_world_base, const Eigen::Vector3d& imu_position_base,
                          const Eigen::Vector3d& hip_midpoint_base, double wheel_radius)
{
  VelocityKalmanFilter::Input input;
  input.rotation_world_base = rotation_world_base;
  input.yaw = chassis.yaw;
  input.yaw_rate = chassis.x[YAW_RATE];
  input.specific_force_base << chassis.linear_acc.x, chassis.linear_acc.y, chassis.linear_acc.z;
  input.angular_velocity_base << chassis.angular_vel.x, chassis.angular_vel.y, chassis.angular_vel.z;
  input.imu_position_base = imu_position_base;
  input.wheel_midpoint_base = hip_midpoint_base;
  input.wheel_midpoint_rate_base.setZero();
  for (int side = 0; side < 2; ++side)
  {
    const double length = chassis.length[side], length_rate = chassis.dlength[side];
    const double angle = legs[side].vmc->getPos().theta;
    const double angle_rate = chassis.x[DTHETA_L + 2 * side] - chassis.x[BODY_RATE];
    const double sine = std::sin(angle), cosine = std::cos(angle);
    input.wheel_midpoint_base += .5 * Eigen::Vector3d(-length * sine, 0., -length * cosine);
    input.wheel_midpoint_rate_base += .5 * Eigen::Vector3d(-length_rate * sine - length * cosine * angle_rate, 0.,
                                                           -length_rate * cosine + length * sine * angle_rate);
  }
  input.kinematic_velocity = chassis.x[V];
  const auto geometry = observeBaseTask(chassis, rotation_world_base, hip_midpoint_base, wheel_radius);
  input.wheel_lateral_velocity =
      Eigen::Vector3d(-std::sin(chassis.yaw), std::cos(chassis.yaw), 0.).dot(geometry.wheel_velocity_world);
  input.dt = chassis.dt;
  VelocityKalmanFilter::Output output;
  filter.update(input, output);
  chassis.x[V] = output.velocity;
  updateBaseObservation(chassis, rotation_world_base, hip_midpoint_base, wheel_radius);
  const Eigen::Vector3d forward(std::cos(chassis.yaw), std::sin(chassis.yaw), 0.);
  chassis.base_velocity_world += forward * (output.base_velocity - forward.dot(chassis.base_velocity_world));
  chassis.x_vel = output.base_velocity;
}

void updateBaseObservation(ChassisState& state, const Eigen::Matrix3d& rotation_world_base,
                           const Eigen::Vector3d& hip_midpoint_base, double wheel_radius)
{
  const auto base = observeBaseTask(state, rotation_world_base, hip_midpoint_base, wheel_radius);
  state.base_velocity_world = base.velocity_world;
  state.base_height = base.height;
  state.base_height_rate = base.height_rate;
  state.x_vel = base.forward_velocity;
}

void integrateVelocity(ChassisState& state)
{
  state.x[S] += state.x[V] * state.dt;
  state.base_position_world += state.base_velocity_world.head<2>() * state.dt;
}

void updateTaskReference(const Config& config, ChassisState& chassis, NormalFeedback& feedback,
                         LengthReference& length_reference, const GroundCommand& command)
{
  if (!feedback.reference_valid)
  {
    feedback.reference.setZero();
    feedback.reference[YAW] = chassis.x[YAW];
    feedback.base_position_reference = chassis.base_position_world;
    length_reference.reset(chassis.length.mean());
    feedback.reference_valid = true;
  }
  length_reference.update(command.leg_length, chassis.dt, config.length_reference_tau);
  const State10 equilibrium = evaluateEquilibrium(config, Eigen::Vector2d::Constant(length_reference.position));
  for (int i : { THETA_L, THETA_R, BODY_PITCH })
    feedback.reference[i] = equilibrium[i];
  const double epsilon = 1e-5;
  const State10 before = evaluateEquilibrium(config, Eigen::Vector2d::Constant(length_reference.position - epsilon));
  const State10 after = evaluateEquilibrium(config, Eigen::Vector2d::Constant(length_reference.position + epsilon));
  for (int i : { THETA_L, THETA_R, BODY_PITCH })
    feedback.reference[i + 1] = (after[i] - before[i]) * length_reference.velocity / (2. * epsilon);
  const double velocity_reference = command.complete_stand ? command.forward_velocity : 0.;
  feedback.reference[YAW_RATE] = command.yaw_rate;
  feedback.reference[YAW] += command.yaw_rate * chassis.dt;
  const Eigen::Vector3d forward(std::cos(chassis.yaw), std::sin(chassis.yaw), 0.);
  const Eigen::Vector3d reference_forward(std::cos(feedback.reference[YAW]), std::sin(feedback.reference[YAW]), 0.);
  feedback.base_velocity_reference = reference_forward * velocity_reference;
  feedback.base_position_reference += feedback.base_velocity_reference.head<2>() * chassis.dt;

  const auto rotation = (Eigen::AngleAxisd(chassis.yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(chassis.pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(chassis.roll, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();
  const auto actual_base = observeBaseTask(chassis, rotation, config.hip_midpoint_base, config.wheel_radius);
  const double actual_wheel_offset = forward.dot(rotation * actual_base.wheel_midpoint_base);
  const double pitch_reference = feedback.reference[BODY_PITCH];
  double reference_wheel_offset = std::cos(pitch_reference) * config.hip_midpoint_base.x() +
                                  std::sin(pitch_reference) * config.hip_midpoint_base.z();
  double reference_wheel_rate = (-std::sin(pitch_reference) * config.hip_midpoint_base.x() +
                                 std::cos(pitch_reference) * config.hip_midpoint_base.z()) *
                                feedback.reference[BODY_RATE];
  feedback.base_height_reference = config.wheel_radius + std::sin(pitch_reference) * config.hip_midpoint_base.x() -
                                   std::cos(pitch_reference) * config.hip_midpoint_base.z();
  feedback.base_height_rate_reference = (std::cos(pitch_reference) * config.hip_midpoint_base.x() +
                                         std::sin(pitch_reference) * config.hip_midpoint_base.z()) *
                                        feedback.reference[BODY_RATE];
  for (int side = 0; side < 2; ++side)
  {
    const double angle = feedback.reference[THETA_L + 2 * side];
    const double rate = feedback.reference[DTHETA_L + 2 * side];
    reference_wheel_offset -= .5 * length_reference.position * std::sin(angle);
    reference_wheel_rate -=
        .5 * (length_reference.velocity * std::sin(angle) + length_reference.position * std::cos(angle) * rate);
    feedback.base_height_reference += .5 * length_reference.position * std::cos(angle);
    feedback.base_height_rate_reference +=
        .5 * (length_reference.velocity * std::cos(angle) - length_reference.position * std::sin(angle) * rate);
  }
  const double base_position_error =
      forward.head<2>().dot(chassis.base_position_world - feedback.base_position_reference);
  // Lift the world-base task into the retained wheel-coordinate model.  The
  // observed wheel integral stays continuous, including while spinning in RAW.
  const double wheel_position_error = base_position_error + actual_wheel_offset - reference_wheel_offset;
  feedback.reference[S] = chassis.x[S] - wheel_position_error;
  feedback.reference[V] = forward.dot(feedback.base_velocity_reference) + reference_wheel_rate;
}

void computeFeedback(const Config& config, ChassisState& chassis, NormalFeedback& feedback,
                     LengthReference& length_reference, const GroundCommand& command)
{
  updateTaskReference(config, chassis, feedback, length_reference, command);
  Gain10 gain;
  Input4 feedforward;
  evaluate(config, chassis.length, gain, feedforward);
  feedback.output = feedforward - gain * (chassis.x - feedback.reference);
  feedback.active = true;
}

Joint6 computeGroundTorques(const Config& config, const ModelParams& model, double wheel_track, ChassisState& chassis,
                            LegState legs[2], NormalFeedback& feedback, LengthReference& length_reference,
                            const std::array<control_toolbox::Pid*, 2>& leg_pid, control_toolbox::Pid& roll_pid,
                            const GroundCommand& command)
{
  const double circle = chassis.x_vel * chassis.x[YAW_RATE];
  computeFeedback(config, chassis, feedback, length_reference, command);
  const ros::Duration period(chassis.dt);
  const double roll_force = roll_pid.computeCommand(-chassis.roll, -chassis.roll_rate, period);
  const double height_error = feedback.base_height_reference - chassis.base_height;
  const double height_rate_error = feedback.base_height_rate_reference - chassis.base_height_rate;
  const auto rotation = (Eigen::AngleAxisd(chassis.pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(chassis.roll, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();
  for (int side = 0; side < 2; ++side)
  {
    const auto& pos = legs[side].vmc->getPos();
    double pid = leg_pid[side]->computeCommand(height_error, height_rate_error, period);
    clamp(pid, -150., 150.);
    const double turning = model.M * circle * pos.L0 / wheel_track;
    const double turning_force = side == LEFT ? -turning : turning;
    const double roll_component = side == LEFT ? roll_force : -roll_force;
    const double spring_force = -springForce(pos.L0);
    const Eigen::Vector3d upward(std::sin(pos.theta), 0., std::cos(pos.theta));
    const double vertical_projection = (rotation * upward).z();
    feedback.axial_force[side] =
        (pid + turning_force + model.f_gravity + roll_component) / vertical_projection + spring_force;
  }
  double left_torque[2], right_torque[2];
  legs[LEFT].vmc->leg_conv(feedback.axial_force[LEFT], feedback.output[2], left_torque);
  legs[RIGHT].vmc->leg_conv(feedback.axial_force[RIGHT], feedback.output[3], right_torque);
  Joint6 output;
  output << left_torque[0], left_torque[1], right_torque[0], right_torque[1], feedback.output[0], feedback.output[1];
  return output;
}

double springForce(double length)
{
  // Shared spring0 comparison baseline: keep the original fit here for manual restoration.
  // return ((2094.45f * length - 3091.28f) * length + 1408.375f) * length - 80.91f;
  (void)length;
  return 0.;
}
}  // namespace ground_core

void GroundControllerCore::configure(const GroundCoreParameters& parameters)
{
  parameters_ = parameters;
  parameters_.lqr.hip_midpoint_base = parameters.installation.hip_midpoint_base;
  state_ = ChassisState{};
  feedback_ = NormalFeedback{};
  velocity_filter_.configure(parameters.velocity_estimator);
  for (int side = 0; side < 2; ++side)
  {
    legs_[side] = LegState{};
    legs_[side].vmc = std::make_shared<VMC>(parameters.l1, parameters.l2);
    const auto& gains = parameters.leg_pid[side];
    leg_pid_[side].initPid(gains.p, gains.i, gains.d, gains.i_max, gains.i_min, gains.antiwindup);
  }
  const auto& gains = parameters.roll_pid;
  roll_pid_.initPid(gains.p, gains.i, gains.d, gains.i_max, gains.i_min, gains.antiwindup);
  rotation_world_base_.setIdentity();
  enterNormal();
}

void GroundControllerCore::enterNormal()
{
  for (auto& pid : leg_pid_)
    pid.reset();
  roll_pid_.reset();
  feedback_.active = false;
  feedback_.reference_valid = false;
}

void GroundControllerCore::updateObservation(const GroundSample& sample)
{
  ground_core::transformImu(sample, parameters_.installation, state_, rotation_world_base_);
  ground_core::updateVirtualLegs(legs_, sample.position, sample.velocity);
  state_.dt = sample.dt;
  const double pitch_rate = state_.angular_vel.y * std::cos(state_.roll) - state_.angular_vel.z * std::sin(state_.roll);
  Eigen::Vector2d carrier_rates;
  for (int side = 0; side < 2; ++side)
  {
    carrier_rates[side] =
        ground_core::readVirtualLeg(state_, legs_[side], side, sample.position[2 * side], sample.velocity[2 * side],
                                    sample.velocity[2 * side + 1], pitch_rate);
    legs_[side].posture.theta = state_.x[THETA_L + 2 * side];
    legs_[side].posture.rate = state_.x[DTHETA_L + 2 * side];
  }
  ground_core::observe(state_, pitch_rate, carrier_rates, sample.velocity.tail<2>(), parameters_.lqr.wheel_radius);
  ground_core::updateGroundVelocity(state_, legs_, velocity_filter_, rotation_world_base_,
                                    parameters_.installation.imu_position_base,
                                    parameters_.installation.hip_midpoint_base, parameters_.lqr.wheel_radius);
  ground_core::integrateVelocity(state_);
}

void GroundControllerCore::updateTaskReference(const GroundCommand& command)
{
  ground_core::updateTaskReference(parameters_.lqr, state_, feedback_, length_reference_, command);
}

Joint6 GroundControllerCore::step(const GroundSample& sample, const GroundCommand& command)
{
  updateObservation(sample);
  return ground_core::computeGroundTorques(parameters_.lqr, parameters_.model, parameters_.wheel_track, state_, legs_,
                                           feedback_, length_reference_, { { &leg_pid_[LEFT], &leg_pid_[RIGHT] } },
                                           roll_pid_, command);
}
}  // namespace rm_chassis_controllers
