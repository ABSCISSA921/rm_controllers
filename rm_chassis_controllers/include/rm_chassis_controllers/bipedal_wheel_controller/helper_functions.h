//
// Created by guanlin on 25-8-27.
//

#pragma once

#include <angles/angles.h>
#include <cstddef>
#include <memory>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Quaternion.h>
#include <hardware_interface/joint_command_interface.h>

#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
static inline double get_LM(const double &l) { return 0.218f * l + 0.075f; };

static inline double get_theta_leg_offset(const double &l) { return M_PI_4 / 2; }

/**
 * Set joint commands to the joint handles
 * @param joints
 * @param left_cmd
 * @param right_cmd
 * @param wheel_left
 * @param wheel_right
 */
inline void setJointCommands(std::vector<hardware_interface::JointHandle *> &joints, const LegCommand &left_cmd,
                             const LegCommand &right_cmd, double wheel_left = 0., double wheel_right = 0.)
{
  if (joints.size() != 6)
    throw std::runtime_error("Joint handle vector size must be 6!");

  joints[0]->setCommand(left_cmd.input[0]);
  joints[1]->setCommand(left_cmd.input[1]);
  joints[2]->setCommand(right_cmd.input[0]);
  joints[3]->setCommand(right_cmd.input[1]);
  joints[4]->setCommand(wheel_left);
  joints[5]->setCommand(wheel_right);
}

/**
 * Convert quaternion to roll, pitch, yaw
 * @param q
 * @param roll
 * @param pitch
 * @param yaw
 */
inline void quatToRPY(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw)
{
  double as = std::max(-.99999, std::min(-2. * (q.x * q.z - q.w * q.y), .99999));
  yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  pitch = std::asin(as);
  roll = std::atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

inline void clamp(double &val, const double &minVal, const double &maxVal)
{
  if (val < minVal)
    val = minVal;
  if (val > maxVal)
    val = maxVal;
}

} // namespace rm_chassis_controllers
