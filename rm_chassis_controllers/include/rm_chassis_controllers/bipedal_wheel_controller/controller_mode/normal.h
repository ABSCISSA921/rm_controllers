#pragma once
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include "mode_base.h"
#include "bipedal_wheel_controller/normal_reference.h"

namespace rm_chassis_controllers
{
class Normal : public ModeBase
{
public:
  Normal(BipedalControllerInterface *, const std::vector<hardware_interface::JointHandle *> &,
         const std::vector<control_toolbox::Pid *> &, control_toolbox::Pid *yaw, control_toolbox::Pid *theta_diff,
         control_toolbox::Pid *roll, control_toolbox::Pid *wheel_diff,
         const std::vector<control_toolbox::Pid *> &thetas = {});
  Normal(BipedalControllerInterface *c, const std::vector<hardware_interface::JointHandle *> &j,
         const std::vector<control_toolbox::Pid *> &l, control_toolbox::Pid *roll)
      : Normal(c, j, l, nullptr, nullptr, roll, nullptr)
  {
  }
  void execute(const ros::Time &, const ros::Duration &) override;
  const char *name() const override { return "NORMAL"; }

private:
  bool reject(lqr10::Reason reason);
  bool computeFeedback(double velocity, double yaw_rate);
  void requestMode(int mode);
  void executeJump(const ros::Time &, const ros::Duration &);
  std::vector<control_toolbox::Pid *> thetas_;
  control_toolbox::Pid *yaw_, *theta_diff_, *wheel_diff_;
  JumpPhase jump_phase_{IDLE};
  double last_jump_time_{-1e9}, jump_dwell_{0.};
  std::vector<hardware_interface::JointHandle *> joints_;
  std::vector<control_toolbox::Pid *> legs_;
  control_toolbox::Pid *roll_;
  lqr10::LengthReference length_reference_;
  lqr10::LongitudinalReference longitudinal_reference_;
};
} // namespace rm_chassis_controllers
