//
// Created by wk on 2026/4/11.
//
#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/vmc/VMC.h"

namespace rm_chassis_controllers
{
class BipedalControllerInterface
{
public:
  virtual ~BipedalControllerInterface() = default;
  virtual const lqr10::Config &getLqrConfig() const = 0;
  virtual lqr10::NormalFeedback &getNormalFeedback() = 0;
  virtual void latchControlFault(lqr10::Reason reason) = 0;

  virtual const ControlParams &getActionParams() const = 0;
  virtual int getBalanceMode() const = 0;
  virtual bool getStateChange() const = 0;
  virtual void setStateChange(bool state) = 0;
  virtual int getBaseState() const = 0;
  virtual bool getOverturn() const = 0;
  virtual void setMode(int mode) = 0;
  virtual void clearRecoveryFlag() = 0;
  virtual bool getCompleteStand() const = 0;

  virtual const std::shared_ptr<ModelParams> &getModelParams() const = 0;
  virtual const std::shared_ptr<LegStateThresholdParams> &getLegThresholdParams() const = 0;
  virtual const std::shared_ptr<ChassisGeometryParams> &getChassisGeometryParams() const = 0;
  virtual double getLegCmd() const = 0;
  virtual double getJumpCmd() const = 0;
  virtual double getDefaultLegLength() const = 0;
  virtual geometry_msgs::Vector3 getVelCmd() = 0;
  virtual const ChassisState &getChassisState() = 0;
  virtual LegState &getLegState(Side side) = 0;
  virtual bool getDown5cmStairFlag() const = 0;
  virtual void setDown5cmStairFlag(bool flag) = 0;
  virtual void setCompleteStand(bool state) = 0;
  virtual void setJumpCmd(bool cmd) = 0;
  virtual double f_spring_force(double L0) = 0;
  virtual void pubState() = 0;
  virtual void pubLegLenStatus(const bool &upstair_flag) = 0;
  virtual bool getRecoveryLegSpdTurnback() const = 0;
  virtual void setRecoveryLegSpdTurnback(bool recovery_leg_spd_turnback) = 0;
};

} // namespace rm_chassis_controllers
