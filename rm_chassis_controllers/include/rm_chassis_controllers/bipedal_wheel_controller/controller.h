//
// Created by guanlin on 25-8-28.
//

#pragma once

#include <rm_msgs/DebugData.h>
#include <rm_msgs/LegCmd.h>
#include <rm_msgs/LeggedChassisMode.h>
#include <rm_msgs/LeggedChassisStatus.h>
#include <rm_msgs/LeggedLQRStatus.h>
#include <rm_msgs/LeggedUpstairStatus.h>
#include <rm_common/filters/lp_filter.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <array>
#include <atomic>
#include <mutex>
#include <dynamic_reconfigure/server.h>
#include <rm_chassis_controllers/LQRWeightConfig.h>
#include "bipedal_wheel_controller/dynamics/model10.h"
#include "bipedal_wheel_controller/estimation/velocity_kf.h"
#include "rm_chassis_controllers/chassis_base.h"

#include "bipedal_wheel_controller/helper_functions.h"
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/controller_mode/mode_manager.h"
#include "bipedal_wheel_controller/vmc/VMC.h"
#include "bipedal_wheel_controller/controller_interface.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;

class BipedalController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>,
                          public BipedalControllerInterface
{
public:
  BipedalController() = default;
  const lqr10::Config &getLqrConfig() const override { return lqr_config_; }
  lqr10::NormalFeedback &getNormalFeedback() override { return normal_feedback_; }
  void latchControlFault(lqr10::Reason reason) override;
  const ControlParams &getActionParams() const override { return action_params_; }
  int getBalanceMode() const override { return balance_mode_; }
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void moveJoint(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time &time) override;
  // clang-format off
  // BipedalControllerInterface implementations
  bool getOverturn() const override { return overturn_; }
  bool getStateChange() const override { return balance_state_changed_; }
  bool getCompleteStand() const override { return complete_stand_; }
  const std::shared_ptr<ModelParams>& getModelParams() const override { return model_params_; }
  const std::shared_ptr<LegStateThresholdParams>& getLegThresholdParams() const override { return leg_threshold_params_; }
  const std::shared_ptr<ChassisGeometryParams>& getChassisGeometryParams() const override { return chassis_geometry_params_; }
  double getLegCmd() const override { return legCmd_; }
  double getJumpCmd() const override { return jumpCmd_; }
  int getBaseState() const override { return state_; }
  inline double getDefaultLegLength() const override { return default_leg_length_;}
  geometry_msgs::Vector3 getVelCmd() override { return vel_cmd_; }
  inline const ChassisState& getChassisState() override { return chassis_state_; };
  inline LegState& getLegState(Side side) override { return leg_state_[side]; };
  inline bool getDown5cmStairFlag() const override { return down_5cm_stair_flag_.load(std::memory_order_acquire); }
  inline void setDown5cmStairFlag(bool flag) override { down_5cm_stair_flag_.store(flag, std::memory_order_release); }
  inline void setStateChange(bool state) override { balance_state_changed_ = state; }
  inline void setCompleteStand(bool state) override { complete_stand_ = state; }
  void setJumpCmd(bool cmd) override { jumpCmd_ = cmd; }
  void setMode(int mode) override { balance_mode_ = mode; }
  inline void clearRecoveryFlag() override { overturn_ = false; }
  double f_spring_force(double L0) override;
  void pubState() override;
  void pubLegLenStatus(const bool& upstair_flag) override;
  void setRecoveryLegSpdTurnback(bool recovery_leg_spd_turnback) override { recovery_leg_spd_turnback_ = recovery_leg_spd_turnback; }
  bool getRecoveryLegSpdTurnback() const override { return recovery_leg_spd_turnback_.load(std::memory_order_acquire);}
  // clang-format on
protected:
  bool readVirtualLeg(int side, double pitch_rate, double &carrier_rate);
  bool observe(double pitch_rate, const Eigen::Vector2d &carrier_rates);
  void validateObservationTime();
  lqr10::Config lqr_config_;
  lqr10::NormalFeedback normal_feedback_;

private:
  friend class Normal;
  bool translation_source_active_{false}, translation_ramp_zero_{true};
  // Fixed recording storage, reset to missing each control call. No control consumers.
  std::atomic<bool> turn_debug_enabled_{false};
  bool turn_debug_capture_{false};
  std::array<double, 197> turn_debug_{};
  uint64_t turn_debug_cycle_{0};
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::DebugData>> turn_debug_pub_;
  ros::WallTimer turn_debug_timer_;
  void publishTurnDebug();
  bool updateEstimation(const ros::Time &time, const ros::Duration &period);
  void resetObservation(const ros::Time &time, const ros::Duration &period);
  void updateChassisState(const ros::Time &time);
  void publishMode();
  void publishCompatibilityStatus();
  ControlParams action_params_;
  bool spring_compensation_enabled_{false};
  ros::Time start_time_;
  bool command_authorized_{false};
  lqr10::ObservationHistory observation_history_;
  VelocityKalmanFilter velocity_kf_;
  Eigen::Matrix3d rotation_world_base_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d imu_position_base_{Eigen::Vector3d::Zero()}, hip_midpoint_base_{Eigen::Vector3d::Zero()};
  bool velocity_integrated_{false}; // Exactly one S/last_v update per valid observation.
  bool setupVelocityEstimator(ros::NodeHandle &controller_nh);
  bool updateGroundVelocity();
  void integrateVelocity();
  void finishObservation();

  bool setupParams(ros::NodeHandle &controller_nh);
  bool loadLqrParams(ros::NodeHandle &controller_nh, lqr10::Config &config);
  bool setupLQR(ros::NodeHandle &controller_nh);
  lqr10::GainUpdate polyfit(const lqr10::State10 &q, const lqr10::Input4 &r, lqr10::DesignReport &report) const;
  void reconfigCB(LQRWeightConfig &config, uint32_t level);
  void publishGainStatus(const ros::TimerEvent &);
  void configureLqrInterface(ros::NodeHandle &controller_nh);
  lqr10::Config design_config_; // Immutable non-RT model/domain/gates after init.
  realtime_tools::RealtimeBuffer<lqr10::GainUpdate> gain_update_buffer_;
  std::atomic<bool> control_running_{false};
  std::atomic<uint64_t> applied_gain_revision_{0};
  uint64_t requested_gain_revision_{0};
  std::mutex gain_callback_mutex_;
  boost::recursive_mutex gain_server_mutex_;
  uint64_t staged_gain_revision_{0};
  bool first_reconfigure_{true};
  LQRWeightConfig accepted_weights_;
  std::string gain_status_{"initialized"};
  std::unique_ptr<dynamic_reconfigure::Server<LQRWeightConfig>> lqr_server_;
  ros::Timer gain_status_timer_;
  bool setupModelParams(ros::NodeHandle &controller_nh);
  bool setupThresholdParams(ros::NodeHandle &controller_nh);
  bool setupSpringParams(ros::NodeHandle &controller_nh);
  bool setupChassisGeometryParams(ros::NodeHandle &controller_nh);
  void triggerDown5cmStairAction() { down_5cm_stair_flag_.store(true, std::memory_order_release); }
  bool down5cmStairSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  geometry_msgs::Twist odometry() override;

  std::shared_ptr<ModelParams> model_params_;
  std::shared_ptr<SpringParams> spring_params_;
  std::shared_ptr<ChassisGeometryParams> chassis_geometry_params_;
  std::shared_ptr<LegStateThresholdParams> leg_threshold_params_;

  int balance_mode_ = BalanceMode::SIT_DOWN;
  bool balance_state_changed_ = false;
  std::shared_ptr<ModeManager> mode_manager_;

  ChassisState chassis_state_;
  LegState leg_state_[2];
  double default_leg_length_{0.12};
  std::atomic_bool down_5cm_stair_flag_{false};
  // stand up
  bool complete_stand_ = false, overturn_ = false;
  // recovery
  std::atomic<bool> recovery_leg_spd_turnback_{false};

  // handles
  hardware_interface::ImuSensorHandle imu_handle_, gimbal_imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_;
  hardware_interface::JointHandle left_hip_joint_handle_, left_knee_joint_handle_, right_hip_joint_handle_,
      right_knee_joint_handle_;
  std::vector<hardware_interface::JointHandle *> joint_handles_;

  // Leg Cmd
  struct LegSetpoint
  {
    double length{.2};
    bool jump{false};
    ros::Time received;
  };
  realtime_tools::RealtimeBuffer<LegSetpoint> leg_setpoint_buffer_;
  ros::Time last_leg_setpoint_;
  double legCmd_{0.2};
  bool jumpCmd_{false};

  // ROS Interface
  ros::Subscriber leg_cmd_sub_, recovery_leg_spd_turnback_sub_;
  ros::Publisher unstick_pub_, upstair_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>> legged_chassis_mode_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisStatus>> legged_chassis_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedLQRStatus>> lqr_status_pub_;
  ros::Time cmd_update_time_;
  double last_status_time_{-1.};
  ros::ServiceServer down_5cm_stair_srv_;
};
} // namespace rm_chassis_controllers
