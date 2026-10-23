#pragma once

#include <memory>

#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
struct LinearModel;

class mpc_backend
{
public:
  using StateTrajectory = Eigen::Matrix<double, 10, Eigen::Dynamic>;

  struct Params
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 预测模型的离散采样周期，单位 s
    double sample_period{ 0. };
    // 预测状态节点数 N，乘上采样周期就是预测时间范围
    int horizon{ 16 };
    State10 q{ State10::Ones() };
    Input4 r{ Input4::Ones() };

    // 各预测步总输入的下界，顺序和 r 力度相同，单位 N·m；
    // 控制器默认配置为 [-3.58, -3.58, -infinity, -infinity]。
    Input4 input_min{ Input4::Zero() };
    // 各预测步总输入的上界，顺序和 r 力度相同，单位 N·m；
    // 控制器默认配置为 [3.58, 3.58, infinity, infinity]。
    Input4 input_max{ Input4::Zero() };

    // 固定 ADMM 惩罚参数
    double rho{ 1000. };
    // 单次求解的最大 ADMM 迭代次数
    int max_iter{ 100 };
    double abs_pri_tol{ 1e-4 };
    double abs_dua_tol{ 1e-4 };
    // 求解耗时接受上限，单位 s；0 表示不检查
    // 求解返回后才检查，超过时拒绝结果；不会中断正在执行的求解器。
    double max_solve_time{ 0. };
  };

  enum class Status
  {
    NOT_INITIALIZED,
    SUCCESS,
    INVALID_INPUT,
    SOLVER_FAILED,
    TIME_LIMIT
  };

  struct Report
  {
    Status status{ Status::NOT_INITIALIZED };
    int iterations{ 0 };
    double solve_time{ 0. };
  };

  mpc_backend();
  ~mpc_backend();

  mpc_backend(const mpc_backend&) = delete;
  mpc_backend& operator=(const mpc_backend&) = delete;
  mpc_backend(mpc_backend&&) = delete;
  mpc_backend& operator=(mpc_backend&&) = delete;
  bool initialize(const LinearModel& continuous_model, const Params& params);
  bool setInputBounds(const Input4& input_min, const Input4& input_max);
  bool solve(const State10& state, const StateTrajectory& reference, Input4& output);
  void reset();
  bool initialized() const noexcept;
  const Report& report() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
