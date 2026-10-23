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

    // 预测模型的离散采样周期，单位 s，必须 > 0；控制器初始化时固定为 0.001 s。
    double sample_period{ 0. };
    // 预测状态节点数 N，必须 >= 2；包含 N-1 个控制输入，预测跨度为 (N-1)*sample_period。
    int horizon{ 16 };
    // 传入 SDK 的状态跟踪代价 Q 的对角元素，必须非负；控制器从 YAML 的 q 读取。
    // 顺序：位移、速度、航向角、航向角速度、左腿角/角速度、右腿角/角速度、机身俯仰角/角速度。
    State10 q{ State10::Ones() };
    // 输入相对静态前馈的偏差代价 R 的对角元素，必须 > 0；控制器从 YAML 的 r 读取。
    // 顺序：左轮、右轮、左腿虚拟摆动、右腿虚拟摆动力矩。
    Input4 r{ Input4::Ones() };

    // 各预测步总输入的下界，顺序同 r，单位 N·m，包含静态前馈；-infinity 表示无下界。
    // 控制器默认配置为 [-3.58, -3.58, -infinity, -infinity]。
    Input4 input_min{ Input4::Zero() };
    // 各预测步总输入的上界，必须逐项 >= input_min；+infinity 表示无上界。
    // 控制器默认配置为 [3.58, 3.58, infinity, infinity]。
    Input4 input_max{ Input4::Zero() };

    // 固定 ADMM 惩罚参数，必须 > 0；官方缓存近似也会影响有效代价，需与 Q/R 联合调节。
    double rho{ 1000. };
    // 单次求解的最大 ADMM 迭代次数，必须 > 0；达到上限仍未收敛时拒绝结果。
    int max_iter{ 100 };
    // 原始残差绝对阈值，必须 > 0：状态/输入与对应投影变量的最大绝对差。
    double abs_pri_tol{ 1e-4 };
    // 对偶残差绝对阈值，必须 > 0：rho 乘以状态/输入投影变量迭代变化的最大绝对值。
    double abs_dua_tol{ 1e-4 };
    // 求解耗时接受上限，单位 s；0 表示不检查，控制器默认设为 0.008 s，可由 ROS 参数覆盖。
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
