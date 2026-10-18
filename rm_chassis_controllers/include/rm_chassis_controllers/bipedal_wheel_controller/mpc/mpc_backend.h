#pragma once

#include <memory>

#include "bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
struct LinearModel;

// Normal-ground MPC interface. TinyMPC types and storage belong to the private implementation in the .cpp file.
// State order is Index; input order is [T_L, T_R, Tp_L, Tp_R], in N*m.
// T_L/T_R are wheel-joint torques; Tp_L/Tp_R are virtual leg torques.
// Internally solve for z = x - equilibrium and v = u - uff; callers use absolute states and total inputs.
// An instance has a single owner thread: initialization, bound updates, reset and solve must not run concurrently.
class mpc_backend
{
public:
  using StateTrajectory = Eigen::Matrix<double, 10, Eigen::Dynamic>;

  struct Params
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double sample_period{ 0. };    // Seconds; must be explicitly set > 0 and match the caller's MPC update period.
    int horizon{ 16 };             // N state nodes, N-1 control intervals; must be >= 2.
    State10 q{ State10::Ones() };  // Diagonal state weights, finite and >= 0.
    Input4 r{ Input4::Ones() };    // Diagonal input-deviation weights, finite and > 0.

    // Absolute total U4 bounds, finite with input_min <= input_max. Configure physical limits before use.
    // The implementation subtracts uff before giving these bounds to TinyMPC. Zero defaults allow only zero input.
    // Tp bounds constrain virtual leg torques; hip/knee limits still need checking after VMC.
    Input4 input_min{ Input4::Zero() };
    Input4 input_max{ Input4::Zero() };

    double rho{ 1. };            // Fixed ADMM penalty, finite and > 0; adaptive rho is disabled.
    int max_iter{ 100 };         // Positive iteration cap; tune against the actual control budget.
    double abs_pri_tol{ 1e-4 };  // Positive, finite primal residual tolerance.
    double abs_dua_tol{ 1e-4 };  // Positive, finite dual residual tolerance.
    // Seconds, finite and >= 0; 0 disables elapsed-time rejection. This is not a solver preemption deadline.
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
    double solve_time{ 0. };  // Elapsed solver time in seconds, measured using a monotonic clock.
  };

  mpc_backend();
  ~mpc_backend();  // Defined out of line where Impl is complete; releases the owned solver.

  mpc_backend(const mpc_backend&) = delete;
  mpc_backend& operator=(const mpc_backend&) = delete;
  mpc_backend(mpc_backend&&) = delete;
  mpc_backend& operator=(mpc_backend&&) = delete;

  // Non-RT: validate parameters/trim, discretize the continuous LinearModel using sample_period, allocate the
  // workspace and precompute all caches. Reinitialize after model, leg length, sampling, Q/R or rho changes.
  // Reinitialization invalidates old warm starts/results; on failure the backend is left uninitialized.
  bool initialize(const LinearModel& continuous_model, const Params& params);

  // Non-RT: update absolute total U4 bounds, converting them with the initialized model's uff.
  // Requires initialization and finite, ordered bounds. Invalid updates return false without replacing valid bounds.
  // Successful updates clear the warm start/results so that no solution from the former bounds can be reused.
  bool setInputBounds(const Input4& input_min, const Input4& input_max);

  // reference is a finite 10 x N absolute state trajectory; column 0 is the current reference, subsequent columns
  // are spaced by sample_period. Generate position/heading references consistently with velocity/yaw-rate targets.
  // Convert state/reference by subtracting the same model equilibrium; the internal v reference is zero.
  // Return true only for a converged, finite, timely solution whose first total U4 satisfies the configured bounds.
  // On success, output = uff + v.col(0): the caller must not add feedforward again. On failure output is unchanged;
  // the caller must use a separately validated fallback, not the previous command. See report() for diagnostics.
  // This interface does not guarantee allocation-free or hard real-time execution of the upstream solver.
  bool solve(const State10& state, const StateTrajectory& reference, Input4& output);

  // Clear warm starts and previous diagnostics on mode changes/reentry; retain the initialized model and bounds.
  void reset();
  bool initialized() const noexcept;
  const Report& report() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace rm_chassis_controllers
