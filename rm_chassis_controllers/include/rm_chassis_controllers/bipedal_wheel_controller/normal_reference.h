#pragma once
#include <cmath>
namespace rm_chassis_controllers
{
namespace lqr10
{
// Exact zero-order-held target solution of a critically damped length reference.
struct LengthReference
{
  double position{}, velocity{};
  void reset(double p)
  {
    position = p;
    velocity = 0.;
  }
  double update(double target, double dt, double tau)
  {
    const double w = 1. / tau, error = position - target;
    const double a = velocity + w * error, decay = std::exp(-w * dt);
    position = target + (error + a * dt) * decay;
    velocity = (velocity - w * a * dt) * decay;
    return position;
  }
};
// Reference-only state: never modifies the measured S/V or their estimator.
struct LongitudinalReference
{
  bool moving{ false };
  double released_error{};
  // Called only when NormalFeedback reference history is initialized.
  void reset(bool complete)
  {
    moving = complete;
    released_error = 0.;
  }
  void update(double s, double v, double vref, bool complete, bool source_active, bool ramp_zero, double dt, double tau,
              double capture_speed, double& reference)
  {
    if (!complete)
    {
      moving = false;
      return;
    }
    if (!moving)
    {
      if (source_active || !ramp_zero || vref != 0.)
      {
        // Preserve the old anchor on the transition cycle.
        released_error = s - reference;
        moving = true;
      }
    }
    else
    {
      released_error *= std::exp(-dt / tau);
      reference = s - released_error;
      // Capture once after braking. Subsequent disturbances cannot move the anchor.
      if (!source_active && ramp_zero && vref == 0. && std::abs(v) <= capture_speed)
        moving = false;
    }
  }
};
}  // namespace lqr10
}  // namespace rm_chassis_controllers
