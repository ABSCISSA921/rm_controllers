#pragma once
#include <cmath>
namespace rm_chassis_controllers
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
}  // namespace rm_chassis_controllers
