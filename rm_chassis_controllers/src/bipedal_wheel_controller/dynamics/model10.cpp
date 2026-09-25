#include "bipedal_wheel_controller/dynamics/model10.h"
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rm_chassis_controllers
{
namespace lqr10
{
namespace
{
using Vec5 = Eigen::Matrix<double, 5, 1>;
struct Leg
{
  double l, lb, lw, m, I, cb, cw, offset;
};

double chebyshev(const Curve8 &coefficients, double t)
{
  double b1 = 0., b2 = 0.;
  for (int i = coefficients.size() - 1; i >= 1; --i)
  {
    const double b = 2. * t * b1 - b2 + coefficients[i];
    b2 = b1;
    b1 = b;
  }
  return t * b1 - b2 + coefficients[0];
}

std::array<Leg, 2> physicalLegs(const PhysicalModel &p, const Eigen::Vector2d &lengths)
{
  if (!lengths.allFinite() || !p.geometry_domain.allFinite() || p.geometry_domain[0] <= 0. ||
      p.geometry_domain[1] <= p.geometry_domain[0])
    throw std::invalid_argument("invalid equivalent-leg domain/input");
  std::array<Leg, 2> legs;
  for (int i = 0; i < 2; ++i)
  {
    const double L0 = lengths[i];
    if (L0 < p.geometry_domain[0] || L0 > p.geometry_domain[1])
      throw std::invalid_argument("VMC length outside equivalent-leg domain");
    const double t = 2. * (L0 - p.geometry_domain[0]) / (p.geometry_domain[1] - p.geometry_domain[0]) - 1.;
    const auto &a = p.legs[i];
    auto &z = legs[i];
    z.l = L0; // Use the VMC hip-to-wheel length directly; wheel radius is separate.
    z.offset = chebyshev(a.offset, t);
    z.lb = chebyshev(a.lb, t);
    z.lw = chebyshev(a.lw, t);
    z.I = chebyshev(a.inertia, t);
    z.m = a.mass;
    if (!std::isfinite(z.offset) || std::min({z.l, z.lb, z.lw, z.I, z.m}) <= 0.)
      throw std::invalid_argument("nonpositive equivalent-leg parameter");
    const double tol = 32. * std::numeric_limits<double>::epsilon() * std::max({z.l, z.lb, z.lw});
    if (z.lb + z.lw < z.l - tol || std::abs(z.lb - z.lw) > z.l + tol)
      throw std::invalid_argument("COM triangle outside equivalent-leg domain");
    z.cb = (z.lb * z.lb + z.l * z.l - z.lw * z.lw) / (2. * z.lb * z.l);
    z.cw = (z.lw * z.lw + z.l * z.l - z.lb * z.lb) / (2. * z.lw * z.l);
    if (!std::isfinite(z.cb) || !std::isfinite(z.cw) ||
        std::max(std::abs(z.cb), std::abs(z.cw)) > 1. + 64. * std::numeric_limits<double>::epsilon())
      throw std::invalid_argument("invalid COM triangle cosine");
    z.cb = std::max(-1., std::min(1., z.cb));
    z.cw = std::max(-1., std::min(1., z.cw));
  }
  return legs;
}

// PDF force elimination, evaluated in the equivalent hip-to-wheel chord geometry.
// The generalized coordinates retain the established VMC angle convention.
Vec5 residual(const PhysicalModel &p, const std::array<Leg, 2> &legs, Vec5 q, const Vec5 &dq, const Vec5 &ddq,
              const Input4 &u)
{
  for (int i = 0; i < 2; ++i)
    q[i + 2] += legs[i].offset;
  auto sa = [](double a, double v, double d) { return std::cos(a) * d - std::sin(a) * v * v; };
  auto ca = [](double a, double v, double d) { return -std::sin(a) * d - std::cos(a) * v * v; };
  double hip_x = 0., hip_z = 0.;
  for (int i = 0; i < 2; ++i)
  {
    hip_x += legs[i].l * sa(q[i + 2], dq[i + 2], ddq[i + 2]) / 2.;
    hip_z += legs[i].l * ca(q[i + 2], dq[i + 2], ddq[i + 2]) / 2.;
  }
  const double sb = p.Rw * (ddq[0] + ddq[1]) / 2. + hip_x + p.lc * sa(q[4] + p.phi_c, dq[4], ddq[4]);
  const double hb = hip_z + p.lc * ca(q[4] + p.phi_c, dq[4], ddq[4]);
  Eigen::Vector2d fl, fws, fbs, hl, fbh;
  double fwh = p.mb * (hb + p.g);
  for (int i = 0; i < 2; ++i)
  {
    const auto &z = legs[i];
    fl[i] = (u[i] - p.Iw * ddq[i]) / p.Rw;
    fws[i] = fl[i] - p.mw * p.Rw * ddq[i];
    const double sl = p.Rw * ddq[i] + z.lw * sa(q[i + 2] - std::acos(z.cw), dq[i + 2], ddq[i + 2]);
    hl[i] = hip_z - z.lb * ca(q[i + 2] + std::acos(z.cb), dq[i + 2], ddq[i + 2]);
    fbs[i] = fws[i] - z.m * sl;
    fwh += z.m * (hl[i] + p.g);
  }
  fwh /= 2.;
  Vec5 out;
  for (int i = 0; i < 2; ++i)
  {
    const auto &z = legs[i];
    fbh[i] = fwh - z.m * (hl[i] + p.g);
    const double aw = q[i + 2] - std::acos(z.cw), ab = q[i + 2] + std::acos(z.cb);
    const double torque = fwh * z.lw * std::sin(aw) - fws[i] * z.lw * std::cos(aw) + fbh[i] * z.lb * std::sin(ab) -
                          fbs[i] * z.lb * std::cos(ab) - u[i] + u[i + 2];
    out[i] = z.I * ddq[i + 2] - torque;
  }
  out[2] = p.mb * sb - fbs.sum();
  out[3] = p.Ib * ddq[4] + u.tail<2>().sum() + fbs.sum() * p.lc * std::cos(q[4] + p.phi_c) -
           fbh.sum() * p.lc * std::sin(q[4] + p.phi_c);
  double yaw_acc = p.Rw * (ddq[1] - ddq[0]) / (2. * p.Rl);
  yaw_acc += (-legs[0].l * sa(q[2], dq[2], ddq[2]) + legs[1].l * sa(q[3], dq[3], ddq[3])) / (2. * p.Rl);
  out[4] = p.Izz * yaw_acc - (-fl[0] + fl[1]) * p.Rl;
  return out;
}

State10 poly33Basis(const Eigen::Vector2d &lengths, const Eigen::Matrix2d &domain)
{
  const Eigen::Vector2d n =
      (2. * (lengths - domain.col(0)).array() / (domain.col(1) - domain.col(0)).array() - 1.).matrix();
  const double x = n[0], y = n[1];
  State10 b;
  b << 1., x, y, x * x, x * y, y * y, x * x * x, x * x * y, x * y * y, y * y * y;
  return b;
}
} // namespace

Input4 staticFeedforward(const PhysicalModel &p)
{
  if (!std::isfinite(p.mb) || p.mb <= 0. || !std::isfinite(p.g) || p.g <= 0. || !std::isfinite(p.lc) || p.lc < 0. ||
      !std::isfinite(p.phi_c))
    throw std::invalid_argument("invalid body parameters for static feedforward");
  Input4 uff = Input4::Zero();
  uff.tail<2>().setConstant(.5 * p.mb * p.g * p.lc * std::sin(p.phi_c));
  if (!uff.allFinite())
    throw std::invalid_argument("nonfinite static feedforward");
  return uff;
}

State10 staticEquilibrium(const PhysicalModel &p, const Eigen::Vector2d &lengths, const Input4 &uff)
{
  if (!uff.allFinite())
    throw std::invalid_argument("nonfinite static feedforward");
  const auto legs = physicalLegs(p, lengths);
  const double fw = .5 * p.g * (p.mb + legs[0].m + legs[1].m);
  State10 equilibrium = State10::Zero();
  for (int i = 0; i < 2; ++i)
  {
    const auto &z = legs[i];
    const double a = fw * z.l - z.m * p.g * z.lb * z.cb;
    const double b = z.m * p.g * z.lb * std::sqrt(std::max(0., 1. - z.cb * z.cb));
    const double radius = std::hypot(a, b);
    if (!std::isfinite(radius) || radius <= 0.)
      throw std::invalid_argument("static equilibrium radius invalid");
    const double sine = -uff[2 + i] / radius;
    if (!std::isfinite(sine) || std::abs(sine) > 1.)
      throw std::invalid_argument("static equilibrium has no real solution");
    equilibrium[THETA_L + 2 * i] = std::atan2(b, a) + std::asin(sine) - z.offset;
  }
  if (!equilibrium.allFinite())
    throw std::invalid_argument("nonfinite static equilibrium");
  return equilibrium;
}

LinearModel linearModel(const PhysicalModel &p, const Eigen::Vector2d &lengths)
{
  for (double n : {p.mb, p.Ib, p.Izz, p.mw, p.Iw, p.Rw, p.Rl, p.g})
    if (!std::isfinite(n) || n <= 0.)
      throw std::invalid_argument("invalid physical model scalar");
  if (!std::isfinite(p.phi_c) || !std::isfinite(p.lc) || p.lc < 0.)
    throw std::invalid_argument("invalid body COM");
  const auto legs = physicalLegs(p, lengths);
  LinearModel model;
  model.uff = staticFeedforward(p);
  model.equilibrium = staticEquilibrium(p, lengths, model.uff);
  Vec5 q = Vec5::Zero(), zero = Vec5::Zero();
  q[2] = model.equilibrium[THETA_L];
  q[3] = model.equilibrium[THETA_R];
  const auto base = residual(p, legs, q, zero, zero, model.uff);
  if (base.cwiseAbs().maxCoeff() > 1e-8 || q.segment<2>(2).cwiseAbs().maxCoeff() >= .2)
    throw std::invalid_argument("static equilibrium residual/outside model envelope");
  Eigen::Matrix<double, 5, 5> M, Dq, T = Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::Matrix<double, 5, 4> Bu;
  for (int i = 0; i < 5; ++i)
  {
    Vec5 unit = zero;
    unit[i] = 1.;
    M.col(i) = residual(p, legs, q, zero, unit, model.uff) - base;
    Dq.col(i) = -(residual(p, legs, q + 1e-6 * unit, zero, zero, model.uff) -
                  residual(p, legs, q - 1e-6 * unit, zero, zero, model.uff)) /
                (2e-6);
  }
  for (int i = 0; i < 4; ++i)
  {
    Input4 unit = model.uff;
    unit[i] += 1.;
    Bu.col(i) = -(residual(p, legs, q, zero, zero, unit) - base);
  }
  T.row(0) << p.Rw / 2., p.Rw / 2., 0., 0., 0.;
  T.row(1) << -p.Rw / (2. * p.Rl), p.Rw / (2. * p.Rl),
      -legs[0].l * std::cos(q[2] + legs[0].offset) / (2. * p.Rl),
      legs[1].l * std::cos(q[3] + legs[1].offset) / (2. * p.Rl), 0.;
  const auto solve = M.fullPivLu();
  if (!solve.isInvertible() || solve.rcond() < 1e-10)
    throw std::invalid_argument("ill-conditioned dynamics");
  const Eigen::Matrix<double, 5, 5> a = T * solve.solve(Dq);
  const Eigen::Matrix<double, 5, 4> b = T * solve.solve(Bu);
  for (int i = 0; i < 5; ++i)
  {
    model.A(2 * i, 2 * i + 1) = 1.;
    for (int j = 0; j < 3; ++j)
      model.A(2 * i + 1, 4 + 2 * j) = a(i, 2 + j);
    model.B.row(2 * i + 1) = b.row(i);
  }
  const Vec5 r = T * solve.solve(-base);
  for (int i = 0; i < 5; ++i)
    model.residual[2 * i + 1] = r[i];
  return model;
}

bool evaluate(const Config &config, const Eigen::Vector2d &lengths, Gain10 &gain, Input4 &uff, State10 *equilibrium)
{
  gain.setZero();
  uff.setZero();
  if (!lengths.allFinite() || (lengths.array() < config.domain.col(0).array()).any() ||
      (lengths.array() > config.domain.col(1).array()).any())
    return false;
  const State10 b = poly33Basis(lengths, config.domain);
  for (int state = 0; state < 10; ++state)
    for (int input = 0; input < 4; ++input)
      gain(input, state) = b.dot(config.coeffs.col(input + 4 * state));
  try
  {
    const State10 eq = staticEquilibrium(config.model, lengths, config.uff);
    if (eq.cwiseAbs().maxCoeff() >= config.max_angle)
      return false;
    if (equilibrium)
      *equilibrium = eq;
  }
  catch (const std::exception &)
  {
    return false;
  }
  uff = config.uff;
  return gain.allFinite() && uff.allFinite();
}
} // namespace lqr10
} // namespace rm_chassis_controllers
