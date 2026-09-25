#include "bipedal_wheel_controller/dynamics/model10.h"
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>
#include <array>
#include <stdexcept>
#include <vector>

namespace rm_chassis_controllers
{
namespace lqr10
{
namespace
{
double maxReal(const Eigen::MatrixXd &a)
{
  Eigen::EigenSolver<Eigen::MatrixXd> e(a, false);
  if (e.info() != Eigen::Success)
    throw std::runtime_error("eigenvalue calculation failed");
  return e.eigenvalues().real().maxCoeff();
}
double radius(const Eigen::MatrixXd &a)
{
  Eigen::EigenSolver<Eigen::MatrixXd> e(a, false);
  if (e.info() != Eigen::Success)
    throw std::runtime_error("sampled eigenvalue calculation failed");
  return e.eigenvalues().cwiseAbs().maxCoeff();
}
State10 basis(const Eigen::Vector2d &l, const Eigen::Matrix2d &domain)
{
  const Eigen::Vector2d n = (2. * (l - domain.col(0)).array() / (domain.col(1) - domain.col(0)).array() - 1.).matrix();
  const double x = n[0], y = n[1];
  State10 b;
  b << 1., x, y, x * x, x * y, y * y, x * x * x, x * x * y, x * y * y, y * y * y;
  return b;
}
double curveAt(const Curve8 &coefficients, double length, const Eigen::Vector2d &domain)
{
  const double t = 2. * (length - domain[0]) / (domain[1] - domain[0]) - 1.;
  double b1 = 0., b2 = 0.;
  for (int i = coefficients.size() - 1; i >= 1; --i)
  {
    const double b = 2. * t * b1 - b2 + coefficients[i];
    b2 = b1;
    b1 = b;
  }
  return t * b1 - b2 + coefficients[0];
}
} // namespace

Gain10 solveCare(const LinearModel &m, const State10 &q, const Input4 &r, DesignReport &report)
{
  if (!q.allFinite() || !r.allFinite() || (q.array() <= 0.).any() || (r.array() <= 0.).any())
    throw std::invalid_argument("Q10 and R4 must be finite and strictly positive");
  const Eigen::MatrixXd Q = q.asDiagonal(), Rinv = r.cwiseInverse().asDiagonal();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(20, 20);
  H.topLeftCorner(10, 10) = m.A;
  H.topRightCorner(10, 10) = -m.B * Rinv * m.B.transpose();
  H.bottomLeftCorner(10, 10) = -Q;
  H.bottomRightCorner(10, 10) = -m.A.transpose();
  Eigen::ComplexEigenSolver<Eigen::MatrixXd> eig(H);
  if (eig.info() != Eigen::Success)
    throw std::runtime_error("CARE Hamiltonian eigensolver failed");
  Eigen::MatrixXcd stable(20, 10);
  int count = 0;
  for (int i = 0; i < 20; ++i)
    if (eig.eigenvalues()[i].real() < -1e-8)
    {
      if (count == 10)
        throw std::runtime_error("CARE stable subspace dimension > 10");
      stable.col(count++) = eig.eigenvectors().col(i);
    }
  if (count != 10)
    throw std::runtime_error("CARE stable subspace dimension != 10");
  const Eigen::MatrixXcd top = stable.topRows(10);
  const auto lu = top.fullPivLu();
  if (!lu.isInvertible() || lu.rcond() < 1e-12)
    throw std::runtime_error("CARE invariant subspace ill-conditioned");
  const Eigen::MatrixXcd pc = stable.bottomRows(10) * lu.inverse();
  if (pc.imag().norm() > 1e-7 * std::max(1., pc.real().norm()))
    throw std::runtime_error("CARE non-real solution");
  Eigen::MatrixXd P = pc.real();
  P = (P + P.transpose()).eval() / 2.;
  Gain10 K = Rinv * m.B.transpose() * P;
  const Eigen::MatrixXd res = m.A.transpose() * P + P * m.A - P * m.B * Rinv * m.B.transpose() * P + Q;
  const double rel = res.norm() / std::max({1., Q.norm(), (m.A.transpose() * P).norm()});
  const double minimum = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(P).eigenvalues().minCoeff();
  const double pole = maxReal(m.A - m.B * K);
  if (!K.allFinite() || rel > 1e-8 || minimum <= 0. || pole >= -1e-7)
    throw std::runtime_error("CARE residual/positivity/stability validation failed");
  report.care_residual = std::max(report.care_residual, rel);
  report.minimum_p = report.nodes ? std::min(report.minimum_p, minimum) : minimum;
  report.max_real = std::max(report.max_real, pole);
  ++report.nodes;
  return K;
}

void validateSampled(const LinearModel &m, const Gain10 &k, double dt, DesignReport &report)
{
  if (!std::isfinite(dt) || dt <= 0.)
    throw std::invalid_argument("invalid sample period");
  Eigen::MatrixXd aug = Eigen::MatrixXd::Zero(14, 14);
  aug.topLeftCorner(10, 10) = m.A;
  aug.topRightCorner(10, 4) = m.B;
  const Eigen::MatrixXd exp = (aug * dt).exp();
  const Eigen::MatrixXd ad = exp.topLeftCorner(10, 10), bd = exp.topRightCorner(10, 4);
  Eigen::MatrixXd delay = Eigen::MatrixXd::Zero(14, 14);
  delay.topLeftCorner(10, 10) = ad;
  delay.topRightCorner(10, 4) = bd;
  delay.bottomLeftCorner(4, 10) = -k;
  const double rho = std::max(radius(ad - bd * k), radius(delay));
  report.max_rho = std::max(report.max_rho, rho);
  if (!std::isfinite(rho) || rho >= 1.)
    throw std::runtime_error("sampled/one-sample-delay closed loop unstable");
}

GainUpdate designTable(const Config &config, const State10 &q, const Input4 &r, DesignReport &report)
{
  Eigen::MatrixXd rows(144, 10), ks(144, 40);
  const auto &d = config.domain;
  for (int i = 0; i < 12; ++i)
    for (int j = 0; j < 12; ++j)
    {
      const Eigen::Vector2d l(d(0, 0) + (d(0, 1) - d(0, 0)) * i / 11., d(1, 0) + (d(1, 1) - d(1, 0)) * j / 11.);
      const auto m = linearModel(config.model, l);
      report.equilibrium_error = std::max(report.equilibrium_error, m.residual.cwiseAbs().maxCoeff());
      const auto k = solveCare(m, q, r, report);
      for (double dt : {config.dt_min, config.dt_max})
        validateSampled(m, k, dt, report);
      const int row = 12 * i + j;
      rows.row(row) = basis(l, d).transpose();
      for (int state = 0; state < 10; ++state)
        for (int input = 0; input < 4; ++input)
          ks(row, input + 4 * state) = k(input, state);
    }
  const auto qr = rows.colPivHouseholderQr();
  if (qr.rank() != 10)
    throw std::runtime_error("poly33 basis is rank deficient");
  GainUpdate update;
  update.coeffs = qr.solve(ks);
  update.q = q;
  update.r = r;
  report.fit_error = (rows * update.coeffs - ks).cwiseAbs().maxCoeff();
  Config fit = config;
  fit.coeffs = update.coeffs;

  auto validatePoint = [&](const Eigen::Vector2d &l) {
    const auto m = linearModel(config.model, l);
    report.equilibrium_error = std::max(report.equilibrium_error, m.residual.cwiseAbs().maxCoeff());
    Gain10 k;
    Input4 uff;
    State10 equilibrium;
    if (!evaluate(fit, l, k, uff, &equilibrium))
      throw std::runtime_error("poly33 evaluation/static trim failed");
    if ((equilibrium - m.equilibrium).cwiseAbs().maxCoeff() > 1e-12 ||
        (m.B * (uff - m.uff)).cwiseAbs().maxCoeff() > 1e-12)
      throw std::runtime_error("analytic static trim mismatch");
    if (maxReal(m.A - m.B * k) >= 0.)
      throw std::runtime_error("poly33 continuous closed loop unstable");
    for (int n = 0; n < 5; ++n)
      validateSampled(m, k, config.dt_min + (config.dt_max - config.dt_min) * n / 4., report);
    ++report.validation_points;
  };

  for (int i = 0; i < 15; ++i)
    for (int j = 0; j < 15; ++j)
      validatePoint({d(0, 0) + (d(0, 1) - d(0, 0)) * i / 14.,
                     d(1, 0) + (d(1, 1) - d(1, 0)) * j / 14.});
  const std::array<double, 10> required{{.09, .090492, .10, .12, .14, .20, .24, .30, .34, .35}};
  for (double left : required)
    for (double right : required)
      validatePoint({left, right});

  const Eigen::Vector2d low = d.col(0), high = d.col(1), mid = (low + high) / 2.;
  const std::array<Eigen::Vector2d, 5> points{{low, mid, high, {low[0], high[1]}, {high[0], low[1]}}};
  for (int field = 0; field < 9; ++field)
    for (double scale : {.8, 1.2})
      for (int n = 0; n < (field >= 6 ? 4 : 5); ++n)
      {
        PhysicalModel p = config.model;
        bool rejected_geometry = false;
        const std::array<double *, 6> scalar{{&p.mb, &p.Ib, &p.Izz, &p.mw, &p.Iw, &p.lc}};
        if (field < 6)
          *scalar[field] *= scale;
        else
          for (int side = 0; side < 2; ++side)
          {
            auto &z = p.legs[side];
            if (field == 6)
              z.mass *= scale;
            if (field == 7)
              z.inertia *= scale;
            if (field == 8)
            {
              const double L = points[n][side];
              const double l = L;
              const double lb = curveAt(z.lb, L, p.geometry_domain);
              const double lw = curveAt(z.lw, L, p.geometry_domain);
              const double cb = (lb * lb + l * l - lw * lw) / (2. * lb * l);
              const double along = lb * cb * scale;
              const double transverse = lb * std::sqrt(std::max(0., 1. - cb * cb)) * scale;
              if (along <= 0. || along >= l)
              {
                rejected_geometry = true;
                break;
              }
              z.lb.setZero();
              z.lb[0] = std::hypot(along, transverse);
              z.lw.setZero();
              z.lw[0] = std::hypot(l - along, transverse);
            }
          }
        if (rejected_geometry)
          continue;
        Gain10 k;
        Input4 uff;
        if (!evaluate(fit, points[n], k, uff))
          throw std::runtime_error("sensitivity outside gain domain");
        validateSampled(linearModel(p, points[n]), k, config.dt_max, report);
      }
  return update;
}
} // namespace lqr10
} // namespace rm_chassis_controllers
