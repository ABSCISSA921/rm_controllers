#include "bipedal_wheel_controller/dynamics/model10.h"
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>
#include <array>
#include <stdexcept>

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
State10 basis(const Eigen::Vector2d &l, const Eigen::Matrix2d &domain)
{
  const Eigen::Vector2d n = (2. * (l - domain.col(0)).array() / (domain.col(1) - domain.col(0)).array() - 1.).matrix();
  const double x = n[0], y = n[1];
  State10 b;
  b << 1., x, y, x * x, x * y, y * y, x * x * x, x * x * y, x * y * y, y * y * y;
  return b;
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

  return update;
}
} // namespace lqr10
} // namespace rm_chassis_controllers
