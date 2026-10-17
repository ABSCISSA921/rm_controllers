#include "bipedal_wheel_controller/dynamics/model10.h"
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>
#include <algorithm>
#include <stdexcept>

namespace rm_chassis_controllers
{
PhysicalLeg makePhysicalLeg(double mass)
{
  PhysicalLeg leg;
  leg.mass = mass;
  leg.offset << 0.00781088, -0.00731481, 0.00274022, -0.00087926, 0.00038916, -0.00007942;
  leg.lb << 0.08640568, 0.05103905, 0.01026298, -0.00690686, 0.00350066, -0.00201761;
  leg.lw << 0.17428161, 0.07786379, 0.00902815, -0.00340298, 0.00133048, -0.00048186;
  leg.inertia << 0.01540610, -0.00059323, -0.00094531, 0.00026475, -0.00020695, 0.00004118;
  return leg;
}

static double maxReal(const Eigen::MatrixXd& a)
{
  Eigen::EigenSolver<Eigen::MatrixXd> e(a, false);
  if (e.info() != Eigen::Success)
    throw std::runtime_error("eigenvalue calculation failed");
  return e.eigenvalues().real().maxCoeff();
}
static State10 basis(const Eigen::Vector2d& l, const Eigen::Matrix2d& domain)
{
  const Eigen::Vector2d n = (2. * (l - domain.col(0)).array() / (domain.col(1) - domain.col(0)).array() - 1.).matrix();
  const double x = n[0], y = n[1];
  State10 b;
  b << 1., x, y, x * x, x * y, y * y, x * x * x, x * x * y, x * y * y, y * y * y;
  return b;
}

Gain10 solveCare(const LinearModel& m, const State10& q, const Input4& r, DesignReport& report)
{
  if ((q.array() < 0.).any() || (r.array() <= 0.).any())
    throw std::invalid_argument("Q10 must be positive semidefinite and R4 positive definite");
  const Eigen::MatrixXd q_matrix = q.asDiagonal(), r_inverse = r.cwiseInverse().asDiagonal();
  Eigen::MatrixXd hamiltonian = Eigen::MatrixXd::Zero(20, 20);
  hamiltonian.topLeftCorner(10, 10) = m.A;
  hamiltonian.topRightCorner(10, 10) = -m.B * r_inverse * m.B.transpose();
  hamiltonian.bottomLeftCorner(10, 10) = -q_matrix;
  hamiltonian.bottomRightCorner(10, 10) = -m.A.transpose();
  Eigen::ComplexEigenSolver<Eigen::MatrixXd> eig(hamiltonian);
  if (eig.info() != Eigen::Success)
    throw std::runtime_error("CARE Hamiltonian eigensolver failed");
  Eigen::MatrixXcd stable(20, 10);
  int count = 0;
  for (int i = 0; i < 20; ++i)
    if (eig.eigenvalues()[i].real() < 0.)
    {
      if (count == 10)
        throw std::runtime_error("CARE stable subspace dimension > 10");
      stable.col(count++) = eig.eigenvectors().col(i);
    }
  if (count != 10)
    throw std::runtime_error("CARE stable subspace dimension != 10");
  const Eigen::MatrixXcd top = stable.topRows(10);
  const auto lu = top.fullPivLu();
  if (!lu.isInvertible())
    throw std::runtime_error("CARE invariant subspace is singular");
  const Eigen::MatrixXcd pc = stable.bottomRows(10) * lu.inverse();
  Eigen::MatrixXd riccati = pc.real();
  riccati = (riccati + riccati.transpose()).eval() / 2.;
  Gain10 gain = r_inverse * m.B.transpose() * riccati;
  const Eigen::MatrixXd res =
      m.A.transpose() * riccati + riccati * m.A - riccati * m.B * r_inverse * m.B.transpose() * riccati + q_matrix;
  const double rel = res.norm() / std::max({ 1., q_matrix.norm(), (m.A.transpose() * riccati).norm() });
  const double pole = maxReal(m.A - m.B * gain);
  if (!gain.allFinite() || rel > 1e-8 || pole >= 0.)
    throw std::runtime_error("CARE residual/stability validation failed");
  report.care_residual = std::max(report.care_residual, rel);
  report.max_real = std::max(report.max_real, pole);
  ++report.nodes;
  return gain;
}

GainUpdate designTable(const Config& config, const State10& q, const Input4& r, DesignReport& report)
{
  Eigen::MatrixXd rows(144, 10), ks(144, 40);
  const auto& d = config.domain;
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
  GainUpdate update;
  update.coeffs = qr.solve(ks);
  update.q = q;
  update.r = r;
  report.fit_error = (rows * update.coeffs - ks).cwiseAbs().maxCoeff();
  Config fit = config;
  fit.coeffs = update.coeffs;

  auto validate_point = [&](const Eigen::Vector2d& l) {
    const auto m = linearModel(config.model, l);
    report.equilibrium_error = std::max(report.equilibrium_error, m.residual.cwiseAbs().maxCoeff());
    Gain10 k;
    Input4 uff;
    evaluate(fit, l, k, uff);
    if (!k.allFinite())
      throw std::runtime_error("nonfinite poly33 gain");
    if (maxReal(m.A - m.B * k) >= 0.)
      throw std::runtime_error("poly33 continuous closed loop unstable");
    ++report.validation_points;
  };

  for (int i = 0; i < 15; ++i)
    for (int j = 0; j < 15; ++j)
      validate_point({ d(0, 0) + (d(0, 1) - d(0, 0)) * i / 14., d(1, 0) + (d(1, 1) - d(1, 0)) * j / 14. });

  return update;
}
}  // namespace rm_chassis_controllers
