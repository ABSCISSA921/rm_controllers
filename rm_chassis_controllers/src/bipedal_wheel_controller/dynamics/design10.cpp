#include "bipedal_wheel_controller/dynamics/model10.h"
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>
#include <algorithm>
#include <stdexcept>

namespace rm_chassis_controllers
{
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
  return solveCareFull(m, q.asDiagonal(), r, report);
}

Gain10 solveCareFull(const LinearModel& m, const StateCost10& q, const Input4& r, DesignReport& report)
{
  if (!q.allFinite() || !r.allFinite() || !q.isApprox(q.transpose(), 1e-12) || (r.array() <= 0.).any())
    throw std::invalid_argument("Q10 must be finite symmetric and R4 positive definite");
  const Eigen::SelfAdjointEigenSolver<StateCost10> q_eigen(q);
  if (q_eigen.info() != Eigen::Success || q_eigen.eigenvalues().minCoeff() < -1e-10 * std::max(1., q.norm()))
    throw std::invalid_argument("Q10 must be positive semidefinite");
  const Eigen::MatrixXd q_matrix = q, r_inverse = r.cwiseInverse().asDiagonal();
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

template <typename CostFunction>
GainUpdate designTableWithCost(const Config& config, const State10& weights, const Input4& r, DesignReport& report,
                               const CostFunction& cost)
{
  Eigen::MatrixXd rows(144, 10), ks(144, 40);
  const auto& d = config.domain;
  for (int i = 0; i < 12; ++i)
    for (int j = 0; j < 12; ++j)
    {
      const Eigen::Vector2d l(d(0, 0) + (d(0, 1) - d(0, 0)) * i / 11., d(1, 0) + (d(1, 1) - d(1, 0)) * j / 11.);
      const auto m = linearModel(config.model, l);
      report.equilibrium_error = std::max(report.equilibrium_error, m.residual.cwiseAbs().maxCoeff());
      const auto k = solveCareFull(m, cost(l, m.equilibrium), r, report);
      const int row = 12 * i + j;
      rows.row(row) = basis(l, d).transpose();
      for (int state = 0; state < 10; ++state)
        for (int input = 0; input < 4; ++input)
          ks(row, input + 4 * state) = k(input, state);
    }

  const auto qr = rows.colPivHouseholderQr();
  GainUpdate update;
  update.coeffs = qr.solve(ks);
  update.q = weights;
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

GainUpdate designTable(const Config& config, const State10& q, const Input4& r, DesignReport& report)
{
  if ((q.array() < 0.).any())
    throw std::invalid_argument("Q10 must be positive semidefinite");
  return designTableWithCost(config, q, r, report,
                             [&](const Eigen::Vector2d&, const State10&) -> StateCost10 { return q.asDiagonal(); });
}

GainUpdate designBaseTaskTable(const Config& config, const State10& output_weights, const State10& internal_weights,
                               const Input4& r, const Eigen::Vector3d& hip_midpoint_base, DesignReport& report)
{
  if ((output_weights.array() < 0.).any() || (internal_weights.array() < 0.).any())
    throw std::invalid_argument("Base task and internal weights must be nonnegative");
  return designTableWithCost(config, output_weights, r, report,
                             [&](const Eigen::Vector2d& lengths, const State10& trim) {
                               return baseTaskCost(lengths, trim, hip_midpoint_base, output_weights, internal_weights);
                             });
}
}  // namespace rm_chassis_controllers
