#include <tinympc/tiny_api.hpp>
#include <tinympc/codegen.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void require(bool value, const char* message)
{
  if (!value)
    throw std::runtime_error(message);
}

struct Solver
{
  TinySolver* value{nullptr};
  ~Solver()
  {
    if (value)
    {
      delete value->solution;
      delete value->settings;
      delete value->cache;
      delete value->work;
      delete value;
    }
  }
};

void checkCone()
{
  tinyVector input(3), expected(3);
  input << 3., 4., 0.;
  expected << 1.5, 2., 2.5;
  require((project_soc(input, 1.f) - expected).norm() < 1e-12, "SOC projection changed");
  input << 0., 0., 2.;
  require((project_soc(input, 1.f) - input).norm() < 1e-12, "SOC interior changed");
  input << 0., 0., -2.;
  require(project_soc(input, 1.f).norm() < 1e-12, "SOC zero projection changed");
}

void checkControl(double limit, const std::string& name)
{
  constexpr int nx = 10, nu = 4, horizon = 16;
  tinyMatrix a = .9 * tinyMatrix::Identity(nx, nx);
  tinyMatrix b = tinyMatrix::Zero(nx, nu);
  b.topRows(nu) = .1 * tinyMatrix::Identity(nu, nu);
  tinyVector f = tinyVector::Zero(nx);
  tinyMatrix q = tinyMatrix::Identity(nx, nx), r = tinyMatrix::Identity(nu, nu);
  Solver solver;
  require(tiny_setup(&solver.value, a, b, f, q, r, 1., nx, nu, horizon, 0) == 0, "setup failed");
  auto* s = solver.value;
  s->settings->adaptive_rho = 0;
  s->settings->max_iter = 1000;
  s->settings->abs_pri_tol = 1e-7;
  s->settings->abs_dua_tol = 1e-7;
  require(tiny_set_bound_constraints(s,
      tinyMatrix::Constant(nx, horizon, -100.), tinyMatrix::Constant(nx, horizon, 100.),
      tinyMatrix::Constant(nu, horizon - 1, -limit), tinyMatrix::Constant(nu, horizon - 1, limit)) == 0,
      "bounds setup failed");
  require(tiny_set_x_ref(s, tinyMatrix::Zero(nx, horizon)) == 0, "state reference failed");
  require(tiny_set_u_ref(s, tinyMatrix::Zero(nu, horizon - 1)) == 0, "input reference failed");
  tinyVector x(nx);
  x << 1., -.8, .6, -.4, .2, -.15, .1, -.05, .03, -.02;
  double largest_input = 0.;
  tinyVector first;
  for (int step = 0; step < 40; ++step)
  {
    require(tiny_set_x0(s, x) == 0, "initial state failed");
    require(tiny_solve(s) == 0 && s->solution->solved == 1, "solver did not converge");
    require(s->solution->x.allFinite() && s->solution->u.allFinite(), "nonfinite solution");
    const tinyVector u = s->solution->u.col(0);
    largest_input = std::max(largest_input, u.cwiseAbs().maxCoeff());
    require(u.cwiseAbs().maxCoeff() <= limit + 1e-6, "input bound violated");
    const double defect = (s->solution->x.rightCols(horizon - 1) -
        a * s->solution->x.leftCols(horizon - 1) - b * s->solution->u).cwiseAbs().maxCoeff();
    require(defect < 1e-5, "predicted dynamics mismatch");
    if (step == 0)
      first = u;
    x = (a * x + b * u).eval();
  }
  require(x.norm() < .03, "closed-loop state did not decay");
  if (limit < 1.)
    require(largest_input > .99 * limit, "bounded case did not exercise saturation");
  std::cout << std::setprecision(17) << "RESULT " << name << " " << first.transpose()
            << " " << x.norm() << " " << largest_input << "\n";
  // Exercise the patched row-major code generation path as well as the solver.
  require(tiny_codegen(s, ("generated_" + name).c_str(), 0) == 0, "code generation failed");
}
}  // namespace

int main()
{
  try
  {
    std::cout << "EIGEN " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << "\n";
    checkCone();
    checkControl(10., "wide");
    checkControl(.2, "bounded");
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
