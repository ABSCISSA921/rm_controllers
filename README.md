# rm_controllers
## Status
### Continuous Integration
[![CI](https://github.com/rm-controls/rm_controllers/actions/workflows/industrial_ci.yml/badge.svg)](https://github.com/rm-controls/rm_controllers/actions/workflows/industrial_ci.yml)
[![Format](https://github.com/rm-controls/rm_controllers/actions/workflows/format.yml/badge.svg)](https://github.com/rm-controls/rm_controllers/actions/workflows/format.yml)
[![Doxygen-docs](https://github.com/rm-controls/rm_controllers/actions/workflows/doxygen.yml/badge.svg)](https://github.com/rm-controls/rm_controllers/actions/workflows/doxygen.yml)

### Licenses
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2Frm-controls%2Frm_controllers.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2Frm-controls%2Frm_controllers?ref=badge_large)
## Documentation
[rm-controls Documentation](https://rm-control-docs.netlify.app/)

[rm-control Doxygen Documentation](https://rm-controller.netlify.app/)

## Multi-leg-length MPC

With `RM_BUILD_BIPEDAL=ON`, Normal ground feedback uses a bank of TinyMPC
backends built when the controller is loaded. Left and right leg lengths are
scheduled independently. The default axis contains 12 uniformly spaced points
over `lqr10/length_domain`, plus `default_leg_length` if it is not already a grid
point. For the domain `[0.06, 0.40]` and default length `0.12`, this creates 169
backends (13 x 13). Every backend owns its solver workspace and uses the same
startup Q/R, rho, horizon and torque bounds, with a fixed prediction step of
0.001 s.

The nearest backend is selected using measured leg lengths, with midpoint
hysteresis on each axis. Selection does not build models, discretize matrices,
recompute Riccati caches or access files. A selected backend is reset on entry
and on switches, including when revisiting a previously used backend. The
reference posture, solver equilibrium and feedforward all come from the same
selected node. Existing velocity/yaw references, axial force control and
mechanical modes are retained.

An optional custom axis replaces the default grid:

```yaml
mpc:
  leg_lengths: [0.06, 0.10, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32, 0.36, 0.40]
  length_hysteresis: 0.0005  # m; default 0.5 mm
```

The axis must be finite, strictly increasing and include both exact endpoints
of `lqr10/length_domain`. Hysteresis must be nonnegative and smaller than half
the smallest adjacent spacing. An invalid grid or any backend initialization
failure rejects controller loading. Invalid observations, out-of-domain leg
lengths and failed/late solves retain the Normal-to-PROTECT path and clear the
current joint commands. Solver failures log the selected lengths, status,
iteration count and elapsed time before the backend is reset.

The legacy dynamic Q/R interface is read-only: attempted changes are restored to
the applied startup values, with a warning and `update_status` explaining that a
controller reload is required. Stop/start and Normal re-entry do not rebuild the
bank. Startup no longer generates the unused LQR gain table.

This is discrete scheduling, without interpolation or offline cache files.
Grid spacing introduces model/trim approximation between nodes; midpoint
hysteresis reduces switching chatter but does not guarantee stability. More
nodes increase startup time and memory because each backend has its own
workspace. Node switching, cold-start convergence and the entire intended leg
length/rate range need validation on the target robot. `mpc/max_solve_time`
remains a check after synchronous solving returns, not a preemptive deadline.

Build the dedicated bank regression with bipedal support enabled, then run:

```bash
catkin build rm_chassis_controllers --make-args bipedal_mpc_bank_test
source devel/setup.bash
rostest rm_chassis_controllers bipedal_mpc_bank.test
```

It covers asymmetric nodes against direct backend construction, equilibrium
feedforward, axis hysteresis, revisited-backend resets, invalid grids/domain
inputs, Normal integration and failed-solve protection, and read-only Q/R.
The pre-existing `bipedal_observation_test` still references obsolete interfaces
and is not included in this dedicated regression.
