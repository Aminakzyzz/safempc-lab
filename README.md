# SafeMPC Lab

**A reproducible control-theory benchmark for safe mobile-robot navigation.**

SafeMPC Lab is an adaptive safe-control benchmark for a differential-drive robot. Its first release implements a geometric baseline, linear-quadratic regulation (LQR), box-constrained linear MPC, a safety monitor, and online recursive least-squares identification of actuator-gain mismatch. The project focuses on tracking accuracy, actuator limits, collision margin, and robustness to model mismatch.

> Built as a portfolio project for robotics and autonomous-systems study. It is a simulator and research prototype, not safety-certified control software.

## Included

- Kinematic differential-drive model with Euler and RK4 integration
- Pure-pursuit baseline and discrete-time LQR tracker
- Box-constrained, finite-horizon linear MPC and obstacle safety monitor
- Online RLS actuator-gain estimator for wheel-slip and model-mismatch experiments
- Deterministic scenario definitions and tests for dynamics and controller limits
- Modal-ready parameter-sweep entry point for Monte-Carlo experiments

## Quick start

```bash
python -m venv .venv
. .venv/bin/activate
pip install -e '.[dev]'
pytest
python -m safempc_lab.simulation
```

## Benchmark question

**How do controller choice, horizon length, and actuator limits affect tracking error and safety margin for a differential-drive robot under model mismatch?**

The initial benchmark evaluates a figure-eight reference in a bounded workspace with a circular obstacle. Metrics are RMS position error, peak heading error, control effort, minimum obstacle clearance, and constraint violations.

## Modal experiments

`modal_sweep.py` is separated from the core simulator. Once Modal authentication is configured, it can distribute independent controller parameter trials:

```bash
pip install modal
modal setup
modal run modal_sweep.py
```

Modal authentication is not configured on this machine. No remote compute has started and no credits have been consumed.

## Roadmap

- [x] Differential-drive model, baseline controller, LQR, safety monitor
- [x] Box-constrained linear MPC solver
- [x] Online scalar actuator-gain identification
- [ ] Adaptive MPC benchmark under fixed and drifting actuator gain
- [ ] CBF-QP obstacle constraints and benchmark report
- [ ] Modal Monte-Carlo sweep with persisted artifacts
- [ ] Nonlinear MPC using CasADi/acados
- [ ] ROS 2 wrapper and Gazebo/MuJoCo integration

## Research and license

See [docs/project-direction.md](docs/project-direction.md) for the research direction, [docs/research.md](docs/research.md) for primary sources, [docs/validation.md](docs/validation.md) for current limitations, and [MIT](LICENSE) for the license.
