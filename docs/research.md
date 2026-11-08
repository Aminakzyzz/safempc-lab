# Research map

## Technical reading sequence

1. **Dynamics and linearisation.** Derive the unicycle model, discretise it, and verify Jacobians with finite differences.
2. **Stability and LQR.** Study the discrete algebraic Riccati equation, then compare the local LQR tracker to a geometric baseline.
3. **Constrained optimisation.** Formulate finite-horizon MPC with state, input, and obstacle constraints; distinguish hard and soft constraints.
4. **Safety.** Study control barrier functions (CBFs) and replace the initial geometric safety filter with a CBF-QP layer.
5. **Robotics deployment.** Wrap the controller in ROS 2, test it in a simulator, and document latency and failure modes.

## Primary resources

- Rawlings, Mayne, Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed. (2017).
- Ames et al., "Control Barrier Function Based Quadratic Programs for Safety Critical Systems," *IEEE TAC* (2017), DOI: [10.1109/TAC.2016.2638961](https://doi.org/10.1109/TAC.2016.2638961).
- Borrelli, Bemporad, Morari, *Predictive Control for Linear and Hybrid Systems*: [book page](https://mpc.berkeley.edu/mpc-course-material).
- Python Control Systems Library: [documentation](https://python-control.readthedocs.io/).
- CasADi: [official documentation](https://web.casadi.org/docs/).
- Modal: [Python guide](https://modal.com/docs/guide).

## Why this scope

The project demonstrates modelling, linearisation, discrete-time simulation, optimal control, constraints, evaluation, reproducibility, and cloud experiment design. It is more defensible than a polished visualisation alone, while avoiding a claim that a short project implements production-safe autonomy.
