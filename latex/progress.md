# SafeMPC Lab progress log

## 2026-11-08

- Created the public repository, GitHub Pages brief, and CI workflow.
- Installed and authenticated the Modal CLI in the local project environment.
- Ran one bounded nine-trial Modal smoke sweep; fixed the remote image by pinning it to Python 3.11.
- Implemented online RLS identification for an actuator-speed gain.
- Added two identification tests; the suite now has six passing tests.
- Selected the project direction: adaptive predictive control under actuator drift, followed by CBF-QP safety constraints and Modal Monte-Carlo evaluation.

## Next work

1. Add a simulator mode with known fixed and time-varying actuator gain.
2. Feed the RLS estimate into the MPC prediction model.
3. Produce comparable tracking, clearance, and control-effort plots.
4. Replace the geometric obstacle filter with explicit CBF-QP constraints.

## Learning roadmap

1. Reproduce and explain the baseline simulation.
2. Derive differential-drive kinematics and compare integrators.
3. Compare geometric control, LQR, and MPC under identical conditions.
4. Introduce fixed and drifting actuator gain; validate the RLS estimate.
5. Add explicit CBF-QP safety constraints and document all assumptions.
6. Run fixed-seed Monte-Carlo experiments on Modal and write results incrementally.

## Documentation update: research roadmap

- Replaced the initial card-based roadmap with a milestone rail containing six time-boxed gates.
- Added a system architecture reference: plant, observation, RLS identification, adaptive MPC, CBF safety layer, and experiment logging.
- Added comparable implementation references and prerequisite theory resources to the public roadmap.
