# Project direction: Adaptive SafeMPC under actuator drift

## Decision

SafeMPC Lab will become an **adaptive safe-control benchmark**, not merely a path-following MPC demo.

The robot receives a nominal command but its true forward-speed gain changes because of wheel slip, payload variation, actuator degradation, or unmodelled delay. An online recursive least-squares (RLS) estimator identifies the speed gain from streamed observations. The predictive controller then uses the updated model, while a safety layer applies conservative constraint tightening based on identification uncertainty.

## Research question

**When a differential-drive robot has an unknown, time-varying actuator gain, how do nominal MPC, fixed-margin robust MPC, and online-adaptive MPC trade tracking accuracy, control effort, feasibility, and obstacle clearance?**

This is an engineering research question, not a claim of a novel theoretical contribution.

## Experimental ladder

1. **Known model:** compare pure pursuit, LQR, and nominal MPC.
2. **Fixed mismatch:** introduce a constant speed-gain error and measure prediction and tracking degradation.
3. **Drift:** vary the gain during a trajectory; compare nominal MPC and RLS-adaptive MPC.
4. **Safety:** add obstacle constraints and a CBF-QP safety filter; report violations and infeasibility separately.
5. **Robustness:** inject observation noise, delayed estimates, and random obstacle layouts; use Modal for independent Monte-Carlo trials.
6. **Deployment boundary:** reproduce selected scenarios in ROS 2 or MuJoCo, then document the simulation-to-deployment gap.

## Honesty criteria

- The current geometric filter is not a formal safety guarantee.
- A CBF-QP result is only guaranteed under its stated model, sampling, and feasibility assumptions.
- RLS identifies a deliberately simple scalar gain first; a richer model must earn its complexity through better held-out performance.
- All comparison controllers receive the same reference, limits, noise, seeds, and compute budget.

## Reading anchors

- MIT Underactuated Robotics: [system identification](https://underactuated.mit.edu/sysid.html), [LQR](https://underactuated.mit.edu/lqr.html), [robust control](https://underactuated.mit.edu/robust.html).
- Ames et al., *Control Barrier Function Based Quadratic Programs for Safety Critical Systems*, IEEE TAC (2017), [DOI](https://doi.org/10.1109/TAC.2016.2638961).
- Rawlings, Mayne, Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed. (2017).
- The recent work *Safe Navigation Under State Uncertainty: Online Adaptation for Robust Control Barrier Functions* (IEEE RA-L, 2026) is a useful comparison target, not a result to copy: [DOI](https://doi.org/10.1109/LRA.2026.3653366).
