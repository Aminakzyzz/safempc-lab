# Validation record

## Local validation

Executed in the isolated project environment:

```text
ruff check .     PASS
pytest -q        PASS (4 tests)
python -m safempc_lab.simulation     PASS
```

The unit tests verify integration for a straight-line motion, enforcement of actuator limits, angle wrapping, and MPC output limits.

## What the first benchmark means

The current figure-eight run is a smoke test, not a performance claim. The initial geometric obstacle filter still allows negative clearance in the default scenario. This is a useful result: it identifies the concrete next technical milestone, a control-barrier-function quadratic-program layer or explicit obstacle constraints in the MPC formulation.

Do not describe this release as collision-safe or deployment-ready. The repository is designed to make shortcomings visible and testable rather than to conceal them.

## External validation still needed

- Run the Modal parameter sweep after authenticating the Modal profile.
- Add benchmark plots and a fixed experiment report.
- Verify the public GitHub Actions workflow after the repository push succeeds.
- Evaluate the controller in a higher-fidelity ROS 2 simulation before making any hardware claim.
