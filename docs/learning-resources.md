# Learning roadmap and resources

Use this document with the [interactive roadmap](learning-roadmap.html). The order is intentional: build a model, understand feedback, add prediction, then add adaptation and safety.

## Core resources

- MIT Underactuated Robotics: [introduction](https://underactuated.mit.edu/intro.html), [LQR](https://underactuated.mit.edu/lqr.html), [system identification](https://underactuated.mit.edu/sysid.html), and [robust control](https://underactuated.mit.edu/robust.html).
- MIT OpenCourseWare: [Underactuated Robotics video lectures](https://ocw.mit.edu/courses/6-832-underactuated-robotics-spring-2009/video_galleries/video-lectures/).
- [Python Control Systems Library tutorial](https://python-control.readthedocs.io/en/latest/examples/python-control_tutorial.html).
- [CasADi documentation](https://web.casadi.org/docs/), after the MPC formulation is understood.
- Ames et al., [Control Barrier Function Based Quadratic Programs for Safety Critical Systems](https://doi.org/10.1109/TAC.2016.2638961), before implementing the safety layer.

## Working rule

For each phase, record the question, assumptions, parameters, metrics, result, and limitation in `latex/progress.md`. Add a source to `latex/references.bib` before making a technical claim in the report.
