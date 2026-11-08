from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .controllers import LQRTracker, MPCTracker, PurePursuit, safety_filter
from .models import DifferentialDrive, RobotState, wrap_angle


@dataclass(frozen=True)
class Metrics:
    rms_position_error: float
    peak_heading_error: float
    min_clearance: float
    control_effort: float


def figure_eight(samples: int = 500) -> np.ndarray:
    t = np.linspace(0.0, 2.0 * np.pi, samples)
    x, y = 1.2 * np.sin(t), 0.7 * np.sin(t) * np.cos(t)
    yaw = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    omega = np.gradient(yaw) / 0.05
    return np.column_stack([x, y, yaw, omega])


def simulate(controller: str = "lqr", steps: int = 400) -> Metrics:
    robot = DifferentialDrive()
    path = figure_eight(steps + 1)
    obstacle = np.array([0.35, 0.05])
    state = RobotState(-0.1, -0.2, 0.0)
    lqr, mpc, pursuit = LQRTracker.default(), MPCTracker.default(), PurePursuit()
    errors, headings, clearances, efforts = [], [], [], []
    for reference in path[:steps]:
        if controller == "lqr":
            control = lqr.command(state, reference, robot)
        elif controller == "mpc":
            control = mpc.command(state, reference, robot)
        else:
            control = pursuit.command(state, reference, robot)
        control = safety_filter(control, state, obstacle, safe_radius=0.25, robot=robot)
        state = robot.step(state, control)
        errors.append((state.x - reference[0]) ** 2 + (state.y - reference[1]) ** 2)
        headings.append(abs(wrap_angle(state.yaw - reference[2])))
        clearances.append(np.hypot(state.x - obstacle[0], state.y - obstacle[1]) - 0.25)
        efforts.append(float(control @ control))
    return Metrics(float(np.sqrt(np.mean(errors))), float(max(headings)), float(min(clearances)), float(np.mean(efforts)))


if __name__ == "__main__":
    for controller in ("pursuit", "lqr", "mpc"):
        print(controller, simulate(controller))
