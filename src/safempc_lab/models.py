from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin
from typing import Literal

import numpy as np


@dataclass(frozen=True)
class RobotState:
    """Planar pose of a unicycle-equivalent differential-drive robot."""

    x: float
    y: float
    yaw: float

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw], dtype=float)


def wrap_angle(angle: float) -> float:
    """Map an angle to [-pi, pi)."""
    return (angle + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class DifferentialDrive:
    dt: float = 0.05
    v_limit: float = 1.0
    omega_limit: float = 2.0

    def clip_input(self, control: np.ndarray) -> np.ndarray:
        return np.array([
            np.clip(control[0], -self.v_limit, self.v_limit),
            np.clip(control[1], -self.omega_limit, self.omega_limit),
        ])

    def derivative(self, state: RobotState, control: np.ndarray) -> np.ndarray:
        v, omega = self.clip_input(np.asarray(control, dtype=float))
        return np.array([v * cos(state.yaw), v * sin(state.yaw), omega])

    def step(self, state: RobotState, control: np.ndarray, method: Literal["euler", "rk4"] = "rk4") -> RobotState:
        u = self.clip_input(np.asarray(control, dtype=float))
        y0 = state.as_array()
        if method == "euler":
            y1 = y0 + self.dt * self.derivative(state, u)
        elif method == "rk4":
            def f(y: np.ndarray) -> np.ndarray:
                return self.derivative(RobotState(*y), u)
            k1 = f(y0)
            k2 = f(y0 + 0.5 * self.dt * k1)
            k3 = f(y0 + 0.5 * self.dt * k2)
            k4 = f(y0 + self.dt * k3)
            y1 = y0 + self.dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        else:
            raise ValueError(f"Unknown integration method: {method}")
        return RobotState(float(y1[0]), float(y1[1]), wrap_angle(float(y1[2])))

    def linearize(self, state: RobotState, speed: float) -> tuple[np.ndarray, np.ndarray]:
        """First-order discrete linearization around state and forward speed."""
        a = np.eye(3)
        a[0, 2] = -self.dt * speed * sin(state.yaw)
        a[1, 2] = self.dt * speed * cos(state.yaw)
        b = np.array([
            [self.dt * cos(state.yaw), 0.0],
            [self.dt * sin(state.yaw), 0.0],
            [0.0, self.dt],
        ])
        return a, b
