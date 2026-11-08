from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin

import numpy as np

from .models import DifferentialDrive, RobotState, wrap_angle


@dataclass(frozen=True)
class PurePursuit:
    speed: float = 0.45
    lookahead: float = 0.35

    def command(self, state: RobotState, target: np.ndarray, robot: DifferentialDrive) -> np.ndarray:
        dx, dy = target[0] - state.x, target[1] - state.y
        alpha = wrap_angle(atan2(dy, dx) - state.yaw)
        omega = 2.0 * self.speed * sin(alpha) / self.lookahead
        return robot.clip_input(np.array([self.speed, omega]))


@dataclass(frozen=True)
class LQRTracker:
    q: np.ndarray
    r: np.ndarray
    speed: float = 0.45
    iterations: int = 100

    @classmethod
    def default(cls) -> LQRTracker:
        return cls(q=np.diag([8.0, 8.0, 3.0]), r=np.diag([0.5, 0.3]))

    def gain(self, robot: DifferentialDrive, state: RobotState) -> np.ndarray:
        a, b = robot.linearize(state, self.speed)
        p = self.q.copy()
        for _ in range(self.iterations):
            s = self.r + b.T @ p @ b
            k = np.linalg.solve(s, b.T @ p @ a)
            p_next = self.q + a.T @ p @ (a - b @ k)
            if np.max(np.abs(p_next - p)) < 1e-10:
                break
            p = p_next
        return k

    def command(self, state: RobotState, reference: np.ndarray, robot: DifferentialDrive) -> np.ndarray:
        error = np.array([state.x - reference[0], state.y - reference[1], wrap_angle(state.yaw - reference[2])])
        u_nominal = np.array([self.speed, reference[3]])
        return robot.clip_input(u_nominal - self.gain(robot, state) @ error)


@dataclass(frozen=True)
class MPCTracker:
    """Box-constrained linear MPC solved by projected gradient descent.

    The robot is linearised at every sample. Circular-obstacle avoidance remains
    a separate safety layer until the CBF-QP milestone is implemented.
    """

    q: np.ndarray
    r: np.ndarray
    horizon: int = 12
    speed: float = 0.45
    iterations: int = 80

    @classmethod
    def default(cls) -> MPCTracker:
        return cls(q=np.diag([10.0, 10.0, 4.0]), r=np.diag([0.4, 0.25]))

    def command(self, state: RobotState, reference: np.ndarray, robot: DifferentialDrive) -> np.ndarray:
        a, b = robot.linearize(state, self.speed)
        n, m, h = 3, 2, self.horizon
        sx = np.zeros((n * h, n))
        su = np.zeros((n * h, m * h))
        for row in range(h):
            sx[row * n : (row + 1) * n] = np.linalg.matrix_power(a, row + 1)
            for col in range(row + 1):
                su[row * n : (row + 1) * n, col * m : (col + 1) * m] = np.linalg.matrix_power(a, row - col) @ b
        qbar = np.kron(np.eye(h), self.q)
        rbar = np.kron(np.eye(h), self.r)
        hessian = su.T @ qbar @ su + rbar
        error = np.array([state.x - reference[0], state.y - reference[1], wrap_angle(state.yaw - reference[2])])
        gradient_offset = su.T @ qbar @ sx @ error
        nominal = np.tile(np.array([self.speed, reference[3]]), h)
        lower = np.tile(np.array([-robot.v_limit, -robot.omega_limit]), h) - nominal
        upper = np.tile(np.array([robot.v_limit, robot.omega_limit]), h) - nominal
        delta = np.zeros(m * h)
        step = 1.0 / max(float(np.linalg.eigvalsh(hessian).max()), 1e-9)
        for _ in range(self.iterations):
            delta = np.clip(delta - step * (hessian @ delta + gradient_offset), lower, upper)
        return robot.clip_input(nominal[:m] + delta[:m])


def safety_filter(control: np.ndarray, state: RobotState, obstacle: np.ndarray, safe_radius: float, robot: DifferentialDrive) -> np.ndarray:
    """Conservative geometric filter that removes inward forward velocity."""
    radial = np.array([state.x, state.y]) - obstacle[:2]
    distance = float(np.linalg.norm(radial))
    if distance >= safe_radius or distance < 1e-9:
        return robot.clip_input(control)
    heading = np.array([cos(state.yaw), sin(state.yaw)])
    inward = float(np.dot(heading, radial / distance))
    if inward < 0.0:
        control = np.array([min(control[0], 0.0), control[1]])
    return robot.clip_input(control)
