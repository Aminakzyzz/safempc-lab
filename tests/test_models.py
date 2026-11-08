import numpy as np

from safempc_lab.models import DifferentialDrive, RobotState, wrap_angle


def test_straight_line_step() -> None:
    robot = DifferentialDrive(dt=0.1)
    result = robot.step(RobotState(0.0, 0.0, 0.0), np.array([0.5, 0.0]))
    assert np.isclose(result.x, 0.05)
    assert np.isclose(result.y, 0.0)


def test_input_limits_are_enforced() -> None:
    robot = DifferentialDrive(v_limit=1.0, omega_limit=2.0)
    assert np.allclose(robot.clip_input(np.array([3.0, -4.0])), [1.0, -2.0])


def test_angles_are_wrapped() -> None:
    assert np.isclose(wrap_angle(3.0 * np.pi), -np.pi)
