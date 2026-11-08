import numpy as np

from safempc_lab.identification import RecursiveSpeedEstimator


def test_recursive_estimator_recovers_constant_actuator_gain() -> None:
    estimator = RecursiveSpeedEstimator()
    true_gain = 0.72
    commands = np.linspace(0.15, 0.9, 40)
    for command in commands:
        estimator.update(command, true_gain * command)
    assert np.isclose(estimator.gain, true_gain, atol=0.02)


def test_estimator_ignores_uninformative_zero_command() -> None:
    estimator = RecursiveSpeedEstimator(gain=0.8)
    assert estimator.update(0.0, 0.1) == 0.8
