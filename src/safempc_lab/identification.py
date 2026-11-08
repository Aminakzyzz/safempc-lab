"""Online identification primitives for adaptive control experiments."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RecursiveSpeedEstimator:
    """Scalar RLS estimate of actual speed = gain * commanded speed.

    This deliberately narrow model isolates the first research question: can an
    online estimate recover wheel-slip or actuator-gain mismatch quickly enough
    to improve a controller's prediction model? It is not a full vehicle model.
    """

    gain: float = 1.0
    covariance: float = 20.0
    forgetting_factor: float = 0.995
    min_gain: float = 0.2
    max_gain: float = 1.8

    def update(self, commanded_speed: float, observed_speed: float) -> float:
        """Assimilate one speed observation and return the bounded gain estimate."""
        phi = float(commanded_speed)
        if abs(phi) < 1e-8:
            return self.gain
        denominator = self.forgetting_factor + phi * self.covariance * phi
        kalman_gain = self.covariance * phi / denominator
        residual = float(observed_speed) - phi * self.gain
        self.gain = float(np.clip(self.gain + kalman_gain * residual, self.min_gain, self.max_gain))
        self.covariance = float((self.covariance - kalman_gain * phi * self.covariance) / self.forgetting_factor)
        return self.gain

    @property
    def standard_deviation(self) -> float:
        """One-sigma uncertainty proxy for the estimated scalar gain."""
        return float(np.sqrt(max(self.covariance, 0.0)))

    def predicted_speed(self, commanded_speed: float) -> float:
        return self.gain * float(commanded_speed)
