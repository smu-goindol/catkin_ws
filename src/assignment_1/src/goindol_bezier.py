import math
import typing

import numpy as np

from goindol_types import Point


class BezierCurve:
    def __init__(self, *control_points: Point, samples=1000):
        self.control_points = [*control_points]
        self.samples = samples
        self.curve = []
        self.curve_len = 0.0
        self._init()

    def _init(self):
        self.curve = self._make_curve(self.control_points, self.samples)
        self.curve_len = self._calc_len(self.curve)

    def _make_curve(self, points: typing.List[Point], samples: int) -> np.ndarray:
        P = np.array(points)
        T = np.linspace(0, 1, samples)
        N = P.shape[0] - 1  # Degree of the Bezier curve (number of control points - 1)
        M = T.shape[0]  # Number of parameter values
        # Create an array to hold the points on the Bezier curve
        curve_points = np.zeros((M, 2))
        # Calculate the Bezier curve points
        for i in range(N + 1):
            binomial_coefficient = math.comb(N, i)
            term = (binomial_coefficient * (1 - T) ** (N - i) * T ** i)[:, np.newaxis]
            curve_points += term * points[i]
        return curve_points

    def _calc_len(self, points: np.ndarray) -> float:
        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        return np.sum(distances)