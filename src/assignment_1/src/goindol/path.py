from typing import *
from math import comb

import numpy as np
import pygame

from .point import Point


class Path:
    def __len__(self) -> int:
        """Return the length of the curve."""
        ...

    def __iter__(self) -> Iterable:
        """Return an iterator over the points of the curve."""
        ...

    def get_points(self) -> np.ndarray:
        """Return the points of the curve."""
        ...

    def get_x_points(self) -> List[float]:
        """Return the x-coordinates of the points of the curve."""
        ...

    def get_y_points(self) -> List[float]:
        """Return the y-coordinates of the points of the curve."""
        ...

    def draw(self, surface: pygame.Surface) -> None:
        """Draw the curve on the surface."""
        ...


class AbstractPath(Path):
    def __init__(self, points: np.ndarray):
        self._points = points

    def __len__(self) -> int:
        return np.sum(np.sqrt(np.sum(np.diff(self.get_points(), axis=0)**2, axis=1)))

    def __iter__(self) -> Iterable:
        for x, y in self.get_points():
            yield x, y

    def get_points(self) -> np.ndarray:
        return self._points

    def get_x_points(self) -> List[float]:
        return self._points[:, 0].tolist()

    def get_y_points(self) -> List[float]:
        return self._points[:, 1].tolist()

    def draw(self, surface: pygame.Surface) -> None:
        LINE_COLOR = (0, 255, 0)
        pygame.draw.lines(surface, LINE_COLOR, False, self.get_points(), width=2)


class BezierCurve(AbstractPath):
    def __init__(self, *points: Point, samples: int = 1000):
        P = np.array(points)
        T = np.linspace(0, 1, samples) # sample points
        N = P.shape[0] - 1  # Degree of the Bezier curve (number of control points - 1)
        M = T.shape[0]  # Number of parameter values
        # Create an array to hold the points on the Bezier curve
        bezier_curve = np.zeros((M, 2))
        # Calculate the Bezier curve points
        for i in range(N + 1):
            term = (comb(N, i) * (1 - T) ** (N - i) * T ** i)[:, np.newaxis]
            bezier_curve += term * P[i]
        super().__init__(bezier_curve)
