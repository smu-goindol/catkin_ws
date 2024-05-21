import numpy as np
import math


def bezier_curve(P, num_points=1000):
    """
    Calculate the Bezier curve for given control points P and parameters T.
    P: Control points, a (N, 2) numpy array
    num_points: Number of points to sample on the curve
    Returns:
    curve_points: A (M, 2) numpy array of points on the Bezier curve
    """
    T = np.linspace(0, 1, num_points)
    N = P.shape[0] - 1  # Degree of the Bezier curve (number of control points - 1)
    M = T.shape[0]  # Number of parameter values

    # Create an array to hold the points on the Bezier curve
    curve_points = np.zeros((M, 2))

    # Calculate the Bezier curve points
    for i in range(N + 1):
        binomial_coefficient = math.comb(N, i)
        term = (binomial_coefficient * (1 - T) ** (N - i) * T ** i)[:, np.newaxis]
        curve_points += term * P[i]

    return curve_points


def bezier_curve_length(P, num_points=1000):
    """
    Approximate the length of a Bezier curve given control points P.
    P: Control points, a (N, 2) numpy array
    num_points: Number of points to sample on the curve
    Returns:
    length: Approximate length of the Bezier curve
    """
    # Sample points on the Bezier curve
    T = np.linspace(0, 1, num_points)
    curve_points = bezier_curve(P, T)

    # Compute the distance between successive points
    distances = np.sqrt(np.sum(np.diff(curve_points, axis=0)**2, axis=1))

    # Sum the distances to get the total length
    length = np.sum(distances)

    return length