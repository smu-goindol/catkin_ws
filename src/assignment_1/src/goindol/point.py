from typing import Tuple
import math


Point = Tuple[float, float]


def calc_distance(p1: Point, p2: Point) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def calc_yaw(p1: Point, p2: Point) -> float:
    """두 점 사이의 yaw를 계산합니다."""
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
