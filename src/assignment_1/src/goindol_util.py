import math
from goindol_types import *


def calc_dist(p1: Point, p2: Point) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def calc_yaw(p1: Point, p2: Point) -> float:
    """두 점 사이의 yaw를 계산합니다."""
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))


def calc_next_point(point: Point, yaw: float, dist: float) -> Point:
    """현재 위치에서 yaw방향으로 dist만큼 이동한 위치를 계산합니다."""
    return point[0] + dist * math.cos(math.radians(yaw)), point[1] + dist * math.sin(math.radians(yaw))


def yaw_fix(yaw: float) -> float:
    """planning에서 주어지는 Yaw는 y-positive 방향이 0도이다.
    이를 x-positive 방향이 0도인 yaw로 변환한다.

    (그래야 삼각함수를 사용할 때 편하다)
    """
    yaw = (yaw + 90) % 360
    if yaw > 180:
        yaw -= 360
    return yaw


def yaw_normalize(yaw: float) -> float:
    """[0,360) 범위의 yaw값을 [-180, 180)범위로 정규화한다."""
    return (yaw + 180) % 360 - 180


def yaw_add(yaw1: float, yaw2: float) -> float:
    """yaw1과 yaw2를 더한 값을 [0, 360) 범위로 정규화하여 반환한다."""
    return (yaw1 + yaw2) % 360
