from __future__ import annotations

import dataclasses
import typing


Point = typing.Tuple[float, float]


class PointMixin:
    def to_point(self) -> Point:
        return self.x, self.y


@dataclasses.dataclass
class CarStatus(PointMixin):
    """차량의 상태를 나타내는 클래스

    yaw: x-positive 방향이 0도, y-positive 방향이 90도
    """

    @classmethod
    def stop_at(cls, point: Point, yaw: float = 0) -> CarStatus:
        """특정 위치에서 정지한 차량 상태를 반환합니다."""
        return cls(x=point[0], y=point[1], yaw=yaw)

    x: float = 0
    y: float = 0
    yaw: float = 0
    velocity: float = 0
    max_acceleration: float = 0
    dt: float = 0

    prev_state: CarStatus = None

    @property
    def max_velocity(self) -> float:
        return self.velocity + self.max_acceleration * self.dt

    def calc_angle(self) -> float:
        from goindol_util import yaw_normalize
        assert self.prev_state is not None
        return yaw_normalize(self.yaw - self.prev_state.yaw)
