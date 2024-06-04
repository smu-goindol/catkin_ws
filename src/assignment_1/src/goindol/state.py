from __future__ import annotations

from dataclasses import dataclass
import math

from point import Point


@dataclass
class CarState:
    """차량의 상태를 나타내는 클래스

    yaw: x-positive 방향이 0도, y-positive 방향이 90도
    """
    x: float = 0
    y: float = 0
    yaw: float = 0
    velocity: float = 0
    max_acceleration: float = 0
    dt: float = 0

    @classmethod
    def from_point(cls, point: Point, yaw: float = 0, velocity: float = 0, max_acceleration: float = 0, dt: float = 0) -> CarState:
        return cls(x=point[0], y=point[1], yaw=yaw, velocity=velocity, max_acceleration=max_acceleration, dt=dt)

    def __sub__(self, other: CarState) -> CarStateDiff:
        return CarStateDiff(
            dx=self.x - other.x,
            dy=self.y - other.y,
            dyaw=self.yaw - other.yaw
        )

    def point(self) -> Point:
        return self.x, self.y

    def max_velocity(self) -> float:
        return self.velocity + self.max_acceleration * self.dt

    def move(self, dist: float = None) -> CarState:
        if dist is None:
            dist = self.velocity * self.dt
        return CarState(
            x=self.x + dist * math.cos(math.radians(self.yaw)),
            y=self.y + dist * math.sin(math.radians(self.yaw)),
            yaw=self.yaw,
            velocity=self.velocity,
            max_acceleration=self.max_acceleration,
            dt=self.dt
        )

    def rotate(self, angle: float) -> CarState:
        return CarState(
            x=self.x,
            y=self.y,
            yaw=(self.yaw + angle) % 360,
            velocity=self.velocity,
            max_acceleration=self.max_acceleration,
            dt=self.dt
        )


@dataclass
class CarStateDiff:
    """차량의 상태 변화량을 나타내는 클래스"""

    dx: float
    dy: float
    dyaw: float

    def distance(self) -> float:
        return (self.dx ** 2 + self.dy ** 2) ** 0.5

    def speed(self, dt: float) -> float:
        return self.distance / dt

    def angle(self) -> float:
        """[-180, 180) 범위의 yaw값 변화량을 반환합니다."""
        return (self.dyaw + 180) % 360 - 180
