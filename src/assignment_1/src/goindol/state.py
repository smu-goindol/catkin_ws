from __future__ import annotations

from dataclasses import dataclass
import math

import pygame

from .point import *


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
        """반시계방향으로 회전"""
        return CarState(
            x=self.x,
            y=self.y,
            yaw=(self.yaw + angle) % 360,
            velocity=self.velocity,
            max_acceleration=self.max_acceleration,
            dt=self.dt
        )

    def flip_horizontal(self) -> CarState:
        return CarState(
            x=self.x,
            y=self.y,
            yaw=(180 - self.yaw) % 360,
            velocity=self.velocity,
            max_acceleration=self.max_acceleration,
            dt=self.dt
        )

    def flip_vertical(self) -> CarState:
        return CarState(
            x=self.x,
            y=self.y,
            yaw=(360 - self.yaw) % 360,
            velocity=self.velocity,
            max_acceleration=self.max_acceleration,
            dt=self.dt
        )

    def yaw_heading_to(self, point: Point) -> float:
        """현재 위치에서 point로 향하는 yaw값을 반환합니다."""
        return calc_yaw(self.point(), point)

    def cos_heading_to(self, point: Point) -> float:
        """현재 위치에서 point로 향하는 방향의 cos값을 반환합니다."""
        return math.cos(math.radians(self.yaw_heading_to(point) - self.yaw))

    def is_heading_to(self, point: Point) -> bool:
        """현재 위치에서 point로 향하는지 여부를 반환합니다."""
        return self.cos_heading_to(point) >= 0.5 # 60 degrees

    def draw(self, surface: pygame.Surface):
        COLOR_X_AXIS = (0,0,255)
        COLOR_Y_AXIS = (255,0,0)
        LENGTH_X_AXIS = 25
        LENGTH_Y_AXIS = 75
        pygame.draw.circle(surface, COLOR_X_AXIS, self.rotate(0).move(LENGTH_X_AXIS).point(), radius=5)
        pygame.draw.line(surface, COLOR_X_AXIS, self.point(), self.rotate(0).move(LENGTH_X_AXIS).point(), width=2)
        pygame.draw.line(surface, COLOR_X_AXIS, self.point(), self.rotate(180).move(LENGTH_X_AXIS).point(), width=2)
        pygame.draw.circle(surface, COLOR_Y_AXIS, self.rotate(90).move(LENGTH_Y_AXIS).point(), radius=5)
        pygame.draw.line(surface, COLOR_Y_AXIS, self.point(), self.rotate(90).move(LENGTH_Y_AXIS).point(), width=2)
        pygame.draw.line(surface, COLOR_Y_AXIS, self.point(), self.rotate(270).move(LENGTH_Y_AXIS).point(), width=2)


@dataclass
class CarStateDiff:
    """차량의 상태 변화량을 나타내는 클래스"""

    dx: float
    dy: float
    dyaw: float

    def distance(self) -> float:
        return (self.dx ** 2 + self.dy ** 2) ** 0.5

    def speed(self, dt: float) -> float:
        return self.distance() / dt

    def angle(self) -> float:
        """(-180, 180] 범위의 yaw값 변화량을 반환합니다.

        시계방향으로 회전할 경우 양수의 값을 가집니다.
        """
        return -((self.dyaw + 180) % 360 - 180)
