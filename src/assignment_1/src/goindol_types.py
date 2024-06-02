import dataclasses
import typing


Point = typing.Tuple[float, float]


@dataclasses.dataclass
class Car:
    x: float = 0
    y: float = 0
    yaw: float = 0

    @property
    def point(self) -> Point:
        return self.x, self.y



@dataclasses.dataclass
class CarCurrentStatus:
    car: Car = dataclasses.field(default_factory=Car)
    velocity: float = 0
    max_acceleration: float = 0
    dt: float = 0

    @property
    def max_velocity(self) -> float:
        return self.velocity + self.max_acceleration * self.dt


@dataclasses.dataclass
class CarNextStatus:
    car: Car = dataclasses.field(default_factory=Car)
    angle: float = 0
    speed: float = 0
