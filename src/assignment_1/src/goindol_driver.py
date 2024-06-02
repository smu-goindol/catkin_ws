from collections import deque
import math

from goindol_plan import PathPlan
from goindol_types import *


def calc_dist(p1: Point, p2: Point) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def calc_yaw(p1: Point, p2: Point) -> float:
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))

def yaw2angle(yaw: float) -> float:
    return (yaw + 90) % 360

def angle2yaw(angle: float) -> float:
    return (angle - 90) % 360


class CarDriver:
    plan: PathPlan
    queue: deque

    def __init__(self, car: Car, parking_entry: Point, parking_end: Point):
        self.end_pos = parking_entry
        self.end_yaw = calc_yaw(parking_end, parking_entry)
        self.dist_epsilon = 10 # 이 거리 안에 들어오면 다음 점으로 넘어감
        self.plan_for(car)

    def plan_for(self, car: Car):
        """차량의 경로를 다시 계획합니다."""
        start_yaw = (car.yaw + 90) % 360
        self.plan = PathPlan(car.point, start_yaw, self.end_pos, self.end_yaw)
        self.queue = deque(self.plan.path)
        print("goindol car driver noticed planning has done.")

    def _prune_close_points(self, pos: Point):
        """Queue에서 충분히 가까운 점들을 제거합니다."""
        while self.queue and calc_dist(self.queue[0], pos) < self.dist_epsilon:
            self.queue.popleft()

    def drive(self, status: CarCurrentStatus) -> CarNextStatus:
        """차량의 다음 상태를 계산합니다.

        우리의 차를 주행시키기 위해 구현해야 할 부분은 여기입니다.
        """

        # 충분히 가까운 점은 제거
        self._prune_close_points(status.car.point)

        if len(self.queue) == 0:
            # 경로가 끝났을 때
            return CarNextStatus(*self.plan.end_pos)

        # 다음 점까지 점진적으로 수렴시켜보자
        next_status = CarNextStatus(*self.queue[0])
        while True:
            spos = status.car.point
            epos = self.queue[0]

            dist = calc_dist(spos, epos)
            yaw = calc_yaw(spos, epos)
            speed = dist / status.dt

            next_status.angle = yaw2angle(status.car.yaw - yaw)
            next_status.speed = min(status.max_velocity, speed)

            print(f'planned to go {next_status.angle:.2f} degrees, {next_status.speed:.2f} speed.')

            if abs(next_status.angle) >= 90:
                # 90도 이상 핸들을 꺾을 수 없으므로, 다시 경로를 갱신.
                self.plan_for(status.car)
            else:
                # 이대로면 충분히 진행 가능하다.
                break

        # 다음 점으로 가기위한 상태를 반환한다.
        return next_status