from collections import deque
import math
import typing

from goindol_plan import PathPlan
from goindol_types import *
from goindol_util import *


class CarDriver:
    plan: PathPlan
    queue: deque

    def __init__(self, parking_entry: Point, parking_end: Point):
        self.parking_entry = parking_entry
        self.parking_end = parking_end
        self.parking_yaw = calc_yaw(parking_entry, parking_end)
        self.dist_epsilon = 10 # 이 거리 안에 들어오면 다음 점으로 넘어감

    def _parking_end_status(self) -> CarStatus:
        """주차라인 끝에 있어야 할 차량의 모습을 반환합니다."""
        return CarStatus.stop_at(self.parking_end, self.parking_yaw)

    def _parking_entry_status(self) -> CarStatus:
        """주차라인 시작에 있어야 할 차량의 모습을 반환합니다."""
        return CarStatus.stop_at(self.parking_entry, self.parking_yaw)

    def make_plan(self, init_status: CarStatus):
        """현재 위치에서 주차라인 끝까지의 경로를 계획합니다."""
        plan = self.plan_forward(init_status)
        # TODO: 후진으로도 갈 수 있는 경로를 추가해보세요.
        self.plan = plan
        self.queue = deque(plan.path)

    def plan_forward(self, init_status: CarStatus):
        """현재 위치에서 주차라인 끝까지의 전진 경로를 계획합니다."""
        plan_1 = PathPlan(init_status, self._parking_entry_status())
        plan_2 = PathPlan(self._parking_entry_status(), self._parking_end_status())
        return plan_1 + plan_2

    ########################################

    def drive(self, curr_status: CarStatus) -> CarStatus:
        """차량의 다음 상태를 계산합니다.

        우리의 차를 주행시키기 위해 구현해야 할 부분은 여기입니다.
        """
        if self._is_near_end(curr_status):
            # 주차라인 끝에 충분히 가까워졌을 때 주차라인 끝에 멈추도록 합니다.
            return self._parking_end_status()

        next_status = self._linear_status_prediction(curr_status)

        if abs(next_status.calc_angle()) > 90:
            # 90도 이상 핸들을 꺾을 수 없으므로, 다시 경로를 갱신할 준비.
            self.make_plan(curr_status)

        return next_status # 다음 점으로 가기위한 상태를 반환한다.

    ########################################

    def _is_near_end(self, status: CarStatus) -> bool:
        """현재 위치가 주차라인 끝에 충분히 가까운지 확인합니다."""
        return calc_dist(status.to_point(), self.parking_end) < self.dist_epsilon

    def _next_point(self, status: CarStatus) -> Point:
        """다음 점을 반환합니다."""
        self._remove_close_points(status)
        if self.queue:
            return self.queue[0]
        return self.parking_end

    def _remove_close_points(self, status: CarStatus):
        """현재 위치에서 충분히 가까운 점들을 제거합니다."""
        while self.queue and calc_dist(self.queue[0], status.to_point()) < self.dist_epsilon:
            self.queue.popleft()

    def _linear_status_prediction(self, curr_status: CarStatus) -> CarStatus:
        """현재 상태에서 다음 상태를 선형으로 예측합니다."""
        curr_point = curr_status.to_point()
        next_point = self._next_point(curr_status)
        next_yaw = calc_yaw(curr_point, next_point)

        dist = calc_dist(curr_point, next_point)
        speed = min(dist / curr_status.dt, curr_status.max_velocity)

        return CarStatus(
            x=next_point.x,
            y=next_point.y,
            yaw=next_yaw,
            velocity=speed,
            max_acceleration=curr_status.max_acceleration,
            dt=curr_status.dt,
            prev_state=curr_status,
        )
