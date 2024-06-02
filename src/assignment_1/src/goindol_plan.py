from __future__ import annotations

from goindol_bezier import BezierCurve
from goindol_types import *
from goindol_util import calc_next_point, yaw_add

class PathPlan:
    def __init__(self, init_status: CarStatus, dest_status: CarStatus):
        """
        start: 시작시 차량의 상태
        end: 도착시 차량의 상태 (차량이 도착지점에서 바라보는 방향이 end.yaw가 됩니다.)
        """
        self.init_status = init_status
        self.dest_status = dest_status
        self.path = []
        self.make_plan()

    def __add__(self, other: PathPlan) -> PathPlan:
        self.path = self.path + other.path
        return self

    def make_plan(self):
        print('start making plan!')
        # 베지어 곡선의 폭
        bezier_offset = 320

        # 양 끝점
        p0 = self.init_status.to_point()
        p1 = self.dest_status.to_point()
        # 보간된 점
        p2 = calc_next_point(p0, self.init_status.yaw, bezier_offset)
        p3 = calc_next_point(p1, yaw_add(self.dest_status.yaw, 180), bezier_offset)

        bezier_curve = BezierCurve(p0, p2, p3, p1)
        self.path = bezier_curve.curve

        print(f' {len(self.path)} points, of length {bezier_curve.curve_len}')
        print('done making plan.')

    def add_point(self, point: Point):
        self.path.append(point)

    def extract_x(self):
        """경로의 x좌표를 반환합니다."""
        return self.path[:, 0]

    def extract_y(self):
        """경로의 y좌표를 반환합니다."""
        return self.path[:, 1]
