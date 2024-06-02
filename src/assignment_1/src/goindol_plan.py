import math

from goindol_bezier import BezierCurve
from goindol_types import *


class PathPlan:
    def __init__(self, start_pos: Point, start_yaw: float, end_pos: Point, end_yaw: float):
        self.start_pos = start_pos
        self.start_yaw = start_yaw
        self.end_pos = end_pos
        self.end_yaw = end_yaw
        self.path = []
        self.make_plan()

    def make_plan(self):
        print('start making plan!')
        bezier_offset = 320 # 베지어 곡선의 폭
        p0 = self.start_pos
        p1 = self._make_point(self.start_pos, self.start_yaw, bezier_offset)
        p2 = self._make_point(self.end_pos, self.end_yaw, bezier_offset)
        p3 = self.end_pos
        bezier_curve = BezierCurve(p0, p1, p2, p3)
        self.path = bezier_curve.curve
        print('done making plan.')
        print(f' {len(self.path)} points, of length {bezier_curve.curve_len}')

    def _make_point(self, point: Point, yaw: float, offset: float):
        return point[0] + offset*math.cos(math.radians(yaw)), point[1] + offset*math.sin(math.radians(yaw))
