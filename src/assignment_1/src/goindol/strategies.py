from collections import deque
from typing import *

from .path import *
from .point import *
from .state import CarState


class Strategy:
    def __init__(self, src_state: CarState, dst_state: CarState) -> None:
        """Initialize the strategy with the source and destination states."""
        ...

    def get_src_state(self) -> CarState:
        """Return the source state of the strategy."""
        ...

    def get_dst_state(self) -> CarState:
        """Return the destination state of the strategy."""
        ...

    def get_path(self) -> Path:
        """Return the path of the strategy."""
        ...

    def predict(self, state: CarState) -> CarState:
        """Predict the next state of the car."""
        ...


class AbstractStrategy(Strategy):
    def __init__(self, src_state: CarState, dst_state: CarState, path: Path) -> None:
        self._src_state = src_state
        self._dst_state = dst_state
        self._path = path
        print(f'path initialized @ {src_state.point()}')

    def get_src_state(self) -> CarState:
        return self._src_state

    def get_dst_state(self) -> CarState:
        return self._dst_state

    def get_path(self) -> Path:
        return self._path

    def predict(self, state: CarState) -> CarState:
        raise NotImplementedError


class ForwardStrategy(AbstractStrategy):
    @classmethod
    def get_strategy(cls, src_state: CarState, dst_state: CarState) -> Strategy:
        return cls(src_state, dst_state)

    def __init__(self, src_state: CarState, dst_state: CarState) -> None:
        path = BezierCurve(
            src_state.point(),
            src_state.move(320).point(),
            dst_state.rotate(180).move(320).point(),
            dst_state.point(),
            samples=200,
        )
        super().__init__(src_state, dst_state, path)
        self._queue = deque(path.get_points())

    def predict(self, state: CarState) -> CarState:
        EPSILON = 20

        # TODO: 주차라인 끝에 충분히 가까워졌을때 주차라인 끝에 멈추도록 합니다.

        # 다음으로 방문할 좌표 선정
        while self._queue:
            next_point = self._queue.popleft()
            if calc_distance(state.point(), next_point) > EPSILON:
                self._queue.appendleft(next_point)
                break

        # 다음으로 방문할 좌표가 없다면, 목적지로 이동
        if not self._queue:
            return self.get_dst_state()

        # 다음으로 방문할 좌표가 차량의 방향과 일치하지 않는다면, 다시 경로를 설정
        if not state.is_heading_to(next_point):
            self.__init__(state, self.get_dst_state())
            return state

        next_state = CarState(
            x=next_point[0],
            y=next_point[1],
            yaw=calc_yaw(state.point(), next_point),
            velocity=state.velocity,
            max_acceleration=state.max_acceleration,
            dt=state.dt,
        )

        return next_state
