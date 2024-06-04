from typing import *

from .path import *
from .point import Point
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

    def get_queue(self) -> PathQueue:
        """Return the queue of points in the path."""
        ...

    def has_arrived(self, state: CarState) -> bool:
        """Check if the car has arrived at the destination."""
        ...

    def predict(self, state: CarState) -> CarState:
        """Predict the next state of the car."""
        ...


class AbstractStrategy(Strategy):
    def __init__(self, src_state: CarState, dst_state: CarState, path: Path) -> None:
        self._src_state = src_state
        self._dst_state = dst_state
        self._path = path
        self._queue = PathQueue(path)

    def get_src_state(self) -> CarState:
        return self._src_state

    def get_dst_state(self) -> CarState:
        return self._dst_state

    def get_path(self) -> Path:
        return self._path

    def get_queue(self) -> PathQueue:
        return self._queue

    def has_arrived(self, state: CarState, point: Point = None, epsilon=10) -> bool:
        if point is None:
            dst_state = self._dst_state
        else:
            dst_state = CarState.from_point(point)
        return (dst_state - state).distance() < epsilon

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
        )
        return super().__init__(src_state, dst_state, path)

    def predict(self, state: CarState) -> CarState:
        # 주차라인 끝에 충분히 가까워졌을때 주차라인 끝에 멈추도록 합니다.
        if self.has_arrived(state):
            return self.get_dst_state()
        if (next_point := self.get_queue().front(lambda p: not self.has_arrived(state, p))) is None:
            return self.get_dst_state()
        # 다음 점으로 가기위한 상태를 반환한다.
        next_status = CarState.from_point(next_point)
        diff = next_status - state
        # 90도 이상 핸들을 꺾을 수 없으므로, 다시 경로를 갱신할 준비.
        if abs(diff.angle()) > 90:
            self.__init__(state, self.get_dst_state())
        return next_status
