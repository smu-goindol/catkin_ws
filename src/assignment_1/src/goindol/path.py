from typing import *
from math import comb

import numpy as np
import pygame

from .point import Point


class Path:
    ## 곡선의 길이 반환.
    def __len__(self) -> int:
        ...
        
    ## 곡선을 순회하는 iterator 반환.
    def __iter__(self) -> Iterable:
        ...
        
    ## 곡선의 모든 점들을 numpy 배열로 반환.
    def get_points(self) -> np.ndarray:        
        ...
        
    ## 곡선의 x좌표를 리스트로 반환.
    def get_x_points(self) -> List[float]:
        ...
        
    ## 곡선의 y좌표를 리스트로 반환.
    def get_y_points(self) -> List[float]:       
        ...
        
    ## pygame 화면에 곡선 그리기.
    def draw(self, surface: pygame.Surface) -> None:
        ...


class AbstractPath(Path):
    ## 주어진 점들을 사용하여 경로 초기화.
    def __init__(self, points: np.ndarray):
        self._points = points # 점들을 인스턴스 변수로 저장.

    ## 연결된 점들의 거리의 합인 경로의 길이 반환.
    def __len__(self) -> int:
        return np.sum(np.sqrt(np.sum(np.diff(self.get_points(), axis=0)**2, axis=1))) # 점들 사이의 거리 합을 반환.

    ## 경로를 순회하며 각 반복에서 x,y좌표 반환.
    def __iter__(self) -> Iterable:
        for x, y in self.get_points(): # 각 점의 x,y좌표를 순회.
            yield x, y

    ## 경로를 구성하는 점들의 배열 반환.
    def get_points(self) -> np.ndarray:
        return self._points # 점들의 배열을 반환.

    ## 경로의 x좌표 반환.
    def get_x_points(self) -> List[float]:
        return self._points[:, 0].tolist() # x좌표를 리스트로 반환.

    ## 경로의 y좌표 반환.
    def get_y_points(self) -> List[float]:
        return self._points[:, 1].tolist() # y좌표를 리스트로 반환.

    ## pygame 화면에서 경로는 초록색 선으로 표시됨.
    def draw(self, surface: pygame.Surface) -> None:
        LINE_COLOR = (0, 255, 0) # 선 색상을 초록색으로 설정.
        pygame.draw.lines(surface, LINE_COLOR, False, self.get_points(), width=2) # pygame 화면에 경로 그리기.


class BezierCurve(AbstractPath):
    def __init__(self, *points: Point, samples: int = 1000):
        P = np.array(points) # 주어진 점들을 numpy 배열로 반환.
        T = np.linspace(0, 1, samples) # 0부터 1까지의 sample points 생성.
        N = P.shape[0] - 1  # 제어점들의 개수에서 1을 뺀 값을 베지어 곡선의 차수로 계산. 
        M = T.shape[0]  # 매개변수 T의 값의 개수 계산. 
        bezier_curve = np.zeros((M, 2)) #베지어 곡선 상의 점들을 저장할 배열 생성. 

        #==========================================
        # 베지어 곡선 계산. 
        # 제어점의 개수에 따라 반복하여 제어점에 대한 가중치를 계산한 후 새로운 축에 대한 배열 반환.
        #==========================================
        for i in range(N + 1): # 제어점의 개수만큼 반복.
            term = (comb(N, i) * (1 - T) ** (N - i) * T ** i)[:, np.newaxis] # 베지어 곡선의 가중치를 계산.
            bezier_curve += term * P[i] # 가중치를 각 제어점에 곱하여 곡선을 구성하는 점들을 계산.
        super().__init__(bezier_curve) # 베지어 곡선을 나타내는 점들의 배열로 초기화.
