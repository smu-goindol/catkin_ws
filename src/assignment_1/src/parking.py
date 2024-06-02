#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 고인돌
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import collections
import sys
import rospy # type: ignore
from xycar_msgs.msg import xycar_motor
import math_ext

#=============================================
# 모터 토픽을 발행할 것임을 선언
#=============================================
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
queue = collections.deque() # 가야 할 길 들을 점들로 나타냄


#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global queue
    queue = make_plan(sx, sy, syaw, P_ENTRY[0], P_ENTRY[1], calc_yaw(P_END, P_ENTRY))
    X = []
    Y = []
    for x, y in queue:
        X.append(x)
        Y.append(y)
    return X, Y

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen: pygame.Surface, x, y, yaw, velocity, max_acceleration, dt):
    global queue

    if not queue:
        drive(angle=0, speed=0)
        return

    s_pos = (x,y)
    e_pos = queue[0]

    if len(queue) < 10:
        epsilon = 4
    else:
        epsilon = 100

    # 다음 점까지 점진적으로 수렴시켜보자
    max_velocity = velocity + max_acceleration * dt

    next_yaw = calc_yaw(s_pos, e_pos)
    next_dist = calc_dist(s_pos, e_pos)

    next_angle = ((yaw - next_yaw + 360) % 360) - 180
    next_speed = min(max_velocity, next_dist / dt)

    pygame.draw.line(screen, (128, 128, 0), s_pos, e_pos)
    sys.stdout.write(f'{next_angle:03.2f}, {next_dist:06.2f}\n')

    if abs(next_angle) > 90 and len(queue) >= 2:
        # 너무 벗어났으면 다시 경로를 갱신
        queue = make_plan(*s_pos, yaw, *P_ENTRY, calc_yaw(P_END, P_ENTRY))

    drive(angle=next_angle, speed=next_speed)

    if next_dist < epsilon:
        # 충분히 가까우면 도달한 것으로 판정
        queue.popleft()


def make_plan(sx: float, sy: float, syaw: float, ex: float, ey: float, eyaw: float) -> collections.deque:
    offset = 320
    P0 = [sx, sy] # 출발 지점
    P1 = [sx + offset*math.cos(syaw), sy + offset*math.sin(syaw)] # 출발 시 바라보는 방향으로 갈 수 있도록 베지어 좌표 1 설정
    P2 = [ex + offset*math.cos(eyaw), ey + offset*math.sin(eyaw)] # 도착 시 바라보는 방향으로 갈 수 있도록 베지어 좌표 1 설정
    P3 = [ex, ey] # 도착 지점
    P = np.array([P0, P1, P2, P3])
    curve_len = math_ext.bezier_curve_length(P, 200)
    curves = math_ext.bezier_curve(P, max(curve_len//2, 2))
    queue = collections.deque(curves)
    while queue and calc_dist(queue[0], (sx, sy)) < 10:
        queue.popleft()
    return queue


def derivate(p0: tuple, p1: tuple) -> tuple:
    return p1[1]-p0[1], p1[0]-p0[0]


def calc_yaw(p0: tuple, p1: tuple) -> float:
    dy, dx = derivate(p0, p1)
    return -math.degrees(np.arctan2(dy, dx))


def calc_dist(p0: tuple, p1: tuple) -> float:
    dy, dx = derivate(p0, p1)
    return (dy**2 + dx**2)**0.5


def parking_line(x: float) -> float:
    return -x + 1198
