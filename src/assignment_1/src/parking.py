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
rx, ry = [300, 350, 400, 450], [300, 350, 400, 450]
n = 0 # 점의 개수
i = 0 # 지금 몇 번째 점에 있는지


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
    global rx, ry, n, i
    P0 = [sx, sy] # 출발 지점
    P1 = [sx + math.cos(syaw), sy + math.sin(syaw)] # 출발 시 바라보는 방향으로 갈 수 있도록 베지어 좌표 1 설정
    P2 = [*P_ENTRY] # 도착 지점 시작
    P3 = [*P_END] # 도착 지점 종료
    P = np.array([P0, P1, P2, P3])
    n = 1000
    i = 0
    curves = math_ext.bezier_curve(P, n)
    rx, ry = curves[:,0], curves[:,1]
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen: pygame.Surface, x, y, yaw, velocity, max_acceleration, dt):
    pos = (x,y)

    if distance(pos, P_END) <5:
        drive(angle=0, speed=0)
        print("stop1")
        return

    if not queue:
        drive(angle=0, speed=0)
        print("stop2")
        return

    if len(queue) < 10:
        epsilon = 4
        magic = 100 / max(len(queue), 1)
    else:
        epsilon = 100
        magic = 0

    # 다음 점까지 점진적으로 수렴시켜보자
    dy, dx = derivate(queue[0], pos)
    max_velocity = velocity + max_acceleration * dt

    next_yaw = -math.degrees(np.arctan2(dy, dx))
    next_angle = ((yaw - next_yaw + 360) % 360) - 180
    next_dist = (dy**2 + dx**2)**0.5
    next_speed = (100 / (100 + magic)) * min(max_velocity, next_dist / dt)

    pygame.draw.line(screen, (128, 128, 0), pos, queue[0])
    sys.stdout.write(f'{next_angle:03.2f}, {next_dist:06.2f}\n')

    if abs(next_angle) > 90:
        planning(x, y, yaw, max_acceleration, dt)
        # # 어떻게든 유턴 시켜야 함
        # sign = next_angle/abs(next_angle) 
        # next_angle = sign * 90 
        # next_speed = (100 + next_dist) / 100 # 유턴은 천천히

    drive(angle=next_angle, speed=next_speed)

    if next_dist < epsilon:
        # 충분히 가까우면 도달한 것으로 판정
        queue.popleft()


def derivate(p0: tuple, p1: tuple) -> tuple:
    return p1[1]-p0[1], p1[0]-p0[0]


def parking_line(x: float) -> float:
    return -x + 1198

def distance(p0: tuple, p1: tuple) -> float:
    return ((p0[0] - p1[0])**2 +(p0[1]-p1[1])**2)**0.5