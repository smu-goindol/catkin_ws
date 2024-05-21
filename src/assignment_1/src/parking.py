#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 슈퍼카
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
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
    n = 20
    i = 0
    curves = math_ext.bezier_curve(P, np.linspace(0, 1, n))
    rx, ry = curves[:,0], curves[:,1]
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry, n, i
    print(f'{i+1}-th iteration')
    angle = yaw
    speed = 0
    if (d := np.linalg.norm([rx[i]-x, ry[i]-y])) > 1:
        yaw_hat = np.arctan((ry[i]-y)/(rx[i]-x))
        angle = yaw_hat - yaw
        speed = d
        print(f'    angle: {angle:04.2f} speed: {speed}')
    else:
        i += 1
    drive(angle, speed)
