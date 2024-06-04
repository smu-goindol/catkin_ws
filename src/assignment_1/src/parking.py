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
import rospy # type: ignore
from xycar_msgs.msg import xycar_motor

from goindol import *


#=============================================
# 모터 토픽을 발행할 것임을 선언
#=============================================
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
DEST_STATE = CarState(x=P_ENTRY[0], y=P_ENTRY[1], yaw=calc_yaw(P_ENTRY, P_END))
strategy: Strategy

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
    global strategy
    state = CarState(sx, sy, syaw, 0, max_acceleration, dt).rotate(90)
    strategy = ForwardStrategy.get_strategy(state, DEST_STATE)
    return strategy.get_path().get_x_points(), strategy.get_path().get_y_points()

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen: pygame.Surface, x, y, yaw, velocity, max_acceleration, dt):
    curr_state = CarState(x, y, yaw, velocity, max_acceleration, dt).flip_vertical()
    next_state = strategy.predict(curr_state)
    diff = next_state - curr_state
    drive(angle=diff.angle(), speed=next_state.velocity)

    # 시각화
    curr_state.draw(screen)
    next_state.draw(screen)
    strategy.get_path().draw(screen)
