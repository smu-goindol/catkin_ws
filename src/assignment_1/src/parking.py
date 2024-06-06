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

from goindol import * # 같이 제출한 goindol 모듈 내의 모든 객체를 현재 네임스페이스로 가져옴.


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
# 주차라인 진입 시점의 위치와 주차라인 끝 지점을 향하는 방향을 나타내는 Carstate 객체를 이용.
# 차량의 목적지를 나타내는 데 쓰임.
#=============================================
DEST_STATE = CarState(x=P_ENTRY[0], y=P_ENTRY[1], yaw=calc_yaw(P_ENTRY, P_END)) 

## 전략 초기화시, 시작 상태와 목표 상태 전달, 주행 경로 반환, 현재 상태를 기반으로 다음 상태 예측.
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

    ## Carstate 를 사용하여 시작 상태 생성 후 방향 90도 회전.
    ## planning 함수와 tracking 함수에서의 x축(y축)이 90도만큼 차이 발생.
    state = CarState(sx, sy, syaw, 0, max_acceleration, dt).rotate(90)
    ## 시작 상태와 DEST_STATE를 기반으로 전략 결정. 결정된 경로를 x,y좌표로 반환.
    strategy = ForwardStrategy.get_strategy(state, DEST_STATE)
    ## 전략의 주행 경로를 가져와 반환, x좌표와 y좌표를 반환.
    return strategy.get_path().get_x_points(), strategy.get_path().get_y_points()

#=============================================기.
    #=============================================
    curr_state.draw(screen)
    next_state.draw(screen)
    strategy.get_path().draw(screen)
