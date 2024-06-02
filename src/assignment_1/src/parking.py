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

from goindol_driver import CarDriver
from goindol_types import CarStatus
from goindol_util import yaw_fix

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
car_driver = CarDriver(P_ENTRY, P_END)

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
    init_status = CarStatus(sx, sy, yaw_fix(syaw), 0, max_acceleration, dt)
    car_driver.make_plan(init_status)
    return car_driver.plan.extract_x(), car_driver.plan.extract_y()

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen: pygame.Surface, x, y, yaw, velocity, max_acceleration, dt):
    curr_status = CarStatus(x, y, yaw, velocity)
    next_status = car_driver.drive(curr_status, max_acceleration, dt)
    color = (128, 128, 0)
    pygame.draw.line(screen, color, curr_status.to_point(), next_status.to_point(), width=2)
    drive(next_status.calc_angle(), next_status.velocity)
