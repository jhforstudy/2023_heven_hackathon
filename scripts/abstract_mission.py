#!/usr/bin/env python3

import rospy
import math
import numpy as np

from ackermann_msgs.msg import AckermannDrive
from racecar_simulator.msg import HeadPose, CenterPose
from goal import Goal
# Abstract class
from abc import *

# 차량의 현재 상태를 저장하는 클래스
class Carstatus:
    def __init__(self) -> None:
        rospy.Subscriber("/car_center", CenterPose, self.position_callback, queue_size=1)
        rospy.Subscriber("/car_head", HeadPose, self.head_callback, queue_size=1)
        rospy.Subscriber("/drive", AckermannDrive, self.speed_callback, queue_size=1)

        # Information of a car
        self.position = np.array([0,0])
        self.position_yaw = 0
        self.position_unit_vector = np.array([0,0])
        self.head = np.array([0,0])
        self.head_yaw = 0
        self.speed = 0
    
    def position_callback(self, data=CenterPose):
        self.position = np.array([data.pose[0], data.pose[1]])
        self.position_yaw = data.pose[2] # yaw angle (deg)
        self.position_unit_vector = np.array([math.cos(math.radians(data.pose[2])), math.sin(math.radians(data.pose[2]))]) # yaw unit vector

    def head_callback(self, data=HeadPose):
        self.head = np.array([data.pose[0], data.pose[1]])
        self.head_yaw = data.pose[2] # yaw angle (deg)
        self.head_unit_vector = np.array([math.cos(math.radians(data.pose[2])), math.sin(math.radians(data.pose[2]))]) # yaw unit vector

    def speed_callback(self, data=AckermannDrive):
        self.speed = data.speed


# 각 미션별 클래스 형태
# 반드시 아래 형태를 상속받아서 생성해야 함.
class Mission(metaclass=ABCMeta):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def main(self, goal=Goal, car=Carstatus):
        # 이 곳에서는, 각 미션의 메인 함수가 구현됨.
        pass

    @abstractmethod
    def is_in_mission(self, goal=Goal, car=Carstatus):
        # 이 곳에서는, 현재 차량이 각 미션안에 있는지 확인하는 함수가 구현됨.
        pass

    @abstractmethod
    def init_values(self):
        # 이 곳에서는, 사용하는 모든 변수들을 초기화 함.
        pass