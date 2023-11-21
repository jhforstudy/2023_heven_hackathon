#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import time

from database import Database
from ackermann_msgs.msg import AckermannDrive
from parameter_list import Param
from goal import PARKING_SPOT, STOP_LINE, DELIV_PICKUP, DELIV_DROPOFF

param = Param()


class Brain():
    def __init__(self, db=Database, map_number=int):
        self.db = db
        '''
        이 곳은 클래스가 처음 선언될 때 최초로 실행되는 함수입니다.

        여기에는, main 함수가 끝난 후에도 계속 사용할 변수들을 저장할 수 있습니다.
        변수를 사용하기 위해서는, 변수를 최초에 초기화해야 합니다.
        (아래 예시)
        self.prev_angle = 0
        '''
    
    def main(self):
        '''
        # 1. 라이다 데이터를 받아오는 방법
        lidar_data = self.db.lidar_data

        # 2. 차량 위치를 받아오는 방법
        pose_data = self.db.pose_data [x, y, yaw(degree)]

        # 3. 현재 미션을 확인하는 방법
        curr_mission = self.db.current_mission
        (주차 : PARKING_SPOT)
        (정지선 : STOP_LINE)
        (배달 출발 : DELIV_PICKUP)
        (배달 도착 : DELIV_DROPOFF)

        # 각 미션에 필요한 데이터를 받아오는 방법
        # 4-1. 주차 공간에 대한 정보를 얻어오는 방법
        for element in self.db.parking_list:
            park_x, park_y, park_yaw, park_num = element
            (x : x 방향 상대 좌표 차이)
            (y : y 방향 상대 좌표 차이)
            (yaw : 주차 공간 방향 차이)
            (num : 주차 공간의 번호)
        # 4-2. 주차해야 하는 공간
        target_park = self.db.target_park

        # 5. 정지선에 대한 정보를 얻어오는 방법 
        traffic_light = self.db.traffic_light
        (None, STOP, LEFT, RIGHT, STRAIGHT)
        remaining_time = self.db.traffic_remaining_time
        
        # 6-1. 배달에 대한 정보를 얻어오는 방법
        pickup_x, pickup_y, pickup_yaw, pickup_num = self.db.pickup_list
        (x : x 방향 상대 좌표 차이)
        (y : y 방향 상대 좌표 차이)
        (yaw : 배달 출발 위치의 방향 차이)
        (num : 배달해야 하는 공간의 번호)
        # 6-2. 배달 완료해야 하는 공간
        for element in self.db.dropoff_list:
            deliv_x, deliv_y, deliv_yaw, deliv_num = element
            (x : x 방향 상대 좌표 차이)
            (y : y 방향 상대 좌표 차이)
            (yaw : 배달 공간 방향 차이)
            (num : 배달 도착 공간의 번호)

        # 위에서 선언한 클래스 변수에 값을 대입하는 방법
        (아래 예시 : 현재 루프에서 사용한 각도를 클래스 내에 저장함.)
        self.prev_angle = angle

        # 최종적으로 차량의 각도와 속도 결정
        return angle, speed
        '''
        lidar_data = self.db.lidar_data
        pose_data = self.db.pose_data
        traffic_light = self.db.traffic_light
        remaining_time = self.db.traffic_remaining_time
        pickup_list = self.db.pickup_list
        dropoff_list = self.db.dropoff_list

        # Determine the angle & speed
        # angle : 20=LEFT ~ -20=RIGHT (degree)
        # speed : 0 ~ 4 (m/s)
        angle = 0
        speed = 0
        # Return angle & speed
        return angle, speed


if __name__ == "__main__":
    db = Database(lidar=True)
    test_brain = Brain(db)
    rate = rospy.Rate(param.thread_rate)
    control_pub = rospy.Publisher('/drive', AckermannDrive, queue_size=1)
    while not rospy.is_shutdown():
        car_angle, car_speed = test_brain.main()
        motor_msg = AckermannDrive()
        motor_msg.steering_angle = car_angle
        motor_msg.speed = car_speed
        motor_msg.steering_angle_velocity = param.car_angular_velocity
        motor_msg.acceleration = param.car_acceleration
        motor_msg.jerk = param.car_jerk
        control_pub.publish(motor_msg)
        rate.sleep()