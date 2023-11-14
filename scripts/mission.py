#!/usr/bin/env python3

import rospy
import time
import math
import numpy as np
import threading
import tkinter as tk
import cv2

# ROS messages
from geometry_msgs.msg import Pose
from racecar_simulator.msg import DropoffInfoList, ParkingInfoList, Complete
from std_msgs.msg import String
from random import randint
from goal import *
from abstract_mission import Carstatus, Mission
# Mission classes
from mission_parking import ParkingMission
from mission_stopline import StopLineMission
from mission_deliv_pickup import DelivPickupMission
from mission_deliv_dropoff import DelivDropoffMission
from mission_traffic import TrafficMission
# Parameter
from parameter_list import Param
# Type checking
from typing import Dict

param = Param()


# 미션 전체가 실행되는 동안 작동하는 클래스
class Mainmission:
    def __init__(self):
        # ROS settings
        rospy.init_node('mission', anonymous=True)
        self.dropoff = rospy.Publisher("/dropoff_info_list", DropoffInfoList, queue_size=1)
        self.parking = rospy.Publisher("/parking_info_list", ParkingInfoList, queue_size=1)
        self.mission_pub = rospy.Publisher('/cur_mission', String, queue_size=1)
        self.map_number = rospy.get_param('~map_number')

        # 차량이 미션존에 포함되어 있는 미션의 개수
        self.num_in_missions = 0

        # 주차, 배달 정보를 publish하기 위한 리스트
        self.parking_info_list = []
        self.deliv_dropoff_info_list = []

        # 정지선 미션의 개수를 세기 위함
        self.num_stop_line = 0

        # Car status class
        self.car = Carstatus()

        # 맵 별로 미션 클래스 생성 (각각의 미션 클래스에는 미션 메인 함수, 미션 도달 여부 확인하는 함수가 포함됨)
        # =====================================================================================================================================================
        self.mission_list = dict()
        self.mission_list: Dict[int, Mission]   # 이 딕셔너리는 반드시 key = (int), value = (Mission) 형태를 가져야 함
        if self.map_number == 1:
            self.mission_list[PARKING_SPOT]  = ParkingMission(self.map_number)
            self.mission_list[STOP_LINE]     = StopLineMission(self.map_number)
            self.mission_list[DELIV_PICKUP]  = DelivPickupMission(self.map_number)
            self.mission_list[DELIV_DROPOFF] = DelivDropoffMission(self.map_number)
        
        elif self.map_number == 2:
            self.mission_list[PARKING_SPOT]  = ParkingMission(self.map_number)
            self.mission_list[STOP_LINE]     = StopLineMission(self.map_number)
            self.mission_list[DELIV_PICKUP]  = DelivPickupMission(self.map_number)
            self.mission_list[DELIV_DROPOFF] = DelivDropoffMission(self.map_number)
        
        elif self.map_number == 3:
            self.mission_list[PARKING_SPOT]  = ParkingMission(self.map_number)
            self.mission_list[STOP_LINE]     = StopLineMission(self.map_number)
            self.mission_list[DELIV_PICKUP]  = DelivPickupMission(self.map_number)
            self.mission_list[DELIV_DROPOFF] = DelivDropoffMission(self.map_number)

        elif self.map_number == 4:
            pass

        elif self.map_number == 5:
            pass
        
        else:
            rospy.logerr("Incorrect map_number when making a Mission class!")
        # =====================================================================================================================================================

        # 맵 별로 goal 리스트 생성 (하나의 맵에 존재하는 여러 개의 goal list를 생성함)
        # =====================================================================================================================================================
        # 종합미션 (jinhee)
        if self.map_number == 1:
            self.goal_list = [
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_1_PARKING_LOT_X_1, param.MAP_1_PARKING_LOT_Y_1)), yaw=param.MAP_1_PARKING_LOT_YAW_1, number=1),
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_1_PARKING_LOT_X_2, param.MAP_1_PARKING_LOT_Y_2)), yaw=param.MAP_1_PARKING_LOT_YAW_2, number=2),
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_1_PARKING_LOT_X_3, param.MAP_1_PARKING_LOT_Y_3)), yaw=param.MAP_1_PARKING_LOT_YAW_3, number=3),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_1_STOP_LINE_X_1, param.MAP_1_STOP_LINE_Y_1)), yaw=param.MAP_1_STOP_LINE_YAW_1, number=1, traffic=TRAFFIC_RIGHT),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_1_STOP_LINE_X_2, param.MAP_1_STOP_LINE_Y_2)), yaw=param.MAP_1_STOP_LINE_YAW_2, number=2, traffic=TRAFFIC_LEFT),
                            Goal(mode=DELIV_PICKUP, position=np.array((param.MAP_1_DELIV_PICKUP_X, param.MAP_1_DELIV_PICKUP_Y)), yaw=param.MAP_1_DELIV_PICKUP_YAW, number=param.MAP_1_PICKUP_DROPOFF_AREA),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_1_DELIV_DROPOFF_X_1, param.MAP_1_DELIV_DROPOFF_Y_1)), yaw=param.MAP_1_DELIV_DROPOFF_YAW_1, number=1),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_1_DELIV_DROPOFF_X_2, param.MAP_1_DELIV_DROPOFF_Y_2)), yaw=param.MAP_1_DELIV_DROPOFF_YAW_2, number=2),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_1_DELIV_DROPOFF_X_3, param.MAP_1_DELIV_DROPOFF_Y_3)), yaw=param.MAP_1_DELIV_DROPOFF_YAW_3, number=3)
                            ]

        # 굴곡 미션
        elif self.map_number == 2:
            self.goal_list = [
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_2_PARKING_LOT_X_1, param.MAP_2_PARKING_LOT_Y_1)), yaw=param.MAP_2_PARKING_LOT_YAW_1, number=1, tolerance=[2.80, 1.00]),
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_2_PARKING_LOT_X_2, param.MAP_2_PARKING_LOT_Y_2)), yaw=param.MAP_2_PARKING_LOT_YAW_2, number=2, tolerance=[2.80, 1.00]),
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_2_PARKING_LOT_X_3, param.MAP_2_PARKING_LOT_Y_3)), yaw=param.MAP_2_PARKING_LOT_YAW_3, number=3, tolerance=[2.80, 1.00]),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_2_STOP_LINE_X_1, param.MAP_2_STOP_LINE_Y_1)), yaw=param.MAP_2_STOP_LINE_YAW_1, number=1, traffic=TRAFFIC_STRAIGHT),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_2_STOP_LINE_X_2, param.MAP_2_STOP_LINE_Y_2)), yaw=param.MAP_2_STOP_LINE_YAW_2, number=2, traffic=TRAFFIC_LEFT),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_2_STOP_LINE_X_3, param.MAP_2_STOP_LINE_Y_3)), yaw=param.MAP_2_STOP_LINE_YAW_3, number=3, traffic=TRAFFIC_RIGHT),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_2_STOP_LINE_X_4, param.MAP_2_STOP_LINE_Y_4)), yaw=param.MAP_2_STOP_LINE_YAW_4, number=4, traffic=TRAFFIC_STRAIGHT),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_2_STOP_LINE_X_5, param.MAP_2_STOP_LINE_Y_5)), yaw=param.MAP_2_STOP_LINE_YAW_5, number=5, traffic=TRAFFIC_STRAIGHT),
                            Goal(mode=DELIV_PICKUP, position=np.array((param.MAP_2_DELIV_PICKUP_X, param.MAP_2_DELIV_PICKUP_Y)), yaw=param.MAP_2_DELIV_PICKUP_YAW, number=param.MAP_2_PICKUP_DROPOFF_AREA, tolerance=[3.00, 1.00]),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_2_DELIV_DROPOFF_X_1, param.MAP_2_DELIV_DROPOFF_Y_1)), yaw=param.MAP_2_DELIV_DROPOFF_YAW_1, number=1),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_2_DELIV_DROPOFF_X_2, param.MAP_2_DELIV_DROPOFF_Y_2)), yaw=param.MAP_2_DELIV_DROPOFF_YAW_2, number=2),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_2_DELIV_DROPOFF_X_3, param.MAP_2_DELIV_DROPOFF_Y_3)), yaw=param.MAP_2_DELIV_DROPOFF_YAW_3, number=3)
                            ]
        
        # 실제 미션
        elif self.map_number == 3:
            self.goal_list = [
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_3_PARKING_LOT_X_1, param.MAP_3_PARKING_LOT_Y_1)), yaw=param.MAP_3_PARKING_LOT_YAW_1, number=1),
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_3_PARKING_LOT_X_2, param.MAP_3_PARKING_LOT_Y_2)), yaw=param.MAP_3_PARKING_LOT_YAW_2, number=2),
                            Goal(mode=PARKING_SPOT, position=np.array((param.MAP_3_PARKING_LOT_X_3, param.MAP_3_PARKING_LOT_Y_3)), yaw=param.MAP_3_PARKING_LOT_YAW_3, number=3),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_3_STOP_LINE_X_1, param.MAP_3_STOP_LINE_Y_1)), yaw=param.MAP_3_STOP_LINE_YAW_1, number=1, traffic=TRAFFIC_STRAIGHT),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_3_STOP_LINE_X_2, param.MAP_3_STOP_LINE_Y_2)), yaw=param.MAP_3_STOP_LINE_YAW_2, number=2, traffic=TRAFFIC_RIGHT),
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_3_STOP_LINE_X_3, param.MAP_3_STOP_LINE_Y_3)), yaw=param.MAP_3_STOP_LINE_YAW_3, number=3, traffic=TRAFFIC_RIGHT),
                            Goal(mode=DELIV_PICKUP, position=np.array((param.MAP_3_DELIV_PICKUP_X, param.MAP_3_DELIV_PICKUP_Y)), yaw=param.MAP_3_DELIV_PICKUP_YAW, number=param.MAP_3_PICKUP_DROPOFF_AREA),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_3_DELIV_DROPOFF_X_1, param.MAP_3_DELIV_DROPOFF_Y_1)), yaw=param.MAP_3_DELIV_DROPOFF_YAW_1, number=1),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_3_DELIV_DROPOFF_X_2, param.MAP_3_DELIV_DROPOFF_Y_2)), yaw=param.MAP_3_DELIV_DROPOFF_YAW_2, number=2),
                            Goal(mode=DELIV_DROPOFF, position=np.array((param.MAP_3_DELIV_DROPOFF_X_3, param.MAP_3_DELIV_DROPOFF_Y_3)), yaw=param.MAP_3_DELIV_DROPOFF_YAW_3, number=3)
                            ]
        
        elif self.map_number == 4:
            pass
        
        elif self.map_number == 5:
            self.goal_list = []

        else:
            rospy.logerr("Incorrect map_number when making a Goal list!")

        # =====================================================================================================================================================

        # Thread for Init button
        button_mission_thread = threading.Thread(target=self.init_mission_button)
        button_mission_thread.daemon = True
        button_mission_thread.start()

    # Main loop
    ##########################################################################################################
    def main(self):
        # 미션 확인 알고리즘의 메인 루프.
        # 현재 차량이 어느 미션 존에 있는지 확인하고, 어떤 미션 안에 있다면 메인 함수 실행
        self.mission_check()

        # 현재 미션 상태를 점수판에 표시함
        self.visualize_mission_state()
    ##########################################################################################################

    # Compare CAR position with GOAL position
    # jinhee
    def check(self, goal=Goal):
        # 이 곳에는 현재 차량이 해당 미션의 안에 있는지 확인함
        # 미션 공간에 있다면, 해당하는 미션의 이름과 정보를 출력하고, 메인 함수를 실행함.

        try:
            # 0. 주차
            # 주차의 경우, 주차 미션 내에 있게 되면 ParkingInfo 메시지 하나가 리턴됨.
            if goal.mode == PARKING_SPOT:
                parking_info = self.mission_list[PARKING_SPOT].is_in_mission(goal, self.car)
                if parking_info is not None:
                    self.mission_list[PARKING_SPOT].main(goal, self.car)

                    # Publish parking info
                    self.parking_info_list.append(parking_info)

                    self.num_in_missions += 1

            # 1. 정지선
            elif goal.mode == STOP_LINE:
                if self.mission_list[STOP_LINE].is_in_mission(goal, self.car):
                    # Publish mission name
                    self.mission_pub.publish("stop_line")
                    self.mission_list[STOP_LINE].main(goal, self.car)
                    self.num_stop_line += 1

                    self.num_in_missions += 1
                    
            # 2. 배달 수령
            elif goal.mode == DELIV_PICKUP:
                if self.mission_list[DELIV_PICKUP].is_in_mission(goal, self.car):
                    # Publish mission name
                    self.mission_pub.publish("delivery_pickup")
                    self.mission_list[DELIV_PICKUP].main(goal, self.car)

                    self.num_in_missions += 1

            # 3. 배달
            # 배달의 경우, 주차 미션 내에 있게 되면 DropoffInfo 메시지 하나가 리턴됨.
            elif goal.mode == DELIV_DROPOFF:
                dropoff_info = self.mission_list[DELIV_DROPOFF].is_in_mission(goal, self.car)
                if dropoff_info is not None:
                    self.mission_list[DELIV_DROPOFF].main(goal, self.car)

                    # Append dropoff info
                    self.deliv_dropoff_info_list.append(dropoff_info)

                    self.num_in_missions += 1
                
            # 4. 교차로
            elif goal.mode == TRAFFIC:
                if self.mission_list[TRAFFIC].is_in_mission(goal, self.car):
                    # Publish mission name
                    self.mission_pub.publish("traffic")
                    self.mission_list[TRAFFIC].main(goal, self.car)

                    self.num_in_missions += 1

            # 그 외 미션들...

        except:
            # 예외 : 만약 현재 맵에서 해당 미션이 존재하지 않으면, pass
            return False

    # Check if a car is in Mission area
    def mission_check(self):
        # 주차, 배달 미션에 대한 정보 Topic list 초기화
        self.parking_info_list.clear()
        self.deliv_dropoff_info_list.clear()
        self.num_stop_line = 0
        self.num_in_missions = 0

        # goal_list를 탐색하며, 해당 goal 주변에 있을 경우 필요한 정보 출력
        # 또한, 해당 미션 함수 실행
        for curr_goal in self.goal_list:
            self.check(curr_goal)

        # 미션 함수들이 끝나고, 주차와 배달의 경우 여러개의 정보를 담아서 publish
        # Publish ParkingInfo topic & mission name
        if len(self.parking_info_list) != 0:
            self.mission_pub.publish("parking")
            self.parking.publish(self.parking_info_list)
        else:
            # 주차 공간 없으면, 빈 리스트 출력
            self.parking.publish([])
        # Publish DropoffInfo topic & mission name
        if len(self.deliv_dropoff_info_list) != 0:
            self.mission_pub.publish("delivery_dropoff")
            self.dropoff.publish(self.deliv_dropoff_info_list)
        else:
            # 배달 공간 없으면, 빈 리스트 출력
            self.dropoff.publish([])
        # 정지선 미션이 없을 경우, None 출력
        if self.num_stop_line == 0:
            self.mission_list[STOP_LINE].publish_null_traffic()

        # 미션이 아예 없을 경우, 빈 미션 메시지를 출력
        if self.num_in_missions == 0:
            self.mission_pub.publish("")

    def init_mission_button(self):
        # Refresh button
        window = tk.Tk()
        window.geometry('70x50+0+550')
        button = tk.Button(window, text='Mission\nInit', command=self.initialize)
        button.config(width=5, height=2)
        button.place(x=0, y=0)
        window.mainloop()

    def initialize(self):
        # Initial Pose
        pose_pub = rospy.Publisher('pose', Pose, queue_size=1)
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        pose_pub.publish(pose)

        # 버튼을 눌렀을 때, 모든 미션에 들어있는 변수들을 초기화 함
        for mission in self.mission_list.values():
            mission.init_values()

        # spawn index 초기화
        complete = rospy.Publisher("/complete", Complete, queue_size=1)
        complete_msg = Complete()
        complete_msg.complete = False
        complete.publish(complete_msg)

    def visualize_mission_state(self):
        # Show the status of missions
        img = np.full(shape=(400,400,3),fill_value=0,dtype=np.uint8)

        if self.map_number == 1:
            text_1 = "Parking : "

            if self.mission_list[PARKING_SPOT].num_success_parking[param.MAP_1_PARKING_AREA - 1] == 1:
                text_1 += "Success"

            text_2 = "Stop #1 : "
            text_3 = "Stop #2 : "
            text_4 = "PickUp : "
            text_5 = "DropOff : "

            if self.mission_list[STOP_LINE].num_success_stop[0] == 1:
                text_2 += "Success"
            if self.mission_list[STOP_LINE].num_success_stop[1] == 1:
                text_3 += "Success"
            if self.mission_list[DELIV_PICKUP].num_success_pickup == 1:
                text_4 += "Success"
            if self.mission_list[DELIV_DROPOFF].num_success_dropoff[param.MAP_1_PICKUP_DROPOFF_AREA - 1] == 1:
                text_5 += "Success"

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.putText(img, text_1, (0, 30), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_2, (0, 70), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_3, (0, 110), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_4, (0, 150), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_5, (0, 190), font, 1, (255, 255, 255), 2)
            pass

        elif self.map_number == 2:
            text_1 = "Parking : "

            if self.mission_list[PARKING_SPOT].num_success_parking[param.MAP_2_PARKING_AREA - 1] == 1:
                text_1 += "Success"

            text_2 = "Stop #1 : "
            text_3 = "Stop #2 : "
            text_4 = "Stop #3 : "
            text_5 = "Stop #4 : "
            text_6 = "Stop #5 : "
            text_7 = "PickUp : "
            text_8 = "DropOff : "

            if self.mission_list[STOP_LINE].num_success_stop[0] == 1:
                text_2 += "Success"
            if self.mission_list[STOP_LINE].num_success_stop[1] == 1:
                text_3 += "Success"
            if self.mission_list[STOP_LINE].num_success_stop[2] == 1:
                text_4 += "Success"
            if self.mission_list[STOP_LINE].num_success_stop[3] == 1:
                text_5 += "Success"
            if self.mission_list[STOP_LINE].num_success_stop[4] == 1:
                text_6 += "Success"
            if self.mission_list[DELIV_PICKUP].num_success_pickup == 1:
                text_7 += "Success"
            if self.mission_list[DELIV_DROPOFF].num_success_dropoff[param.MAP_2_PICKUP_DROPOFF_AREA - 1] == 1:
                text_8 += "Success"

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.putText(img, text_1, (0, 30), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_2, (0, 70), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_3, (0, 110), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_4, (0, 150), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_5, (0, 190), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_6, (0, 230), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_7, (0, 270), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_8, (0, 310), font, 1, (255, 255, 255), 2)
            pass

        elif self.map_number == 3:
            text_1 = "Parking : "

            if self.mission_list[PARKING_SPOT].num_success_parking[param.MAP_3_PARKING_AREA - 1] == 1:
                text_1 += "Success"

            text_2 = "Stop #1 : "
            text_3 = "Stop #2 : "
            text_4 = "Stop #3 : "
            text_5 = "PickUp : "
            text_6 = "DropOff : "

            if self.mission_list[STOP_LINE].num_success_stop[0] == 1:
                text_2 += "Success"
            if self.mission_list[STOP_LINE].num_success_stop[1] == 1:
                text_3 += "Success"
            if self.mission_list[STOP_LINE].num_success_stop[2] == 1:
                text_4 += "Success"
            if self.mission_list[DELIV_PICKUP].num_success_pickup == 1:
                text_5 += "Success"
            if self.mission_list[DELIV_DROPOFF].num_success_dropoff[param.MAP_3_PICKUP_DROPOFF_AREA - 1] == 1:
                text_6 += "Success"

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.putText(img, text_1, (0, 30), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_2, (0, 70), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_3, (0, 110), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_4, (0, 150), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_5, (0, 190), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_6, (0, 230), font, 1, (255, 255, 255), 2)
            pass
            

        elif self.map_number ==4:
            pass

        cv2.imshow("mission_image",img)
        cv2.moveWindow("mission_image", 0, 150)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        main_mission = Mainmission()
        rate = rospy.Rate(param.thread_rate)

        while not rospy.is_shutdown():
            try:
                main_mission.main()
                rate.sleep()
            except:
                continue
        
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")
