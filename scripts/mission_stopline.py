#!/usr/bin/env python3

import rospy
import numpy as np
import time

from goal import *
from racecar_simulator.msg import Complete, Traffic
from parameter_list import Param
from abstract_mission import Mission, Carstatus

param = Param()


class StopLineMission(Mission):
    def __init__(self, map_number) -> None:
        super().__init__()
        self.map_number = map_number

        # Complete는 다시 스폰되는 장소를 업데이트할때만 사용
        self.complete = rospy.Publisher("/complete", Complete, queue_size=1)
        self.traffic = rospy.Publisher("/traffic", Traffic, queue_size=1)

        # For stop mission
        self.stop_time = 0              # 차량이 멈춘 시간
        self.stop_index = 0             # 미션의 현재 상태(state)를 정의하는 변수
        self.stop_flag = 1              # 미션 성공시 1, 실패시 0
        if self.map_number == 1:
            self.num_success_stop = [0,0]
        elif self.map_number == 2:
            self.num_success_stop = [0,0,0,0,0]
        elif self.map_number == 3:
            self.num_success_stop = [0,0,0]

    def main(self, goal=Goal, car=Carstatus):
        traffic_msg = Traffic()
        if self.num_success_stop[goal.number - 1] == 1:
            # Already Succeed
            rospy.loginfo("Finished Stop line. Go ahead.")
            traffic_msg.traffic = goal.traffic
            traffic_msg.second = 0
            self.traffic.publish(traffic_msg)
        
        else:
            # rospy.loginfo("Curr idx %d" %(self.stop_index))
            if self.stop_index == 0:
                # 차가 멈추었는 지 확인
                if car.speed == 0 and self.is_in_stopline(goal, car):
                    rospy.loginfo("Stopped...")
                    self.stop_time = time.time()
                    self.stop_index += 1

                traffic_msg.traffic = "STOP"
                traffic_msg.second = param.STOP_LINE_TIME
                self.traffic.publish(traffic_msg)
            
            elif self.stop_index == 1:
                # After a car stopped
                if time.time() - self.stop_time >= param.STOP_LINE_TIME:
                    # Stop success
                    rospy.loginfo("Stop mission success!")
                    self.stop_flag = 1
                    self.stop_index += 1

                elif time.time() - self.stop_time < param.STOP_LINE_TIME and car.speed != 0:
                    # Stop fail
                    rospy.loginfo("Stop mission failed...")
                    self.stop_flag = 0
                    self.stop_index += 1
                
                else:
                    rospy.loginfo("Trying to stop...")
                
                traffic_msg.traffic = "STOP"
                traffic_msg.second = param.STOP_LINE_TIME - (time.time() - self.stop_time)
                self.traffic.publish(traffic_msg)

            elif self.stop_index == 2:
                # stop mission failed, break the function
                if self.stop_flag == 0:
                    self.stop_index = 0

                # stop misison success
                else:
                    self.stop_index += 1
            
            elif self.stop_index == 3:
                # Stop mission finally end (성공했으면, 해당 index가 1로 바뀜)
                self.num_success_stop[goal.number - 1] += self.stop_flag
                
                # Spawn index (change spawn area)
                complete_msg = Complete()
                complete_msg.complete = True
                self.complete.publish(complete_msg)
                
                # Reset the trigger
                self.stop_flag = 0
                self.stop_index = 0
                
    def is_in_mission(self, goal=Goal, car=Carstatus):
        # Check if a car is in the stop line mission
        position_diff = goal.position - car.position
        # We need to check : x, y (close to the area?)
        if abs(position_diff[0]) <= goal.tolerance[0] and abs(position_diff[1]) <= goal.tolerance[1]:
            return True
        
        return False

    def is_in_stopline(self, goal=Goal, car=Carstatus):
        # Check if a car passes stop line
        position_diff = goal.position - car.position
        dist = np.linalg.norm(position_diff)

        # Stop
        # We need to check : x, y (close to the area?), is CAR behind the STOP LINE?
        if np.dot(car.position_unit_vector, -goal.unit_vector)<=0\
            and np.dot(position_diff, -goal.unit_vector)<=0\
            and dist <= 1:

            return True
        
        return False
    
    def publish_null_traffic(self):
        traffic_msg = Traffic()
        traffic_msg.traffic = "None"
        traffic_msg.second = 0
        self.traffic.publish(traffic_msg)

    def init_values(self):
        self.stop_time = 0              # 차량이 멈춘 시간
        self.stop_index = 0             # 미션의 현재 상태(state)를 정의하는 변수
        self.stop_flag = 1              # 미션 성공시 1, 실패시 0
        if self.map_number == 1:
            self.num_success_stop = [0,0]
        elif self.map_number == 2:
            self.num_success_stop = [0,0,0,0,0]
        elif self.map_number == 3:
            pass