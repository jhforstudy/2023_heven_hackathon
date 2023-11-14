#!/usr/bin/env python3

import rospy
import numpy as np
import time
import math

from goal import *
from racecar_simulator.msg import Complete, ParkingInfo
from parameter_list import Param
from abstract_mission import Mission, Carstatus

param = Param()


class ParkingMission(Mission):
    def __init__(self, map_number) -> None:
        super().__init__()
        self.map_number = map_number

        # Complete는 다시 스폰되는 장소를 업데이트할때만 사용
        self.complete = rospy.Publisher("/complete", Complete, queue_size=1)

        # For parking
        self.parking_start_time = 0
        self.parking_index = 0
        self.parking_flag = 0
        self.parked_spots = []
        self.num_success_parking = [0,0,0]      # Map 1

    def main(self, goal=Goal, car=Carstatus):
        if goal.number == param.MAP_1_PARKING_AREA:
            if self.num_success_parking[goal.number - 1] == 1:
                # Already Succeed
                rospy.loginfo("Finished Parking. Go ahead.")

            else:
                # Check parking
                # goal.position[0] = x coordinate / goal.position[1] = y coordinate / goal.yaw = yaw angle (degree)
                
                if self.parking_index == 0:
                    # Check if a car is stopped
                    if car.speed == 0 and self.is_in_parking(goal.position, goal.yaw, car.position, car.position_yaw):
                        self.parking_start_time = time.time()
                        self.parking_index += 1

                elif self.parking_index == 1:
                    # After a car stopped
                    if time.time() - self.parking_start_time >= 3:
                        # Parking success
                        rospy.loginfo("Parking mission success!")
                        self.parking_flag = 1
                        self.parking_index += 1

                    elif time.time() - self.parking_start_time < 3 and car.speed != 0:
                        # Parking fail
                        rospy.loginfo("Parking mission failed...")
                        self.parking_flag = 0
                        self.parking_index += 1
                
                    else:
                        rospy.loginfo("Trying to park...")
                    
                elif self.parking_index == 2:
                    # parking misison failed, break the function
                    if self.parking_flag == 0:
                        self.parking_index = 0

                    # parking mission success
                    else:
                        self.parking_index += 1
            
                elif self.parking_index == 3:
                    self.num_success_parking[goal.number - 1] += self.parking_flag
                    
                    # Spawn index (change spawn area)
                    complete_msg = Complete()
                    complete_msg.complete = True
                    self.complete.publish(complete_msg)

                    # Reset the trigger
                    self.parking_flag = 0
                    self.parking_index = 0
        
        else:
            pass
    
    def is_in_mission(self, goal=Goal, car=Carstatus):
        # Check if a car is in the parking lot
        position_diff = goal.position - car.position
        yaw_diff = goal.yaw - car.position_yaw
        # We need to check : x, y (close to the area?)
        if abs(position_diff[0]) <= goal.tolerance[0] and abs(position_diff[1]) <= goal.tolerance[1]:
            # Publish parking info
            parking_vector = np.array([[goal.position[0]], [goal.position[1]]])
            position_vector = np.array([[car.position[0]], [car.position[1]]])

            parkinginfo_msg = ParkingInfo()
            parkinginfo_msg.x, parkinginfo_msg.y, parkinginfo_msg.yaw = self.calc_relative_coordinates(parking_vector, goal.yaw, position_vector, car.position_yaw)
            parkinginfo_msg.park_num = goal.number
            
            return parkinginfo_msg
        
        return None
    
    def is_in_parking(self, goal_pos, goal_yaw, car_pos, car_yaw):
        position_diff = goal_pos - car_pos
        yaw_diff = goal_yaw - car_yaw

        if yaw_diff > 180:
            yaw_diff = 360-yaw_diff
        
        elif yaw_diff < -180:
            yaw_diff = -360-yaw_diff

        if abs(position_diff[0]) <= 0.145 and abs(position_diff[1]) <= 0.145 and abs(yaw_diff) <= 10:
            return True

        else:
            return False
    
    def calc_relative_coordinates(self, target_xy_vector, target_yaw, car_xy_vector, car_yaw):
        rotation_matrix = np.array([[math.cos(math.radians(car_yaw)), math.sin(math.radians(car_yaw))],
                                    [-math.sin(math.radians(car_yaw)), math.cos(math.radians(car_yaw))]])
        
        translation_vecetor = -rotation_matrix @ car_xy_vector

        relative_vector = rotation_matrix @ target_xy_vector + translation_vecetor
        
        x = relative_vector[0][0]
        y = relative_vector[1][0]
        yaw = target_yaw - car_yaw

        if yaw > 180:
            yaw = 360-yaw
        
        elif yaw < -180:
            yaw = -360-yaw

        return x, y, yaw
    
    def init_values(self):
        # For parking
        self.parking_start_time = 0
        self.parking_index = 0
        self.parking_flag = 0
        self.parked_spots = []
        self.num_success_parking = [0,0,0]      # Map 1