#!/usr/bin/env python3

import rospy
import numpy as np
import time

from goal import *
from racecar_simulator.msg import Complete, DropoffInfo
from parameter_list import Param
from abstract_mission import Mission, Carstatus

param = Param()


class DelivDropoffMission(Mission):
    def __init__(self, map_number) -> None:
        super().__init__()
        self.map_number = map_number

        # Complete는 다시 스폰되는 장소를 업데이트할때만 사용
        self.complete = rospy.Publisher("/complete", Complete, queue_size=1)

        # For Delivery Dropoff
        self.dropoff_start_time = 0
        self.dropoff_index = 0
        self.dropoff_flag = 0
        self.num_success_dropoff = [0,0,0]          # Map 1

    def main(self, goal=Goal, car=Carstatus):
        if goal.number == param.MAP_1_PICKUP_DROPOFF_AREA:
            if self.num_success_dropoff[goal.number - 1] == 1:
                # Already Succeed
                rospy.loginfo("Finished Dropoff. Go ahead.")
            else:
                if self.dropoff_index == 0:
                    if car.speed == 0 and self.is_in_deliv(goal.position, goal.yaw, car.position, car.position_yaw):
                        self.dropoff_start_time = time.time()
                        self.dropoff_index += 1
                
                elif self.dropoff_index == 1:
                    if time.time() - self.dropoff_start_time >= 3:
                        # dropoff success
                        rospy.loginfo("Dropoff mission success!")
                        self.dropoff_flag = 1
                        self.dropoff_index += 1
                    
                    elif time.time() - self.dropoff_start_time < 3 and car.speed != 0:
                        # dropoff fail
                        rospy.loginfo("Dropoff mission failed...")
                        self.dropoff_flag = 0
                        self.dropoff_index = 0
                    
                    else:
                        rospy.loginfo("Trying to Dropoff...")

                elif self.dropoff_index == 2:
                    # dropoff misison failed, break the function
                    if self.dropoff_flag == 0:
                        self.dropoff_index = 0

                    # dropoff mission success
                    else:
                        self.dropoff_index += 1
        
                elif self.dropoff_index == 3:
                    self.num_success_dropoff[goal.number - 1] += self.dropoff_flag
                    
                    # Spawn index (change spawn area)
                    complete_msg = Complete()
                    complete_msg.complete = True
                    self.complete.publish(complete_msg)

                    # Reset the trigger
                    self.dropoff_flag = 0
                    self.dropoff_index = 0

        else:
            pass

    def is_in_mission(self, goal=Goal, car=Carstatus):
        position_diff = goal.position - car.position
        yaw_diff = goal.yaw - math.radians(car.position_yaw)
        ###########################################################
        ######## Delivery relative coordinates calculation ########
        ###########################################################

        if abs(position_diff[0]) <= goal.tolerance[0] and abs(position_diff[1]) <= goal.tolerance[1]:
            
            dropoff_vector = np.array([[goal.position[0]], [goal.position[1]]])
            position_vector = np.array([[car.position[0]], [car.position[1]]])
            dropoff_msg = DropoffInfo()
            dropoff_msg.x, dropoff_msg.y, dropoff_msg.yaw = self.calc_relative_coordinates(dropoff_vector, goal.yaw, position_vector, car.position_yaw)
            dropoff_msg.deliv_num = goal.number

            return dropoff_msg
        
        return None
    
    def is_in_deliv(self, goal_pos, goal_yaw, car_pos, car_yaw):
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
        # For Delivery Dropoff
        self.dropoff_start_time = 0
        self.dropoff_index = 0
        self.dropoff_flag = 0
        self.num_success_dropoff = [0,0,0]          # Map 1