#!/usr/bin/env python3

import rospy
import numpy as np
import time

from goal import *
from racecar_simulator.msg import Complete, PickupInfo
from parameter_list import Param
from abstract_mission import Mission, Carstatus

param = Param()


class DelivPickupMission(Mission):
    def __init__(self, map_number) -> None:
        super().__init__()
        self.map_number = map_number

        # Complete는 다시 스폰되는 장소를 업데이트할때만 사용
        self.complete = rospy.Publisher("/complete", Complete, queue_size=1)
        self.pickup = rospy.Publisher("/pickup_info", PickupInfo, queue_size=1)

        # For Delivery PICKUP
        self.pickup_start_time = 0
        self.pickup_index = 0
        self.pickup_flag = 0
        self.num_success_pickup = 0     # Map 1

    def main(self, goal=Goal, car=Carstatus):
        if self.num_success_pickup == 1:
            # Already Succeed
            rospy.loginfo("Finished Pickup. Go ahead.")

        else:
            if self.pickup_index == 0:
                if car.speed == 0 and self.is_in_deliv(goal.position, goal.yaw, car.position, car.position_yaw):
                    self.pickup_start_time = time.time()
                    self.pickup_index += 1
            
            elif self.pickup_index == 1:
                if time.time() - self.pickup_start_time >= 3:
                    # Pickup success
                    rospy.loginfo("Pickup mission success!")
                    self.pickup_index += 1
                    self.pickup_flag = 1
                
                elif time.time() - self.pickup_start_time < 3 and car.speed != 0:
                    # pickup fail
                    rospy.loginfo("pickup mission failed...")
                    self.pickup_flag = 0
                    self.pickup_index = 0
                
                else:
                    rospy.loginfo("Trying to pickup...")
            
            elif self.pickup_index == 2:
                # pickup misison failed, break the function
                if self.pickup_flag == 0:
                    self.pickup_index = 0

                # pickup mission success
                else:
                    self.pickup_index += 1

            elif self.pickup_index == 3:
                self.num_success_pickup = self.pickup_flag
                    
                # Spawn index (change spawn area)
                complete_msg = Complete()
                complete_msg.complete = True
                self.complete.publish(complete_msg)

                # Reset the trigger
                self.pickup_flag = 0
                self.pickup_index = 0

    def is_in_mission(self, goal=Goal, car=Carstatus):
        # Check if a car passes stop line
        position_diff = goal.position - car.position
        yaw_diff = goal.yaw - car.position_yaw

        # Delivery
        # We need to check : x, y (close to the area?), yaw (align direction?)
        if abs(position_diff[0]) <= goal.tolerance[0] and abs(position_diff[1]) <= goal.tolerance[1] and abs(yaw_diff) <= 10:
            # PICKUP (Practice Map)
            pickup_vector = np.array([[goal.position[0]], [goal.position[1]]])
            position_vector = np.array([[car.position[0]], [car.position[1]]])
            pickup_msg = PickupInfo()
            pickup_msg.x, pickup_msg.y, pickup_msg.yaw = self.calc_relative_coordinates(pickup_vector, goal.yaw, position_vector, car.position_yaw)
            pickup_msg.deliv_num = goal.number

            self.pickup.publish(pickup_msg)

            return True
        
        return False
    
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
        # For Delivery PICKUP
        self.pickup_start_time = 0
        self.pickup_index = 0
        self.pickup_flag = 0
        self.num_success_pickup = 0     # Map 1