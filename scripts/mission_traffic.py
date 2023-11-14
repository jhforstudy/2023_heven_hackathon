#!/usr/bin/env python3

import rospy
import numpy as np
import time

from goal import *
from racecar_simulator.msg import Complete
from parameter_list import Param
from abstract_mission import Mission, Carstatus

param = Param()

def dis(pos_1=np.array, pos_2=np.array):
    # Get distance between position 1 and position 2
    position_diff = pos_1 - pos_2
    dist = np.linalg.norm(position_diff)
    return dist


class TrafficMission(Mission):
    def __init__(self, map_number) -> None:
        super().__init__()
        self.map_number = map_number
        
        # Complete는 다시 스폰되는 장소를 업데이트할때만 사용
        self.complete = rospy.Publisher("/complete", Complete, queue_size=1)

        # For traffic mission
        self.traffic_start_time = 0
        self.traffic_index = 0
        self.traffic_flag = 0

    # TBD
    # 만약 갈림길에서 차량이 제대로 이동하는지 자체를 미션으로 평가하려면, 개발해야 함
    def main(self, goal=Goal, car=Carstatus):
        if self.num_passed_traffic == 2:
            # Already Succeed
            rospy.loginfo("Finished Traffic mission. Go ahead.")

        else:
            # goal.position[0] = x coordinate / goal.position[1] = y coordinate / goal.yaw = yaw angle (degree)
            traffic_spot = goal.position
            traffic_msg = Traffic()
            
            if self.traffic_index == 0:
                # Check if a car is stopped
                if goal.mode == STOP_LINE and self.speed == 0 and goal.number == 1:
                    rospy.loginfo("Stopped...")
                    self.traffic_start_time = time.time()
                    self.traffic_index += 1

                traffic_msg.traffic = "STOP"
                traffic_msg.second = param.STOP_LINE_TIME
                self.traffic.publish(traffic_msg)

            elif self.traffic_index == 1:
                # After a car stopped
                if time.time() - self.traffic_start_time >= param.STOP_LINE_TIME:
                    # Traffic stop success
                    rospy.loginfo("Stop mission success!")
                    self.traffic_index += 1
                    self.stop_flag = 1
                    self.num_success_stop[goal.number - 1] += self.stop_flag

                    # publish that parking mission is succeed
                    complete_msg = Complete()
                    complete_msg.complete = True
                    self.complete.publish(complete_msg)

                elif time.time() - self.traffic_start_time < param.STOP_LINE_TIME and self.speed != 0:
                    # Traffic fail
                    rospy.loginfo("Stop mission failed...")
                    self.traffic_index = 0
                    self.stop_flag = 0

                    if not self.traffic_updated:
                        # Spawn index (change spawn area)
                        complete_msg = Complete()
                        complete_msg.complete = True
                        self.complete.publish(complete_msg)
                        self.traffic_updated = True

                else:
                    rospy.loginfo("Trying to stop...")
                
                traffic_msg.traffic = "STOP"
                traffic_msg.second = param.STOP_LINE_TIME - (time.time() - self.traffic_start_time)
                self.traffic.publish(traffic_msg)
            
            elif self.traffic_index == 2:
                # traffic misison failed, break the function
                if traffic_spot is None:
                    self.traffic_start = False
                    self.traffic_index = 0
                # traffic mission success
                else:
                    self.traffic_index += 1
        
            elif self.traffic_index == 3:
                # Check if a car is passed the correct road

                left_pos = np.array((param.MAP_2_CHECK_LEFT_X, param.MAP_2_CHECK_LEFT_Y))
                right_pos = np.array((param.MAP_2_CHECK_RIGHT_X, param.MAP_2_CHECK_RIGHT_Y))
                current_pos = None
                if dis(self.head, left_pos) < 0.5:
                    current_pos = "LEFT"
                elif dis(self.head, right_pos) < 0.5:
                    current_pos = "RIGHT"
                if current_pos is not None:
                    if param.map_2_traffic_dir == current_pos:
                        rospy.loginfo("Correct Way!")
                        self.traffic_success = True
                        self.traffic_flag = 1
                    else:
                        rospy.loginfo("Wrong Way...")
                        self.traffic_success = False
                        self.traffic_flag = 0
                    
                    self.traffic_index += 1

                traffic_msg.traffic = param.map_2_traffic_dir
                self.traffic.publish(traffic_msg)

            elif self.traffic_index == 4:
                if self.traffic_success:
                    self.passed_traffic.append(traffic_spot)

                self.num_passed_traffic += 1
                self.num_success_traffic += self.traffic_flag

                if not self.traffic_updated:
                    # Spawn index (change spawn area)
                    complete_msg = Complete()
                    complete_msg.complete = True
                    self.complete.publish(complete_msg)
                    self.traffic_updated = True

                # Reset the trigger
                self.traffic_start = False
                self.traffic_flag = 0
                self.traffic_success = False
                self.traffic_updated = False
                self.stop_index = 0

    def is_in_mission(self, goal=Goal, car=Carstatus):
        pass

    def init_values(self):
        # For traffic mission
        self.traffic_start_time = 0
        self.traffic_index = 0
        self.traffic_flag = 0