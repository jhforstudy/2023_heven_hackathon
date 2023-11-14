#!/usr/bin/env python3

import rospy
import tf

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from racecar_simulator.msg import CenterPose, Traffic, ParkingInfoList, DropoffInfoList, PickupInfo
from math import *
from goal import *
from parameter_list import Param

param = Param()


class Database():
    def __init__(self, lidar=True):
        # init node
        rospy.init_node('sensor_node')

        # sensor subscriber
        if lidar: rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber("/car_center", CenterPose, self.pose_callback, queue_size=1)
        rospy.Subscriber("/traffic", Traffic, self.traffic_callback, queue_size=1)
        rospy.Subscriber("/parking_info_list", ParkingInfoList, self.parking_callback)
        rospy.Subscriber("/pickup_info", PickupInfo, self.pickup_callback)
        rospy.Subscriber("/dropoff_info_list", DropoffInfoList, self.dropoff_callback)
        rospy.Subscriber("/cur_mission", String, self.mission_name_callback)

        # Which mission?
        self.current_mission = 0

        # Data
        self.lidar_data = None
        self.pose_data = [0,0,0] # x, y, yaw

        # Traffic mission
        self.traffic_light = None
        self.traffic_remaining_time = 0

        # Parking mission
        self.target_park = param.MAP_1_PARKING_AREA
        self.parking_list = []

        # Delivery mission
        self.pickup_list = []
        self.dropoff_list = []

    def lidar_callback(self, data=LaserScan):
        self.lidar_data = data.ranges

    def pickup_callback(self, data=PickupInfo):
        self.pickup_list = [data.x, data.y, data.yaw, data.deliv_num]
    
    def dropoff_callback(self, data=DropoffInfoList):
        dropoff_list = []
        for info in data.InfoList:
            info_element = [info.x, info.y, info.yaw, info.deliv_num]
            dropoff_list.append(info_element)
        self.dropoff_list = dropoff_list

    def pose_callback(self, data=CenterPose):
        self.pose_data = data.pose
        
    def traffic_callback(self, data=Traffic):
        self.traffic_light = data.traffic
        self.traffic_remaining_time = data.second

    def parking_callback(self, data=ParkingInfoList):
        parking_list = []
        for info in data.InfoList:
            info_element = [info.x, info.y, info.yaw, info.park_num]
            parking_list.append(info_element)
        self.parking_list = parking_list

    def mission_name_callback(self, data):
        if data.data == "parking":
            self.current_mission = PARKING_SPOT

        elif data.data == "stop_line":
            self.current_mission = STOP_LINE

        elif data.data == "delivery_pickup":
            self.current_mission = DELIV_PICKUP

        elif data.data == "delivery_dropoff":
            self.current_mission = DELIV_DROPOFF

        elif data.data == "traffic":
            self.current_mission = TRAFFIC

        else:
            self.current_mission = 0

if __name__ == "__main__":
    try:
        test_db = Database(lidar=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")