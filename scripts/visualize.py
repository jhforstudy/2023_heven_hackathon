#!/usr/bin/env python3

import rospy

from math import pi
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from parameter_list import Param
from tf.transformations import quaternion_from_euler

param = Param()

def MakeGoalMarker(map_number):
    if map_number == 1:
        m = param.m
        m.pose.position.x = param.END_POINT_X_1
        m.pose.position.y = param.END_POINT_Y_1
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m
    
    elif map_number == 2:
        m = param.m
        m.pose.position.x = param.END_POINT_X_2
        m.pose.position.y = param.END_POINT_Y_2
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m

    elif map_number == 3:
        m = param.m
        m.pose.position.x = param.END_POINT_X_3
        m.pose.position.y = param.END_POINT_Y_3
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m

    #JERK
    elif map_number == 4:
        m = param.m
        m.pose.position.x = param.END_POINT_X_4
        m.pose.position.y = param.END_POINT_Y_4
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m
    
    elif map_number == 5:
        m = param.m
        m.pose.position.x = param.END_POINT_X_5
        m.pose.position.y = param.END_POINT_Y_5
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m

    else:
        rospy.loginfo("Map number is incorrect.")

def MakeTrafficMarker(map_number):
    if map_number == 1:
        traffic_stop = Marker()
        traffic_stop.header.frame_id = "map"
        traffic_stop.ns = "traffic_stop"
        traffic_stop.type = Marker.LINE_STRIP
        traffic_stop.action = Marker.ADD
        traffic_stop.color.r, traffic_stop.color.g, traffic_stop.color.b = 1, 0, 0
        traffic_stop.color.a = 1
        traffic_stop.scale.x = 0.1
        traffic_stop.scale.y = 0.1
        traffic_stop.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_1_STOP_LINE_X_1
        l_point.y = param.MAP_1_STOP_LINE_Y_1 + param.STOP_LINE_SIZE / 2
        l_point.z = 0 
        r_point = Point()
        r_point.x = param.MAP_1_STOP_LINE_X_1
        r_point.y = param.MAP_1_STOP_LINE_Y_1 - param.STOP_LINE_SIZE / 2
        r_point.z = 0

        traffic_stop.points.append(l_point)
        traffic_stop.points.append(r_point)

        stop_sign = Marker()
        stop_sign.header.frame_id = "map"
        stop_sign.ns = "stop_sign"
        stop_sign.type = Marker.CYLINDER
        stop_sign.action = Marker.ADD
        stop_sign.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign.color.a = 1
        stop_sign.scale.x = param.SIZE_OF_TROPHY
        stop_sign.scale.y = param.SIZE_OF_TROPHY
        stop_sign.scale.z = 0
        stop_sign.pose.position.x = param.MAP_1_STOP_LINE_X_1 - 0.5
        stop_sign.pose.position.y = param.MAP_1_STOP_LINE_Y_1
        stop_sign.pose.position.z = 0
        stop_sign.pose.orientation.x = 0.0
        stop_sign.pose.orientation.y = 0.0
        stop_sign.pose.orientation.z = 0.0
        stop_sign.pose.orientation.w = 1.0

        traffic_stop_2 = Marker()
        traffic_stop_2.header.frame_id = "map"
        traffic_stop_2.ns = "traffic_stop_2"
        traffic_stop_2.type = Marker.LINE_STRIP
        traffic_stop_2.action = Marker.ADD
        traffic_stop_2.color.r, traffic_stop_2.color.g, traffic_stop_2.color.b = 1, 0, 0
        traffic_stop_2.color.a = 1
        traffic_stop_2.scale.x = 0.1
        traffic_stop_2.scale.y = 0.1
        traffic_stop_2.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_1_STOP_LINE_X_2 - param.STOP_LINE_SIZE / 2
        l_point.y = param.MAP_1_STOP_LINE_Y_2
        l_point.z = 0
        r_point = Point()
        r_point.x = param.MAP_1_STOP_LINE_X_2 + param.STOP_LINE_SIZE / 2
        r_point.y = param.MAP_1_STOP_LINE_Y_2
        r_point.z = 0

        traffic_stop_2.points.append(l_point)
        traffic_stop_2.points.append(r_point)

        stop_sign_2 = Marker()
        stop_sign_2.header.frame_id = "map"
        stop_sign_2.ns = "stop_sign_2"
        stop_sign_2.type = Marker.CYLINDER
        stop_sign_2.action = Marker.ADD
        stop_sign_2.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign_2.color.a = 1
        stop_sign_2.scale.x = param.SIZE_OF_TROPHY
        stop_sign_2.scale.y = param.SIZE_OF_TROPHY
        stop_sign_2.scale.z = 0
        stop_sign_2.pose.position.x = param.MAP_1_STOP_LINE_X_2
        stop_sign_2.pose.position.y = param.MAP_1_STOP_LINE_Y_2 + 0.5
        stop_sign_2.pose.position.z = 0
        stop_sign_2.pose.orientation.x = 0.0
        stop_sign_2.pose.orientation.y = 0.0
        stop_sign_2.pose.orientation.z = 0.0
        stop_sign_2.pose.orientation.w = 1.0
        
        return traffic_stop, stop_sign, traffic_stop_2, stop_sign_2
    
    elif map_number == 2:
        traffic_stop = Marker()
        traffic_stop.header.frame_id = "map"
        traffic_stop.ns = "traffic_stop"
        traffic_stop.type = Marker.LINE_STRIP
        traffic_stop.action = Marker.ADD
        traffic_stop.color.r, traffic_stop.color.g, traffic_stop.color.b = 1, 0, 0
        traffic_stop.color.a = 1
        traffic_stop.scale.x = 0.1
        traffic_stop.scale.y = 0.1
        traffic_stop.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_2_STOP_LINE_X_1
        l_point.y = param.MAP_2_STOP_LINE_Y_1 + param.STOP_LINE_SIZE / 2
        l_point.z = 0 
        r_point = Point()
        r_point.x = param.MAP_2_STOP_LINE_X_1
        r_point.y = param.MAP_2_STOP_LINE_Y_1 - param.STOP_LINE_SIZE / 2
        r_point.z = 0

        traffic_stop.points.append(l_point)
        traffic_stop.points.append(r_point)

        stop_sign = Marker()
        stop_sign.header.frame_id = "map"
        stop_sign.ns = "stop_sign"
        stop_sign.type = Marker.CYLINDER
        stop_sign.action = Marker.ADD
        stop_sign.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign.color.a = 1
        stop_sign.scale.x = param.SIZE_OF_TROPHY
        stop_sign.scale.y = param.SIZE_OF_TROPHY
        stop_sign.scale.z = 0
        stop_sign.pose.position.x = param.MAP_2_STOP_LINE_X_1 - 0.5
        stop_sign.pose.position.y = param.MAP_2_STOP_LINE_Y_1
        stop_sign.pose.position.z = 0
        stop_sign.pose.orientation.x = 0.0
        stop_sign.pose.orientation.y = 0.0
        stop_sign.pose.orientation.z = 0.0
        stop_sign.pose.orientation.w = 1.0

        traffic_stop_2 = Marker()
        traffic_stop_2.header.frame_id = "map"
        traffic_stop_2.ns = "traffic_stop_2"
        traffic_stop_2.type = Marker.LINE_STRIP
        traffic_stop_2.action = Marker.ADD
        traffic_stop_2.color.r, traffic_stop_2.color.g, traffic_stop_2.color.b = 1, 0, 0
        traffic_stop_2.color.a = 1
        traffic_stop_2.scale.x = 0.1
        traffic_stop_2.scale.y = 0.1
        traffic_stop_2.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_2_STOP_LINE_X_2
        l_point.y = param.MAP_2_STOP_LINE_Y_2 - param.STOP_LINE_SIZE / 2
        l_point.z = 0
        r_point = Point()
        r_point.x = param.MAP_2_STOP_LINE_X_2
        r_point.y = param.MAP_2_STOP_LINE_Y_2 + param.STOP_LINE_SIZE / 2
        r_point.z = 0

        traffic_stop_2.points.append(l_point)
        traffic_stop_2.points.append(r_point)

        stop_sign_2 = Marker()
        stop_sign_2.header.frame_id = "map"
        stop_sign_2.ns = "stop_sign_2"
        stop_sign_2.type = Marker.CYLINDER
        stop_sign_2.action = Marker.ADD
        stop_sign_2.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign_2.color.a = 1
        stop_sign_2.scale.x = param.SIZE_OF_TROPHY
        stop_sign_2.scale.y = param.SIZE_OF_TROPHY
        stop_sign_2.scale.z = 0
        stop_sign_2.pose.position.x = param.MAP_2_STOP_LINE_X_2 - 0.5
        stop_sign_2.pose.position.y = param.MAP_2_STOP_LINE_Y_2
        stop_sign_2.pose.position.z = 0
        stop_sign_2.pose.orientation.x = 0.0
        stop_sign_2.pose.orientation.y = 0.0
        stop_sign_2.pose.orientation.z = 0.0
        stop_sign_2.pose.orientation.w = 1.0

        traffic_stop_3 = Marker()
        traffic_stop_3.header.frame_id = "map"
        traffic_stop_3.ns = "traffic_stop_3"
        traffic_stop_3.type = Marker.LINE_STRIP
        traffic_stop_3.action = Marker.ADD
        traffic_stop_3.color.r, traffic_stop_3.color.g, traffic_stop_3.color.b = 1, 0, 0
        traffic_stop_3.color.a = 1
        traffic_stop_3.scale.x = 0.1
        traffic_stop_3.scale.y = 0.1
        traffic_stop_3.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_2_STOP_LINE_X_3 - param.STOP_LINE_SIZE / 2
        l_point.y = param.MAP_2_STOP_LINE_Y_3
        l_point.z = 0
        r_point = Point()
        r_point.x = param.MAP_2_STOP_LINE_X_3 + param.STOP_LINE_SIZE / 2
        r_point.y = param.MAP_2_STOP_LINE_Y_3
        r_point.z = 0

        traffic_stop_3.points.append(l_point)
        traffic_stop_3.points.append(r_point)

        stop_sign_3 = Marker()
        stop_sign_3.header.frame_id = "map"
        stop_sign_3.ns = "stop_sign_3"
        stop_sign_3.type = Marker.CYLINDER
        stop_sign_3.action = Marker.ADD
        stop_sign_3.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign_3.color.a = 1
        stop_sign_3.scale.x = param.SIZE_OF_TROPHY
        stop_sign_3.scale.y = param.SIZE_OF_TROPHY
        stop_sign_3.scale.z = 0
        stop_sign_3.pose.position.x = param.MAP_2_STOP_LINE_X_3
        stop_sign_3.pose.position.y = param.MAP_2_STOP_LINE_Y_3 + 0.5
        stop_sign_3.pose.position.z = 0
        stop_sign_3.pose.orientation.x = 0.0
        stop_sign_3.pose.orientation.y = 0.0
        stop_sign_3.pose.orientation.z = 0.0
        stop_sign_3.pose.orientation.w = 1.0

        traffic_stop_4 = Marker()
        traffic_stop_4.header.frame_id = "map"
        traffic_stop_4.ns = "traffic_stop_4"
        traffic_stop_4.type = Marker.LINE_STRIP
        traffic_stop_4.action = Marker.ADD
        traffic_stop_4.color.r, traffic_stop_4.color.g, traffic_stop_4.color.b = 1, 0, 0
        traffic_stop_4.color.a = 1
        traffic_stop_4.scale.x = 0.1
        traffic_stop_4.scale.y = 0.1
        traffic_stop_4.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_2_STOP_LINE_X_4
        l_point.y = param.MAP_2_STOP_LINE_Y_4 - param.STOP_LINE_SIZE / 2
        l_point.z = 0
        r_point = Point()
        r_point.x = param.MAP_2_STOP_LINE_X_4
        r_point.y = param.MAP_2_STOP_LINE_Y_4 + param.STOP_LINE_SIZE / 2
        r_point.z = 0

        traffic_stop_4.points.append(l_point)
        traffic_stop_4.points.append(r_point)

        stop_sign_4 = Marker()
        stop_sign_4.header.frame_id = "map"
        stop_sign_4.ns = "stop_sign_4"
        stop_sign_4.type = Marker.CYLINDER
        stop_sign_4.action = Marker.ADD
        stop_sign_4.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign_4.color.a = 1
        stop_sign_4.scale.x = param.SIZE_OF_TROPHY
        stop_sign_4.scale.y = param.SIZE_OF_TROPHY
        stop_sign_4.scale.z = 0
        stop_sign_4.pose.position.x = param.MAP_2_STOP_LINE_X_4 + 0.5
        stop_sign_4.pose.position.y = param.MAP_2_STOP_LINE_Y_4
        stop_sign_4.pose.position.z = 0
        stop_sign_4.pose.orientation.x = 0.0
        stop_sign_4.pose.orientation.y = 0.0
        stop_sign_4.pose.orientation.z = 0.0
        stop_sign_4.pose.orientation.w = 1.0

        traffic_stop_5 = Marker()
        traffic_stop_5.header.frame_id = "map"
        traffic_stop_5.ns = "traffic_stop_5"
        traffic_stop_5.type = Marker.LINE_STRIP
        traffic_stop_5.action = Marker.ADD
        traffic_stop_5.color.r, traffic_stop_5.color.g, traffic_stop_5.color.b = 1, 0, 0
        traffic_stop_5.color.a = 1
        traffic_stop_5.scale.x = 0.1
        traffic_stop_5.scale.y = 0.1
        traffic_stop_5.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_2_STOP_LINE_X_5
        l_point.y = param.MAP_2_STOP_LINE_Y_5 - param.STOP_LINE_SIZE / 2
        l_point.z = 0
        r_point = Point()
        r_point.x = param.MAP_2_STOP_LINE_X_5
        r_point.y = param.MAP_2_STOP_LINE_Y_5 + param.STOP_LINE_SIZE / 2
        r_point.z = 0

        traffic_stop_5.points.append(l_point)
        traffic_stop_5.points.append(r_point)

        stop_sign_5 = Marker()
        stop_sign_5.header.frame_id = "map"
        stop_sign_5.ns = "stop_sign_5"
        stop_sign_5.type = Marker.CYLINDER
        stop_sign_5.action = Marker.ADD
        stop_sign_5.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign_5.color.a = 1
        stop_sign_5.scale.x = param.SIZE_OF_TROPHY
        stop_sign_5.scale.y = param.SIZE_OF_TROPHY
        stop_sign_5.scale.z = 0
        stop_sign_5.pose.position.x = param.MAP_2_STOP_LINE_X_5 + 0.5
        stop_sign_5.pose.position.y = param.MAP_2_STOP_LINE_Y_5
        stop_sign_5.pose.position.z = 0
        stop_sign_5.pose.orientation.x = 0.0
        stop_sign_5.pose.orientation.y = 0.0
        stop_sign_5.pose.orientation.z = 0.0
        stop_sign_5.pose.orientation.w = 1.0
        
        return traffic_stop, stop_sign, traffic_stop_2, stop_sign_2, traffic_stop_3, stop_sign_3, traffic_stop_4, stop_sign_4, traffic_stop_5, stop_sign_5

    elif map_number == 3:
        traffic_stop = Marker()
        traffic_stop.header.frame_id = "map"
        traffic_stop.ns = "traffic_stop"
        traffic_stop.type = Marker.LINE_STRIP
        traffic_stop.action = Marker.ADD
        traffic_stop.color.r, traffic_stop.color.g, traffic_stop.color.b = 1, 0, 0
        traffic_stop.color.a = 1
        traffic_stop.scale.x = 0.1
        traffic_stop.scale.y = 0.1
        traffic_stop.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_3_STOP_LINE_X_1 - param.STOP_LINE_SIZE / 2
        l_point.y = param.MAP_3_STOP_LINE_Y_1
        l_point.z = 0 
        r_point = Point()
        r_point.x = param.MAP_3_STOP_LINE_X_1 + param.STOP_LINE_SIZE / 2
        r_point.y = param.MAP_3_STOP_LINE_Y_1
        r_point.z = 0

        traffic_stop.points.append(l_point)
        traffic_stop.points.append(r_point)

        stop_sign = Marker()
        stop_sign.header.frame_id = "map"
        stop_sign.ns = "stop_sign"
        stop_sign.type = Marker.CYLINDER
        stop_sign.action = Marker.ADD
        stop_sign.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign.color.a = 1
        stop_sign.scale.x = param.SIZE_OF_TROPHY
        stop_sign.scale.y = param.SIZE_OF_TROPHY
        stop_sign.scale.z = 0
        stop_sign.pose.position.x = param.MAP_3_STOP_LINE_X_1
        stop_sign.pose.position.y = param.MAP_3_STOP_LINE_Y_1 + 0.5
        stop_sign.pose.position.z = 0
        stop_sign.pose.orientation.x = 0.0
        stop_sign.pose.orientation.y = 0.0
        stop_sign.pose.orientation.z = 0.0
        stop_sign.pose.orientation.w = 1.0

        traffic_stop_2 = Marker()
        traffic_stop_2.header.frame_id = "map"
        traffic_stop_2.ns = "traffic_stop_2"
        traffic_stop_2.type = Marker.LINE_STRIP
        traffic_stop_2.action = Marker.ADD
        traffic_stop_2.color.r, traffic_stop_2.color.g, traffic_stop_2.color.b = 1, 0, 0
        traffic_stop_2.color.a = 1
        traffic_stop_2.scale.x = 0.1
        traffic_stop_2.scale.y = 0.1
        traffic_stop_2.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_3_STOP_LINE_X_2 - param.STOP_LINE_SIZE / 2
        l_point.y = param.MAP_3_STOP_LINE_Y_2
        l_point.z = 0
        r_point = Point()
        r_point.x = param.MAP_3_STOP_LINE_X_2 + param.STOP_LINE_SIZE / 2
        r_point.y = param.MAP_3_STOP_LINE_Y_2
        r_point.z = 0

        traffic_stop_2.points.append(l_point)
        traffic_stop_2.points.append(r_point)

        stop_sign_2 = Marker()
        stop_sign_2.header.frame_id = "map"
        stop_sign_2.ns = "stop_sign_2"
        stop_sign_2.type = Marker.CYLINDER
        stop_sign_2.action = Marker.ADD
        stop_sign_2.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign_2.color.a = 1
        stop_sign_2.scale.x = param.SIZE_OF_TROPHY
        stop_sign_2.scale.y = param.SIZE_OF_TROPHY
        stop_sign_2.scale.z = 0
        stop_sign_2.pose.position.x = param.MAP_3_STOP_LINE_X_2
        stop_sign_2.pose.position.y = param.MAP_3_STOP_LINE_Y_2 + 0.5
        stop_sign_2.pose.position.z = 0
        stop_sign_2.pose.orientation.x = 0.0
        stop_sign_2.pose.orientation.y = 0.0
        stop_sign_2.pose.orientation.z = 0.0
        stop_sign_2.pose.orientation.w = 1.0

        traffic_stop_3 = Marker()
        traffic_stop_3.header.frame_id = "map"
        traffic_stop_3.ns = "traffic_stop_3"
        traffic_stop_3.type = Marker.LINE_STRIP
        traffic_stop_3.action = Marker.ADD
        traffic_stop_3.color.r, traffic_stop_3.color.g, traffic_stop_3.color.b = 1, 0, 0
        traffic_stop_3.color.a = 1
        traffic_stop_3.scale.x = 0.1
        traffic_stop_3.scale.y = 0.1
        traffic_stop_3.scale.z = 0
        l_point = Point()
        l_point.x = param.MAP_3_STOP_LINE_X_3 - param.STOP_LINE_SIZE / 2
        l_point.y = param.MAP_3_STOP_LINE_Y_3
        l_point.z = 0
        r_point = Point()
        r_point.x = param.MAP_3_STOP_LINE_X_3 + param.STOP_LINE_SIZE / 2
        r_point.y = param.MAP_3_STOP_LINE_Y_3
        r_point.z = 0

        traffic_stop_3.points.append(l_point)
        traffic_stop_3.points.append(r_point)

        stop_sign_3 = Marker()
        stop_sign_3.header.frame_id = "map"
        stop_sign_3.ns = "stop_sign_3"
        stop_sign_3.type = Marker.CYLINDER
        stop_sign_3.action = Marker.ADD
        stop_sign_3.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign_3.color.a = 1
        stop_sign_3.scale.x = param.SIZE_OF_TROPHY
        stop_sign_3.scale.y = param.SIZE_OF_TROPHY
        stop_sign_3.scale.z = 0
        stop_sign_3.pose.position.x = param.MAP_3_STOP_LINE_X_3
        stop_sign_3.pose.position.y = param.MAP_3_STOP_LINE_Y_3 + 0.5
        stop_sign_3.pose.position.z = 0
        stop_sign_3.pose.orientation.x = 0.0
        stop_sign_3.pose.orientation.y = 0.0
        stop_sign_3.pose.orientation.z = 0.0
        stop_sign_3.pose.orientation.w = 1.0

        return traffic_stop, stop_sign, traffic_stop_2, stop_sign_2, traffic_stop_3, stop_sign_3

    elif map_number == 4:
        pass

    elif map_number == 5:
        pass
    
    else:
        rospy.loginfo("Map number is incorrect.")

def ParkinglotMarker():
    m = Marker()
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.color.r, m.color.g, m.color.b = 0, 0, 1
    m.color.a = 1
    m.scale.x = param.PARKING_LOT_WIDTH
    m.scale.y = param.PARKING_LOT_HEIGHT
    m.scale.z = 0.1
    m.pose.position.z = 0
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0

    return m
    
def MakeParkinglotMarker(map_number):
    if map_number == 1:
        tilt_degree = param.PARKING_LOT_TILT_DEGREE * pi / 180
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,tilt_degree)

        parking_lot_1 = ParkinglotMarker()
        parking_lot_1.header.frame_id = "map"
        parking_lot_1.ns = "lot_1"
        parking_lot_1.pose.position.x = param.MAP_1_PARKING_LOT_X_1
        parking_lot_1.pose.position.y = param.MAP_1_PARKING_LOT_Y_1
        parking_lot_1.pose.orientation.x = qu_x
        parking_lot_1.pose.orientation.y = qu_y
        parking_lot_1.pose.orientation.z = qu_z
        parking_lot_1.pose.orientation.w = qu_w

        parking_lot_2 = ParkinglotMarker()
        parking_lot_2.header.frame_id = "map"
        parking_lot_2.ns = "lot_2"
        parking_lot_2.pose.position.x = param.MAP_1_PARKING_LOT_X_2
        parking_lot_2.pose.position.y = param.MAP_1_PARKING_LOT_Y_2
        parking_lot_2.pose.orientation.x = qu_x
        parking_lot_2.pose.orientation.y = qu_y
        parking_lot_2.pose.orientation.z = qu_z
        parking_lot_2.pose.orientation.w = qu_w

        parking_lot_3 = ParkinglotMarker()
        parking_lot_3.header.frame_id = "map"
        parking_lot_3.ns = "lot_3"
        parking_lot_3.pose.position.x = param.MAP_1_PARKING_LOT_X_3
        parking_lot_3.pose.position.y = param.MAP_1_PARKING_LOT_Y_3
        parking_lot_3.pose.orientation.x = qu_x
        parking_lot_3.pose.orientation.y = qu_y
        parking_lot_3.pose.orientation.z = qu_z
        parking_lot_3.pose.orientation.w = qu_w

        parking_arrow_1 = Marker()

        parking_arrow_1.header.frame_id = "map"
        parking_arrow_1.ns = "arrow_1"
        parking_arrow_1.type = Marker.ARROW
        parking_arrow_1.action = Marker.ADD
        parking_arrow_1.color.r, parking_arrow_1.color.g, parking_arrow_1.color.b = 0, 0, 1
        parking_arrow_1.color.a = 1
        parking_arrow_1.scale.x = 1
        parking_arrow_1.scale.y = 0.1
        parking_arrow_1.scale.z = 0.1
        if param.MAP_1_PARKING_AREA == 1:
            parking_arrow_1.pose.position.x = param.MAP_1_PARKING_LOT_X_1
            parking_arrow_1.pose.position.y = param.MAP_1_PARKING_LOT_Y_1
            parking_arrow_1.pose.position.z = 0
            arrow_degree_1 = param.MAP_1_PARKING_LOT_YAW_1 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_1)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_1.color.r, parking_lot_1.color.g, parking_lot_1.color.b = 0, 1, 0
        elif param.MAP_1_PARKING_AREA == 2:
            parking_arrow_1.pose.position.x = param.MAP_1_PARKING_LOT_X_2
            parking_arrow_1.pose.position.y = param.MAP_1_PARKING_LOT_Y_2
            parking_arrow_1.pose.position.z = 0
            arrow_degree_2 = param.MAP_1_PARKING_LOT_YAW_2 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_2)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_2.color.r, parking_lot_2.color.g, parking_lot_2.color.b = 0, 1, 0
        elif param.MAP_1_PARKING_AREA == 3:
            parking_arrow_1.pose.position.x = param.MAP_1_PARKING_LOT_X_3
            parking_arrow_1.pose.position.y = param.MAP_1_PARKING_LOT_Y_3
            parking_arrow_1.pose.position.z = 0
            arrow_degree_3 = param.MAP_1_PARKING_LOT_YAW_3 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_3)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_3.color.r, parking_lot_3.color.g, parking_lot_3.color.b = 0, 1, 0

        return parking_lot_1, parking_lot_2, parking_lot_3, parking_arrow_1
    

    elif map_number == 2:
        tilt_degree = param.MAP_2_PARKING_LOT_TILT_DEGREE * pi / 180
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,tilt_degree)

        parking_lot_1 = ParkinglotMarker()
        parking_lot_1.header.frame_id = "map"
        parking_lot_1.ns = "lot_1"
        parking_lot_1.pose.position.x = param.MAP_2_PARKING_LOT_X_1
        parking_lot_1.pose.position.y = param.MAP_2_PARKING_LOT_Y_1
        parking_lot_1.pose.orientation.x = qu_x
        parking_lot_1.pose.orientation.y = qu_y
        parking_lot_1.pose.orientation.z = qu_z
        parking_lot_1.pose.orientation.w = qu_w

        parking_lot_2 = ParkinglotMarker()
        parking_lot_2.header.frame_id = "map"
        parking_lot_2.ns = "lot_2"
        parking_lot_2.pose.position.x = param.MAP_2_PARKING_LOT_X_2
        parking_lot_2.pose.position.y = param.MAP_2_PARKING_LOT_Y_2
        parking_lot_2.pose.orientation.x = qu_x
        parking_lot_2.pose.orientation.y = qu_y
        parking_lot_2.pose.orientation.z = qu_z
        parking_lot_2.pose.orientation.w = qu_w

        parking_lot_3 = ParkinglotMarker()
        parking_lot_3.header.frame_id = "map"
        parking_lot_3.ns = "lot_3"
        parking_lot_3.pose.position.x = param.MAP_2_PARKING_LOT_X_3
        parking_lot_3.pose.position.y = param.MAP_2_PARKING_LOT_Y_3
        parking_lot_3.pose.orientation.x = qu_x
        parking_lot_3.pose.orientation.y = qu_y
        parking_lot_3.pose.orientation.z = qu_z
        parking_lot_3.pose.orientation.w = qu_w

        parking_arrow_1 = Marker()

        parking_arrow_1.header.frame_id = "map"
        parking_arrow_1.ns = "arrow_1"
        parking_arrow_1.type = Marker.ARROW
        parking_arrow_1.action = Marker.ADD
        parking_arrow_1.color.r, parking_arrow_1.color.g, parking_arrow_1.color.b = 0, 0, 1
        parking_arrow_1.color.a = 1
        parking_arrow_1.scale.x = 1
        parking_arrow_1.scale.y = 0.1
        parking_arrow_1.scale.z = 0.1
        if param.MAP_2_PARKING_AREA == 1:
            parking_arrow_1.pose.position.x = param.MAP_2_PARKING_LOT_X_1
            parking_arrow_1.pose.position.y = param.MAP_2_PARKING_LOT_Y_1
            parking_arrow_1.pose.position.z = 0
            arrow_degree_1 = param.MAP_2_PARKING_LOT_YAW_1 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_1)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_1.color.r, parking_lot_1.color.g, parking_lot_1.color.b = 0, 1, 0
        elif param.MAP_2_PARKING_AREA == 2:
            parking_arrow_1.pose.position.x = param.MAP_2_PARKING_LOT_X_2
            parking_arrow_1.pose.position.y = param.MAP_2_PARKING_LOT_Y_2
            parking_arrow_1.pose.position.z = 0
            arrow_degree_2 = param.MAP_2_PARKING_LOT_YAW_2 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_2)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_2.color.r, parking_lot_2.color.g, parking_lot_2.color.b = 0, 1, 0
        elif param.MAP_2_PARKING_AREA == 3:
            parking_arrow_1.pose.position.x = param.MAP_2_PARKING_LOT_X_3
            parking_arrow_1.pose.position.y = param.MAP_2_PARKING_LOT_Y_3
            parking_arrow_1.pose.position.z = 0
            arrow_degree_3 = param.MAP_2_PARKING_LOT_YAW_3 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_3)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_3.color.r, parking_lot_3.color.g, parking_lot_3.color.b = 0, 1, 0

        return parking_lot_1, parking_lot_2, parking_lot_3, parking_arrow_1

    elif map_number == 3:
        tilt_degree = param.MAP_3_PARKING_LOT_TILT_DEGREE * pi / 180
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,tilt_degree)

        parking_lot_1 = ParkinglotMarker()
        parking_lot_1.header.frame_id = "map"
        parking_lot_1.ns = "lot_1"
        parking_lot_1.pose.position.x = param.MAP_3_PARKING_LOT_X_1
        parking_lot_1.pose.position.y = param.MAP_3_PARKING_LOT_Y_1
        parking_lot_1.pose.orientation.x = qu_x
        parking_lot_1.pose.orientation.y = qu_y
        parking_lot_1.pose.orientation.z = qu_z
        parking_lot_1.pose.orientation.w = qu_w

        parking_lot_2 = ParkinglotMarker()
        parking_lot_2.header.frame_id = "map"
        parking_lot_2.ns = "lot_2"
        parking_lot_2.pose.position.x = param.MAP_3_PARKING_LOT_X_2
        parking_lot_2.pose.position.y = param.MAP_3_PARKING_LOT_Y_2
        parking_lot_2.pose.orientation.x = qu_x
        parking_lot_2.pose.orientation.y = qu_y
        parking_lot_2.pose.orientation.z = qu_z
        parking_lot_2.pose.orientation.w = qu_w

        parking_lot_3 = ParkinglotMarker()
        parking_lot_3.header.frame_id = "map"
        parking_lot_3.ns = "lot_3"
        parking_lot_3.pose.position.x = param.MAP_3_PARKING_LOT_X_3
        parking_lot_3.pose.position.y = param.MAP_3_PARKING_LOT_Y_3
        parking_lot_3.pose.orientation.x = qu_x
        parking_lot_3.pose.orientation.y = qu_y
        parking_lot_3.pose.orientation.z = qu_z
        parking_lot_3.pose.orientation.w = qu_w

        parking_arrow_1 = Marker()

        parking_arrow_1.header.frame_id = "map"
        parking_arrow_1.ns = "arrow_1"
        parking_arrow_1.type = Marker.ARROW
        parking_arrow_1.action = Marker.ADD
        parking_arrow_1.color.r, parking_arrow_1.color.g, parking_arrow_1.color.b = 0, 0, 1
        parking_arrow_1.color.a = 1
        parking_arrow_1.scale.x = 1
        parking_arrow_1.scale.y = 0.1
        parking_arrow_1.scale.z = 0.1
        if param.MAP_3_PARKING_AREA == 1:
            parking_arrow_1.pose.position.x = param.MAP_3_PARKING_LOT_X_1
            parking_arrow_1.pose.position.y = param.MAP_3_PARKING_LOT_Y_1
            parking_arrow_1.pose.position.z = 0
            arrow_degree_1 = param.MAP_3_PARKING_LOT_YAW_1 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_1)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_1.color.r, parking_lot_1.color.g, parking_lot_1.color.b = 0, 1, 0
        elif param.MAP_3_PARKING_AREA == 2:
            parking_arrow_1.pose.position.x = param.MAP_3_PARKING_LOT_X_2
            parking_arrow_1.pose.position.y = param.MAP_3_PARKING_LOT_Y_2
            parking_arrow_1.pose.position.z = 0
            arrow_degree_2 = param.MAP_3_PARKING_LOT_YAW_2 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_2)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w
            parking_lot_2.color.r, parking_lot_2.color.g, parking_lot_2.color.b = 0, 1, 0
        elif param.MAP_3_PARKING_AREA == 3:
            parking_arrow_1.pose.position.x = param.MAP_3_PARKING_LOT_X_3
            parking_arrow_1.pose.position.y = param.MAP_3_PARKING_LOT_Y_3
            parking_arrow_1.pose.position.z = 0
            arrow_degree_3 = param.MAP_3_PARKING_LOT_YAW_3 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_3)
            parking_arrow_1.pose.orientation.x = qu_x
            parking_arrow_1.pose.orientation.y = qu_y
            parking_arrow_1.pose.orientation.z = qu_z
            parking_arrow_1.pose.orientation.w = qu_w

        return parking_lot_1, parking_lot_2, parking_lot_3, parking_arrow_1

    else:
        return None

def DelivMarker():
    m = Marker()
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.color.r, m.color.g, m.color.b = 0, 1, 1
    m.color.a = 1
    m.scale.x = param.PARKING_LOT_WIDTH
    m.scale.y = param.PARKING_LOT_HEIGHT
    m.scale.z = 0.1
    m.pose.position.z = 0
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0

    return m
    
def MakeDelivMarker(map_number):
    if map_number == 1:
        tilt_degree = -90

        deliv_pickup = DelivMarker()
        deliv_pickup.header.frame_id = "map"
        deliv_pickup.ns = "pickup"
        deliv_pickup.pose.position.x = param.MAP_1_DELIV_PICKUP_X
        deliv_pickup.pose.position.y = param.MAP_1_DELIV_PICKUP_Y
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_1_DELIV_PICKUP_YAW + tilt_degree) * pi / 180)
        deliv_pickup.pose.orientation.x = qu_x
        deliv_pickup.pose.orientation.y = qu_y
        deliv_pickup.pose.orientation.z = qu_z
        deliv_pickup.pose.orientation.w = qu_w

        deliv_dropoff_1 = DelivMarker()
        deliv_dropoff_1.header.frame_id = "map"
        deliv_dropoff_1.ns = "dropoff_1"
        deliv_dropoff_1.pose.position.x = param.MAP_1_DELIV_DROPOFF_X_1
        deliv_dropoff_1.pose.position.y = param.MAP_1_DELIV_DROPOFF_Y_1
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_1_DELIV_DROPOFF_YAW_1 + tilt_degree) * pi / 180)
        deliv_dropoff_1.pose.orientation.x = qu_x
        deliv_dropoff_1.pose.orientation.y = qu_y
        deliv_dropoff_1.pose.orientation.z = qu_z
        deliv_dropoff_1.pose.orientation.w = qu_w

        deliv_dropoff_2 = DelivMarker()
        deliv_dropoff_2.header.frame_id = "map"
        deliv_dropoff_2.ns = "dropoff_2"
        deliv_dropoff_2.pose.position.x = param.MAP_1_DELIV_DROPOFF_X_2
        deliv_dropoff_2.pose.position.y = param.MAP_1_DELIV_DROPOFF_Y_2
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_1_DELIV_DROPOFF_YAW_2 + tilt_degree) * pi / 180)
        deliv_dropoff_2.pose.orientation.x = qu_x
        deliv_dropoff_2.pose.orientation.y = qu_y
        deliv_dropoff_2.pose.orientation.z = qu_z
        deliv_dropoff_2.pose.orientation.w = qu_w

        deliv_dropoff_3 = DelivMarker()
        deliv_dropoff_3.header.frame_id = "map"
        deliv_dropoff_3.ns = "dropoff_3"
        deliv_dropoff_3.pose.position.x = param.MAP_1_DELIV_DROPOFF_X_3
        deliv_dropoff_3.pose.position.y = param.MAP_1_DELIV_DROPOFF_Y_3
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_1_DELIV_DROPOFF_YAW_3 + tilt_degree) * pi / 180)
        deliv_dropoff_3.pose.orientation.x = qu_x
        deliv_dropoff_3.pose.orientation.y = qu_y
        deliv_dropoff_3.pose.orientation.z = qu_z
        deliv_dropoff_3.pose.orientation.w = qu_w

        deliv_dropoff_arrow_1 = Marker()

        deliv_dropoff_arrow_1.header.frame_id = "map"
        deliv_dropoff_arrow_1.ns = "dropoff_arrow_1"
        deliv_dropoff_arrow_1.type = Marker.ARROW
        deliv_dropoff_arrow_1.action = Marker.ADD
        deliv_dropoff_arrow_1.color.r, deliv_dropoff_arrow_1.color.g, deliv_dropoff_arrow_1.color.b = 0, 0, 1
        deliv_dropoff_arrow_1.color.a = 1
        deliv_dropoff_arrow_1.scale.x = 1
        deliv_dropoff_arrow_1.scale.y = 0.1
        deliv_dropoff_arrow_1.scale.z = 0.1
        if param.MAP_1_PICKUP_DROPOFF_AREA == 1:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_1_DELIV_DROPOFF_X_1
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_1_DELIV_DROPOFF_Y_1
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_1 = param.MAP_1_DELIV_DROPOFF_YAW_1 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_1)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_1.color.r, deliv_dropoff_1.color.g, deliv_dropoff_1.color.b = 0, 1, 0
        elif param.MAP_1_PICKUP_DROPOFF_AREA == 2:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_1_DELIV_DROPOFF_X_2
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_1_DELIV_DROPOFF_Y_2
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_2 = param.MAP_1_DELIV_DROPOFF_YAW_2 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_2)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_2.color.r, deliv_dropoff_2.color.g, deliv_dropoff_2.color.b = 0, 1, 0
        elif param.MAP_1_PICKUP_DROPOFF_AREA == 3:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_1_DELIV_DROPOFF_X_3
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_1_DELIV_DROPOFF_Y_3
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_3 = param.MAP_1_DELIV_DROPOFF_YAW_3 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_3)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_3.color.r, deliv_dropoff_3.color.g, deliv_dropoff_3.color.b = 0, 1, 0

        return deliv_pickup, deliv_dropoff_1, deliv_dropoff_2, deliv_dropoff_3, deliv_dropoff_arrow_1
    
    elif map_number == 2:
        tilt_degree = -90

        deliv_pickup = DelivMarker()
        deliv_pickup.header.frame_id = "map"
        deliv_pickup.ns = "pickup"
        deliv_pickup.pose.position.x = param.MAP_2_DELIV_PICKUP_X
        deliv_pickup.pose.position.y = param.MAP_2_DELIV_PICKUP_Y
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_2_DELIV_PICKUP_YAW + tilt_degree) * pi / 180)
        deliv_pickup.pose.orientation.x = qu_x
        deliv_pickup.pose.orientation.y = qu_y
        deliv_pickup.pose.orientation.z = qu_z
        deliv_pickup.pose.orientation.w = qu_w

        deliv_dropoff_1 = DelivMarker()
        deliv_dropoff_1.header.frame_id = "map"
        deliv_dropoff_1.ns = "dropoff_1"
        deliv_dropoff_1.pose.position.x = param.MAP_2_DELIV_DROPOFF_X_1
        deliv_dropoff_1.pose.position.y = param.MAP_2_DELIV_DROPOFF_Y_1
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_2_DELIV_DROPOFF_YAW_1 + tilt_degree) * pi / 180)
        deliv_dropoff_1.pose.orientation.x = qu_x
        deliv_dropoff_1.pose.orientation.y = qu_y
        deliv_dropoff_1.pose.orientation.z = qu_z
        deliv_dropoff_1.pose.orientation.w = qu_w

        deliv_dropoff_2 = DelivMarker()
        deliv_dropoff_2.header.frame_id = "map"
        deliv_dropoff_2.ns = "dropoff_2"
        deliv_dropoff_2.pose.position.x = param.MAP_2_DELIV_DROPOFF_X_2
        deliv_dropoff_2.pose.position.y = param.MAP_2_DELIV_DROPOFF_Y_2
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_2_DELIV_DROPOFF_YAW_2 + tilt_degree) * pi / 180)
        deliv_dropoff_2.pose.orientation.x = qu_x
        deliv_dropoff_2.pose.orientation.y = qu_y
        deliv_dropoff_2.pose.orientation.z = qu_z
        deliv_dropoff_2.pose.orientation.w = qu_w

        deliv_dropoff_3 = DelivMarker()
        deliv_dropoff_3.header.frame_id = "map"
        deliv_dropoff_3.ns = "dropoff_3"
        deliv_dropoff_3.pose.position.x = param.MAP_2_DELIV_DROPOFF_X_3
        deliv_dropoff_3.pose.position.y = param.MAP_2_DELIV_DROPOFF_Y_3
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_2_DELIV_DROPOFF_YAW_3 + tilt_degree) * pi / 180)
        deliv_dropoff_3.pose.orientation.x = qu_x
        deliv_dropoff_3.pose.orientation.y = qu_y
        deliv_dropoff_3.pose.orientation.z = qu_z
        deliv_dropoff_3.pose.orientation.w = qu_w

        deliv_dropoff_arrow_1 = Marker()

        deliv_dropoff_arrow_1.header.frame_id = "map"
        deliv_dropoff_arrow_1.ns = "dropoff_arrow_1"
        deliv_dropoff_arrow_1.type = Marker.ARROW
        deliv_dropoff_arrow_1.action = Marker.ADD
        deliv_dropoff_arrow_1.color.r, deliv_dropoff_arrow_1.color.g, deliv_dropoff_arrow_1.color.b = 0, 0, 1
        deliv_dropoff_arrow_1.color.a = 1
        deliv_dropoff_arrow_1.scale.x = 1
        deliv_dropoff_arrow_1.scale.y = 0.1
        deliv_dropoff_arrow_1.scale.z = 0.1
        if param.MAP_2_PICKUP_DROPOFF_AREA == 1:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_2_DELIV_DROPOFF_X_1
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_2_DELIV_DROPOFF_Y_1
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_1 = param.MAP_2_DELIV_DROPOFF_YAW_1 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_1)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_1.color.r, deliv_dropoff_1.color.g, deliv_dropoff_1.color.b = 0, 1, 0
        elif param.MAP_2_PICKUP_DROPOFF_AREA == 2:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_2_DELIV_DROPOFF_X_2
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_2_DELIV_DROPOFF_Y_2
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_2 = param.MAP_2_DELIV_DROPOFF_YAW_2 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_2)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_2.color.r, deliv_dropoff_2.color.g, deliv_dropoff_2.color.b = 0, 1, 0
        elif param.MAP_2_PICKUP_DROPOFF_AREA == 3:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_2_DELIV_DROPOFF_X_3
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_2_DELIV_DROPOFF_Y_3
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_3 = param.MAP_2_DELIV_DROPOFF_YAW_3 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_3)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_3.color.r, deliv_dropoff_3.color.g, deliv_dropoff_3.color.b = 0, 1, 0

        return deliv_pickup, deliv_dropoff_1, deliv_dropoff_2, deliv_dropoff_3, deliv_dropoff_arrow_1
    
    elif map_number == 3:
        tilt_degree = -90

        deliv_pickup = DelivMarker()
        deliv_pickup.header.frame_id = "map"
        deliv_pickup.ns = "pickup"
        deliv_pickup.pose.position.x = param.MAP_3_DELIV_PICKUP_X
        deliv_pickup.pose.position.y = param.MAP_3_DELIV_PICKUP_Y
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_3_DELIV_PICKUP_YAW + tilt_degree) * pi / 180)
        deliv_pickup.pose.orientation.x = qu_x
        deliv_pickup.pose.orientation.y = qu_y
        deliv_pickup.pose.orientation.z = qu_z
        deliv_pickup.pose.orientation.w = qu_w

        deliv_dropoff_1 = DelivMarker()
        deliv_dropoff_1.header.frame_id = "map"
        deliv_dropoff_1.ns = "dropoff_1"
        deliv_dropoff_1.pose.position.x = param.MAP_3_DELIV_DROPOFF_X_1
        deliv_dropoff_1.pose.position.y = param.MAP_3_DELIV_DROPOFF_Y_1
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_3_DELIV_DROPOFF_YAW_1 + tilt_degree) * pi / 180)
        deliv_dropoff_1.pose.orientation.x = qu_x
        deliv_dropoff_1.pose.orientation.y = qu_y
        deliv_dropoff_1.pose.orientation.z = qu_z
        deliv_dropoff_1.pose.orientation.w = qu_w

        deliv_dropoff_2 = DelivMarker()
        deliv_dropoff_2.header.frame_id = "map"
        deliv_dropoff_2.ns = "dropoff_2"
        deliv_dropoff_2.pose.position.x = param.MAP_3_DELIV_DROPOFF_X_2
        deliv_dropoff_2.pose.position.y = param.MAP_3_DELIV_DROPOFF_Y_2
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_3_DELIV_DROPOFF_YAW_2 + tilt_degree) * pi / 180)
        deliv_dropoff_2.pose.orientation.x = qu_x
        deliv_dropoff_2.pose.orientation.y = qu_y
        deliv_dropoff_2.pose.orientation.z = qu_z
        deliv_dropoff_2.pose.orientation.w = qu_w

        deliv_dropoff_3 = DelivMarker()
        deliv_dropoff_3.header.frame_id = "map"
        deliv_dropoff_3.ns = "dropoff_3"
        deliv_dropoff_3.pose.position.x = param.MAP_3_DELIV_DROPOFF_X_3
        deliv_dropoff_3.pose.position.y = param.MAP_3_DELIV_DROPOFF_Y_3
        qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,(param.MAP_3_DELIV_DROPOFF_YAW_3 + tilt_degree) * pi / 180)
        deliv_dropoff_3.pose.orientation.x = qu_x
        deliv_dropoff_3.pose.orientation.y = qu_y
        deliv_dropoff_3.pose.orientation.z = qu_z
        deliv_dropoff_3.pose.orientation.w = qu_w

        deliv_dropoff_arrow_1 = Marker()

        deliv_dropoff_arrow_1.header.frame_id = "map"
        deliv_dropoff_arrow_1.ns = "dropoff_arrow_1"
        deliv_dropoff_arrow_1.type = Marker.ARROW
        deliv_dropoff_arrow_1.action = Marker.ADD
        deliv_dropoff_arrow_1.color.r, deliv_dropoff_arrow_1.color.g, deliv_dropoff_arrow_1.color.b = 0, 0, 1
        deliv_dropoff_arrow_1.color.a = 1
        deliv_dropoff_arrow_1.scale.x = 1
        deliv_dropoff_arrow_1.scale.y = 0.1
        deliv_dropoff_arrow_1.scale.z = 0.1
        if param.MAP_3_PICKUP_DROPOFF_AREA == 1:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_3_DELIV_DROPOFF_X_1
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_3_DELIV_DROPOFF_Y_1
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_1 = param.MAP_3_DELIV_DROPOFF_YAW_1 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_1)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_1.color.r, deliv_dropoff_1.color.g, deliv_dropoff_1.color.b = 0, 1, 0
        elif param.MAP_3_PICKUP_DROPOFF_AREA == 2:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_3_DELIV_DROPOFF_X_2
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_3_DELIV_DROPOFF_Y_2
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_2 = param.MAP_3_DELIV_DROPOFF_YAW_2 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_2)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_2.color.r, deliv_dropoff_2.color.g, deliv_dropoff_2.color.b = 0, 1, 0
        elif param.MAP_3_PICKUP_DROPOFF_AREA == 3:
            deliv_dropoff_arrow_1.pose.position.x = param.MAP_3_DELIV_DROPOFF_X_3
            deliv_dropoff_arrow_1.pose.position.y = param.MAP_3_DELIV_DROPOFF_Y_3
            deliv_dropoff_arrow_1.pose.position.z = 0
            arrow_degree_3 = param.MAP_3_DELIV_DROPOFF_YAW_3 * pi / 180
            qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,arrow_degree_3)
            deliv_dropoff_arrow_1.pose.orientation.x = qu_x
            deliv_dropoff_arrow_1.pose.orientation.y = qu_y
            deliv_dropoff_arrow_1.pose.orientation.z = qu_z
            deliv_dropoff_arrow_1.pose.orientation.w = qu_w
            deliv_dropoff_3.color.r, deliv_dropoff_3.color.g, deliv_dropoff_3.color.b = 0, 1, 0

        return deliv_pickup, deliv_dropoff_1, deliv_dropoff_2, deliv_dropoff_3, deliv_dropoff_arrow_1

    else:
        return None

if __name__ == "__main__":
    rospy.init_node("Visualize_node")
    visual_pub = rospy.Publisher('marker_array', MarkerArray, queue_size=1)
    rate = rospy.Rate(1)
    # Get mission number
    map_number = rospy.get_param('~map_number')

    goal_marker = MakeGoalMarker(map_number)
    traffic_marker = MakeTrafficMarker(map_number)
    parking_marker = MakeParkinglotMarker(map_number)
    deliv_marker = MakeDelivMarker(map_number)
    # if map_number == 2:
    #     traffic_marker = MakeTrafficMarker(map_number)
    # if map_number == 3:
    #     traffic_marker = MakeTrafficMarker(map_number)
    #     parking_marker = MakeParkinglotMarker()

    while not rospy.is_shutdown():

        mkarray_msg = MarkerArray()
        temp_list = []
        temp_list.append(goal_marker)

        if traffic_marker is not None:
            for i in traffic_marker:
                temp_list.append(i)

        if parking_marker is not None:
            for i in parking_marker:
                temp_list.append(i)

        if deliv_marker is not None:
            for i in deliv_marker:
                temp_list.append(i)
        # if map_number == 2:
        #     for i in traffic_marker:
        #         temp_list.append(i)
        # elif map_number == 3:
        #     for i in traffic_marker:
        #         temp_list.append(i)
        #     for i in parking_marker:
        #         temp_list.append(i)

        if len(temp_list) != 0:
            mkarray_msg = temp_list

            visual_pub.publish(mkarray_msg)
        
        else:
            pass

        rate.sleep()
