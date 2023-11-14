#!/usr/bin/env python3

import rospy
import math
import numpy as np
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import math

from parameter_list import Param
from check_collision import end_detection
from racecar_simulator.msg import CenterPose
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan

pi = math.pi
param = Param()
odom_log = deque([])

check_array = np.zeros(shape=360)

for i in range(0, 26):
    check_array[i] = param.REAR_LIDAR / np.cos(i*pi/180)

for i in range(26, 123):
    check_array[i] = param.WIDTH / np.sin(i*pi/180)

for i in range(123, 181):
    check_array[i] = - (param.WHEELBASE - param.REAR_LIDAR) / np.cos(i*pi/180)

for i in range(181, 360):
    check_array[i] = check_array[360-i]

class JerkMission:
    def __init__(self):
        rospy.init_node('jerk_mission', anonymous=True)
        self.odom_sub = rospy.Subscriber('/car_center', CenterPose, self.odomcb)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scancb)
        self.drive_pub = rospy.Publisher('drive', AckermannDrive, queue_size=10)
        self.cur_pose = None
        self.start_time = rospy.Time.now().to_sec()
        self.time = 0
        self.start_pt = 0
        self.start_cnt = 1
        self.lidar_data = None
        self.end_flag = False

    def odomcb(self, data):
        if self.start_cnt:
            self.start_pt = data.pose
            self.start_cnt = 0

        self.cur_pose = data.pose

        if self.lidar_data is not None:
            if collision_detection(self.lidar_data):
                if distance(self.cur_pose[0],self.cur_pose[1], param.END_POINT_X_4, param.END_POINT_Y_4)<0.5:
                    rospy.loginfo("passed!!")
                    pass
                else:
                    odom_log.clear()
                    print("Restart!")
                self.start_time = rospy.Time.now().to_sec()
        
        self.time = rospy.Time.now().to_sec() - self.start_time
        odom_log.append([self.cur_pose[:2], self.time])
        # print(self.cur_pose)
        # print(self.time)
        return None
    
    def scancb(self, data):
        self.lidar_data = np.array(data.ranges)
        if self.cur_pose is not None:
            if end_detection(self.cur_pose[:2],4) or self.end_flag:
                self.end_flag = True
                stop_msg = AckermannDrive()
                stop_msg.steering_angle = 0
                stop_msg.speed = 0
                jerk.drive_pub.publish(stop_msg)
                
            # else:
            #     side1 = self.lidar_data[89]
            #     side2 = self.lidar_data[269]

            #     drive_msg = AckermannDrive()
            #     drive_msg.speed = 2
            #     drive_msg.steering_angle = (side2 - side1)* 3
            #     self.drive_pub.publish(drive_msg)
    
def result_viz(odom_log,ref_pt_list):
    # result = velocity_calculation(odom_log)
    # jerk_scoring(result)
    frenet_result = convert_frame(odom_log, ref_pt_list)
    frenet_jerk_scoring(frenet_result)
        

        

def collision_detection(lidar_data):
    min_distance = np.min(np.array(lidar_data) - np.array(check_array))
    if min_distance <= 0.01:
        return True
    else:
        return False

def velocity_calculation(odom_log):
    init_run=True
    vlog = deque([])
    odom_log.popleft()
    log = []
    
    while odom_log:
        if init_run:
            init_run=False
            prev = ((0,0),0)
        
        now = odom_log.popleft()
        xn = now[0][0]
        yn = now[0][1]
        tn = now[1]
        xp = prev[0][0]
        yp = prev[0][1]
        tp = prev[1]
        dx = xn-xp
        dy = yn-yp
        dt = tn-tp
        v=dx/dt+dy/dt
        vlog.append([v,tn])
        log.append([str(xn),str(yn)])
        prev = now
    # print('write start')
    # with open('/home/ximp/log.txt', 'w') as f:
    #     for item in log:
    #         f.write(f'[{item[0]},{item[1]}]')
    # print('write done')
    return vlog

def distance(x1,y1,x2,y2):
    result = math.sqrt(pow(x2-x1,2)+pow(y2-y1,2))
    return result

def closest_ref_pt(cart_pt, ref_x, ref_y):
    closest_len = float('inf')
    o_nClosestRefPoint = 0
    x = cart_pt[0]
    y = cart_pt[1]

    for i in range(0,len(ref_pt_list)):    
        dist = distance(x,y,ref_x[i],ref_y[i])
        if dist < closest_len:
            closest_len = dist
            o_nClosestRefPoint = i
        else:
            break
    
    if o_nClosestRefPoint == len(ref_pt_list):
        o_nClosestScdRefPoint = o_nClosestRefPoint -1
    elif o_nClosestRefPoint == 0:
        o_nClosestScdRefPoint = o_nClosestRefPoint +1
    else:
        ref_x_p1 = ref_x[o_nClosestRefPoint+1]
        ref_y_p1 = ref_y[o_nClosestRefPoint+1]
        dist_p1 = distance(x,y,ref_x_p1,ref_y_p1)

        ref_x_m1 = ref_x[o_nClosestRefPoint+1]
        ref_y_m1 = ref_y[o_nClosestRefPoint+1]
        dist_m1 = distance(x,y,ref_x_m1,ref_y_m1)

        if dist_m1 < dist_p1:
            o_nClosestScdRefPoint = o_nClosestRefPoint -1
        else:
            o_nClosestScdRefPoint = o_nClosestRefPoint +1

    return o_nClosestRefPoint, o_nClosestScdRefPoint

def cart2frenet(cart_pt, ref_x, ref_y):

    x = cart_pt[0]
    y = cart_pt[1]

    n1close_ref_pt, n2close_ref_pt = closest_ref_pt(cart_pt, ref_x, ref_y)

    if n1close_ref_pt > n2close_ref_pt:
        nNext_ref_pt = n1close_ref_pt
    else:
        nNext_ref_pt = n2close_ref_pt
    
    nPrev_ref_pt = nNext_ref_pt -1
    if nNext_ref_pt == 0:
        nPrev_ref_pt = 0
        nNext_ref_pt = 1
    
    tanx = ref_pt_list[nNext_ref_pt][0]-ref_pt_list[nPrev_ref_pt][0]
    tany = ref_pt_list[nNext_ref_pt][1]-ref_pt_list[nPrev_ref_pt][1]

    vecx = x - ref_pt_list[nPrev_ref_pt][0]
    vecy = y - ref_pt_list[nPrev_ref_pt][1]

    tan_len = math.sqrt(pow(tanx,2)+pow(tany,2))
    projected_norm_vec = (vecx*tanx + vecy*tany)/tan_len
    proj_vecx = projected_norm_vec*tanx/tan_len
    proj_vecy = projected_norm_vec*tany/tan_len

    o_D = distance(vecx,vecy,proj_vecx,proj_vecy)

    X1 = ref_pt_list[nPrev_ref_pt][0]
    Y1 = ref_pt_list[nPrev_ref_pt][1]
    X2 = ref_pt_list[nNext_ref_pt][0]
    Y2 = ref_pt_list[nNext_ref_pt][1]

    D = (x-X1)*(Y2-Y1)-(y-Y1)*(X2-X1)
    side = np.sign(D)

    if side > 0:
        o_D = o_D * -1
    
    o_S = 0
    for i in range(0,nPrev_ref_pt):
        o_S = o_S + distance(ref_x[i],ref_y[i],ref_x[i+1],ref_y[i+1])
    
    o_S = o_S + projected_norm_vec

    return o_S, o_D
    
def convert_frame(odom,ref_pt_list):
    frenet_log = []
    ref_x = []
    ref_y = []
    for i in range(0,len(ref_pt_list)):
        ref_x.append(ref_pt_list[i][0])
        ref_y.append(ref_pt_list[i][1])

    for pos_t in odom:
        pos = pos_t[0]
        t = pos_t[1]
        s,d = cart2frenet(pos,ref_x, ref_y)
        frenet_log.append([(s,d),t])
    return frenet_log

def frenet_jerk_scoring(frenet_log):
    slist = []
    dlist = []
    tlist = []

    for i in range(0,len(frenet_log)):
        item = frenet_log[i]
        s = item[0][0]
        d = item[0][1]
        t = item[1]
        slist.append(s)
        dlist.append(d)
        tlist.append(t)
    
    poly_s = np.polyfit(tlist, slist, 7)
    poly_d = np.polyfit(tlist, dlist, 7)
    t_line = np.linspace(tlist[0], tlist[-1], 100)
    
    s_pred = np.zeros_like(t_line)
    d_pred = np.zeros_like(t_line)

    #Quintic+2 Poly
    s_pred = poly_s[0]*t_line**7 + poly_s[1]*t_line**6 + poly_s[2]*t_line**5 + poly_s[3]*t_line**4 + poly_s[4]*t_line**3 + poly_s[5]*t_line**2 + poly_s[6]*t_line + poly_s[7]
    d_pred = poly_d[0]*t_line**7 + poly_d[1]*t_line**6 + poly_d[2]*t_line**5 + poly_d[3]*t_line**4 + poly_d[4]*t_line**3 + poly_d[5]*t_line**2 + poly_d[6]*t_line + poly_d[7]
    
    #Quartic+2 Poly
    s_d = 7*poly_s[0]*t_line**6 + 6*poly_s[1]*t_line**5 + 5*poly_s[2]*t_line**4 + 4*poly_s[3]*t_line**3 + 3*poly_s[4]*t_line**2 + 2*poly_s[5]*t_line + poly_s[6]
    d_d = 7*poly_d[0]*t_line**6 + 6*poly_d[1]*t_line**5 + 5*poly_d[2]*t_line**4 + 4*poly_d[3]*t_line**3 + 3*poly_d[4]*t_line**2 + 2*poly_d[5]*t_line + poly_d[6]

    #Cubic+2 Poly
    s_dd = 42*poly_s[0]*t_line**5 + 30*poly_s[1]*t_line**4 + 20*poly_s[2]*t_line**3 + 12*poly_s[3]*t_line**2 + 6*poly_s[4]*t_line + poly_s[5]
    d_dd = 42*poly_d[0]*t_line**5 + 30*poly_d[1]*t_line**4 + 20*poly_d[2]*t_line**3 + 12*poly_d[3]*t_line**2 + 6*poly_d[4]*t_line + poly_d[5]

    #Quadratic+2 Poly - JERK
    s_ddd = 210*poly_s[0]*t_line**4 + 120*poly_s[1]*t_line**3 + 60*poly_s[2]*t_line**2 + 24*poly_s[3]*t_line + 6*poly_s[4]
    d_ddd = 210*poly_d[0]*t_line**4 + 120*poly_d[1]*t_line**3 + 60*poly_d[2]*t_line**2 + 24*poly_d[3]*t_line + 6*poly_d[4]

    sj_total = 0
    for sj in s_ddd:
        sj_total += abs(sj)
    
    dj_total = 0
    for dj in d_ddd:
        dj_total += abs(dj)
    
    jerk_total = sj_total+dj_total

    print("Jerk Total: ",jerk_total)

    plt.subplot(1,2,1)
    plt.title('Logitudinal')
    plt.scatter(tlist, slist, color = 'r')
    plt.plot(t_line, s_pred, color = 'g')
    plt.plot(t_line, s_d, color = 'b')
    plt.plot(t_line, s_dd, color = 'c')
    plt.plot(t_line, s_ddd, color = 'y')
    
    plt.subplot(1,2,2)
    plt.title('Lateral')
    plt.scatter(tlist, dlist, color = 'r')
    plt.plot(t_line, d_pred, color = 'g')
    plt.plot(t_line, d_d, color = 'b')
    plt.plot(t_line, d_dd, color = 'c')
    plt.plot(t_line, d_ddd, color = 'y')

    plt.show()

# def jerk_scoring(vel_log):
#     time_list = []
#     vel_list = []
#     for item in vel_log:
#         v = item[0]
#         t = item[1]
#         time_list.append(t)
#         vel_list.append(v)
    
#     poly = np.polyfit(time_list, vel_list, 4)
#     t_line = np.linspace(time_list[0], time_list[-1], 100)

#     v_pred = np.zeros_like(t_line)

#     v_pred = poly[0]*t_line**4 + poly[1]*t_line**3 + poly[2]*t_line**2 + poly[3]*t_line + poly[4]
    
#     #first derivative - acceleration
#     acc_pred = 4*poly[0]*t_line**3 + 3*poly[1]*t_line**2 + 2*poly[2]*t_line + poly[3]

#     #second derivative - jerk!
#     jerk_pred = 12*poly[0]*t_line**2 + 6*poly[1]*t_line + 2*poly[2]

#     jerk_total = 0
#     for t in t_line:
#         jerk_total += abs(12*poly[0]*t**2 + 6*poly[1]*t + 2*poly[2])
    
#     print("jerk total: ",jerk_total)

#     plt.plot(t_line, v_pred, color = 'b')
#     plt.plot(t_line, acc_pred, color = 'violet')
#     plt.plot(t_line, jerk_pred, color = 'g')
#     plt.scatter(time_list, vel_list, color = 'r')
#     plt.show()




if __name__ == "__main__":

    # path = str(Path(__file__).parent) + '/log.txt'
    # f = open(path, 'r')
    # line = f.readline()
    # pairs_str = line.strip('[]').split('][')
    # pairs_list = [list(map(float,pair.split(','))) for pair in pairs_str]
    ref_pt_list = []
    long_list = list(np.linspace(0, 35, 200))
    for i in long_list:
        ref_pt_list.append([i,0])
    
    try:
        while not rospy.is_shutdown():
            try:
                jerk = JerkMission()
                rospy.spin()

            except :
                continue

    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")
    
    if len(odom_log) ==0:
        print("odom log does not exist")
    if len(ref_pt_list) ==0:
        print("reference does not exist")
    result_viz(odom_log,ref_pt_list)
