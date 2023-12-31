from visualization_msgs.msg import Marker, MarkerArray

# (Map 1) 주차 공간 (1, 2, 3)
MAP_1_PARKING_AREA = 1
# (Map 1) 배달 공간 (1, 2, 3)
MAP_1_DROPOFF_AREA = 1
# (Map 2) 주차 공간 (1, 2, 3)
MAP_2_PARKING_AREA = 1
# (Map 2) 배달 공간 (1, 2, 3)
MAP_2_DROPOFF_AREA = 1
# (Map 3) 주차 공간 (1, 2, 3)
MAP_3_PARKING_AREA = 1
# (Map 3) 배달 공간 (1, 2, 3)
MAP_3_DROPOFF_AREA = 1


class Param():
    def __init__(self):
        # Information of a car
        self.car_angular_velocity = 1
        self.car_acceleration = 1
        self.car_jerk = 0
        self.WHEELBASE = 0.425
        self.REAR_LIDAR = 0.325
        self.WIDTH = 0.145
        self.SIZE_OF_TROPHY = 0.5

        # Endpoint of Map 1
        self.END_POINT_X_1 = 33.580
        self.END_POINT_Y_1 = -21.507
        # Endpoint of Map 2
        self.END_POINT_X_2 = -11.778
        self.END_POINT_Y_2 = 1.755
        # Endpoint of Map 3
        self.END_POINT_X_3 = -49.668
        self.END_POINT_Y_3 = -39.325
        # Endpoint of Map 4
        self.END_POINT_X_4 = 31.28
        self.END_POINT_Y_4 = -1.31
        # Endpoint of Map 5
        self.END_POINT_X_5 = 35.910
        self.END_POINT_Y_5 = -22.693

        # Size of stop lane
        self.STOP_LINE_SIZE = 0.8

        # Size of parking lot
        self.PARKING_LOT_WIDTH = 0.55
        self.PARKING_LOT_HEIGHT = 0.8
        
        # MAP 1
        # ===================================================================================
        self.STOP_LINE_TIME = 5

        # Center of stop line
        self.MAP_1_STOP_LINE_X_1 = 10.924
        self.MAP_1_STOP_LINE_Y_1 = -0.0088
        self.MAP_1_STOP_LINE_YAW_1 = 0
        self.MAP_1_STOP_LINE_X_2 = 11.698
        self.MAP_1_STOP_LINE_Y_2 = -19.560
        self.MAP_1_STOP_LINE_YAW_2 = -90

        # Check point of left or right road
        self.MAP_1_CHECK_X_1 = -6.02
        self.MAP_1_CHECK_Y_1 = -15.87
        self.MAP_1_CHECK_X_2 = -1.89
        self.MAP_1_CHECK_Y_2 = -15.87

        # Center of delivery
        self.MAP_1_DELIV_PICKUP_X = 4.400
        self.MAP_1_DELIV_PICKUP_Y = -0.875
        self.MAP_1_DELIV_PICKUP_YAW = 0
        self.MAP_1_DELIV_DROPOFF_X_1 = 19.151
        self.MAP_1_DELIV_DROPOFF_Y_1 = -22.289
        self.MAP_1_DELIV_DROPOFF_YAW_1 = 0
        self.MAP_1_DELIV_DROPOFF_X_2 = 20.451
        self.MAP_1_DELIV_DROPOFF_Y_2 = -22.289
        self.MAP_1_DELIV_DROPOFF_YAW_2 = 0
        self.MAP_1_DELIV_DROPOFF_X_3 = 21.751
        self.MAP_1_DELIV_DROPOFF_Y_3 = -22.289
        self.MAP_1_DELIV_DROPOFF_YAW_3 = 0

        # Random area of parking
        self.MAP_1_PARKING_AREA = MAP_1_PARKING_AREA

        # Random area of delivery dropoff
        self.MAP_1_PICKUP_DROPOFF_AREA = MAP_1_DROPOFF_AREA

        # Center point of parking lot in MAP 1
        self.MAP_1_PARKING_LOT_X_1 = 10.991
        self.MAP_1_PARKING_LOT_Y_1 = -5.887
        self.MAP_1_PARKING_LOT_YAW_1 = -135

        self.MAP_1_PARKING_LOT_X_2 = 10.991
        self.MAP_1_PARKING_LOT_Y_2 = -7.187
        self.MAP_1_PARKING_LOT_YAW_2 = -135

        self.MAP_1_PARKING_LOT_X_3 = 10.991
        self.MAP_1_PARKING_LOT_Y_3 = -8.587
        self.MAP_1_PARKING_LOT_YAW_3 = -135

        # Tilt degree of parking lot in MAP 1
        self.PARKING_LOT_TILT_DEGREE = 135

        
        # MAP 2
        # ===================================================================================
        
        # Center of stop line
        self.MAP_2_STOP_LINE_X_1 = 17.5
        self.MAP_2_STOP_LINE_Y_1 = -0.0088
        self.MAP_2_STOP_LINE_YAW_1 = 0
        self.MAP_2_STOP_LINE_X_2 = 32.345
        self.MAP_2_STOP_LINE_Y_2 = 0.00
        self.MAP_2_STOP_LINE_YAW_2 = 0
        self.MAP_2_STOP_LINE_X_3 = 52.732
        self.MAP_2_STOP_LINE_Y_3 = 5.494
        self.MAP_2_STOP_LINE_YAW_3 = -90
        self.MAP_2_STOP_LINE_X_4 = 39.193
        self.MAP_2_STOP_LINE_Y_4 = 1.582
        self.MAP_2_STOP_LINE_YAW_4 = 180
        self.MAP_2_STOP_LINE_X_5 = 23.145
        self.MAP_2_STOP_LINE_Y_5 = 1.667
        self.MAP_2_STOP_LINE_YAW_5 = 180

        self.MAP_2_DELIV_PICKUP_X = 7.1
        self.MAP_2_DELIV_PICKUP_Y = -1.575
        self.MAP_2_DELIV_PICKUP_YAW = 0
        self.MAP_2_DELIV_DROPOFF_X_1 = -4.754
        self.MAP_2_DELIV_DROPOFF_Y_1 = 2.812
        self.MAP_2_DELIV_DROPOFF_YAW_1 = 180
        self.MAP_2_DELIV_DROPOFF_X_2 = -5.954
        self.MAP_2_DELIV_DROPOFF_Y_2 = 2.812
        self.MAP_2_DELIV_DROPOFF_YAW_2 = 180
        self.MAP_2_DELIV_DROPOFF_X_3 = -7.154
        self.MAP_2_DELIV_DROPOFF_Y_3 = 2.812
        self.MAP_2_DELIV_DROPOFF_YAW_3 = 180

        # Center point of parking lot in MAP 1
        self.MAP_2_PARKING_LOT_X_1 = 7.333
        self.MAP_2_PARKING_LOT_Y_1 = 2.801
        self.MAP_2_PARKING_LOT_YAW_1 = 135

        self.MAP_2_PARKING_LOT_X_2 = 6.133
        self.MAP_2_PARKING_LOT_Y_2 = 2.801
        self.MAP_2_PARKING_LOT_YAW_2 = 135

        self.MAP_2_PARKING_LOT_X_3 = 4.933
        self.MAP_2_PARKING_LOT_Y_3 = 2.801
        self.MAP_2_PARKING_LOT_YAW_3 = 135

        # Tilt degree of parking lot in MAP 1
        self.MAP_2_PARKING_LOT_TILT_DEGREE = 45
        
        # Random area of parking
        self.MAP_2_PARKING_AREA = MAP_2_PARKING_AREA

        # Random area of delivery dropoff
        self.MAP_2_PICKUP_DROPOFF_AREA = MAP_2_DROPOFF_AREA

        # MAP 3
        # ===================================================================================
        self.MAP_3_STOP_LINE_X_1 = 10.3520
        self.MAP_3_STOP_LINE_Y_1 = -10.2330
        self.MAP_3_STOP_LINE_YAW_1 = -87.0894
        self.MAP_3_STOP_LINE_X_2 = 10.7530
        self.MAP_3_STOP_LINE_Y_2 = -25.9910
        self.MAP_3_STOP_LINE_YAW_2 = -88.1929       
        self.MAP_3_STOP_LINE_X_3 = -15.9716
        self.MAP_3_STOP_LINE_Y_3 = -47.6395
        self.MAP_3_STOP_LINE_YAW_3 = -94.5736

        self.MAP_3_DELIV_PICKUP_X = 9.5924
        self.MAP_3_DELIV_PICKUP_Y = -16.4317
        self.MAP_3_DELIV_PICKUP_YAW = -87.0895
        self.MAP_3_DELIV_DROPOFF_X_1 = 0.4334
        self.MAP_3_DELIV_DROPOFF_Y_1 = -26.5162
        self.MAP_3_DELIV_DROPOFF_YAW_1 = -178.4866
        self.MAP_3_DELIV_DROPOFF_X_2 = -1.5747
        self.MAP_3_DELIV_DROPOFF_Y_2 = -26.5721
        self.MAP_3_DELIV_DROPOFF_YAW_2 = -178.2353
        self.MAP_3_DELIV_DROPOFF_X_3 = -3.6124
        self.MAP_3_DELIV_DROPOFF_Y_3 = -26.6281
        self.MAP_3_DELIV_DROPOFF_YAW_3 = -178.5039

        # Center point of parking lot in MAP 3
        self.MAP_3_PARKING_LOT_X_1 = -16.6074
        self.MAP_3_PARKING_LOT_Y_1 = -39.0546
        self.MAP_3_PARKING_LOT_YAW_1 = -140.2195

        self.MAP_3_PARKING_LOT_X_2 = -16.6655
        self.MAP_3_PARKING_LOT_Y_2 = -40.6006
        self.MAP_3_PARKING_LOT_YAW_2 = -145.4144

        self.MAP_3_PARKING_LOT_X_3 = -16.7379
        self.MAP_3_PARKING_LOT_Y_3 = -42.0653
        self.MAP_3_PARKING_LOT_YAW_3 = -150.8894

        self.MAP_3_PARKING_LOT_TILT_DEGREE = 135

        # Random area of parking
        self.MAP_3_PARKING_AREA = MAP_3_PARKING_AREA

        # Random area of delivery dropoff
        self.MAP_3_PICKUP_DROPOFF_AREA = MAP_3_DROPOFF_AREA

        # Spawn lists (x, y, yaw - degree)
        self.MAP_1_SPAWN_POINT = [(0,0,0), (7.4446,0,0), (10.924,0,0), (11.7528,-11.8417,-90), (11.715,-20.157,-90), (25.2072,-21.5755,0)]
        self.MAP_2_SPAWN_POINT = [(0,0,0), (10.2264,0,0), (17.6167,0,0), (32.5436,0,0), (52.7464,5.37543,-90), (39.05728,1.5946,180), (22.94111,1.63984,180), (1.58777,1.78412,180), (-10.4700,1.73260,180)]
        self.MAP_3_SPAWN_POINT = [(0,0,0), (10.380,-10.700,-87.0894), (10.6453,-19.5551,-87.0895), (10.7380,-25.9737,-88.1929), (-6.865,-27.5207,-178.2), (-15.9061,-45.2137,-94.5), (-15.9783,-47.7374,-94.5)]
        self.MAP_4_SPAWN_POINT = [(0,0,0)]
        self.MAP_5_SPAWN_POINT = [(0,0,0)]

        # Goal Marker
        self.m = Marker()
        self.m.header.frame_id = "map"
        self.m.ns = "goal_marker"
        self.m.type = Marker.CYLINDER
        self.m.action = Marker.ADD
        self.m.color.r, self.m.color.g, self.m.color.b = 1, 1, 0
        self.m.color.a = 1
        self.m.scale.x = 0.5
        self.m.scale.y = 0.5
        self.m.scale.z = 0

        # Rate of each thread
        self.thread_rate = 10