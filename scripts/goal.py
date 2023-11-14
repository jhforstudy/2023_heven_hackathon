#!/usr/bin/env python3

import math
import numpy as np

# 미션 별 번호
PARKING_SPOT  = 1
STOP_LINE     = 2
DELIV_PICKUP  = 3
DELIV_DROPOFF = 4
TRAFFIC       = 5

# 교차로에서 왼쪽, 오른쪽 표현
TRAFFIC_LEFT  = "LEFT"
TRAFFIC_RIGHT = "RIGHT"
TRAFFIC_STRAIGHT = "STRAIGHT"


class Goal:
    def __init__(self, mode=PARKING_SPOT, position=np.array(None, None), yaw=None, flag=0, number=1, traffic=None, tolerance=None):
        
        self.mode = mode
        self.position = position
        self.yaw = yaw
        self.unit_vector = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw))])
        self.flag = flag
        self.tolerance = [0,0,0]
        self.number = number
        self.traffic = traffic

        # tolerance는 미션 존으로 인식되기 위한 최대 거리
        if self.mode == PARKING_SPOT:
            self.tolerance[0] = 3.00                # x_tol
            self.tolerance[1] = 3.00                # y_tol

        elif self.mode == STOP_LINE:
            self.tolerance[0] = 1.00                # x_tol
            self.tolerance[1] = 1.00                # y_tol
        
        elif self.mode == DELIV_PICKUP:
            self.tolerance[0] = 3.00                # x_tol
            self.tolerance[1] = 3.00                # y_tol

        elif self.mode == DELIV_DROPOFF:
            self.tolerance[0] = 3.00                # x_tol
            self.tolerance[1] = 3.00                # y_tol

        elif self.mode == TRAFFIC:
            self.tolerance[0] = 1.00                # x_tol
            self.tolerance[1] = 1.00                # y_tol

        # 특정 tolerance를 적용하려면, 클래스 생성할때 파라미터로 대입
        if tolerance is not None:
            self.tolerance[0] = tolerance[0]
            self.tolerance[1] = tolerance[1]