# HEVEN Hackathon

### 2023 HEVEN 자율주행 해커톤
- 일정 : 2023-11-17(금) 17:00 ~ 2023-11-18(토) 10:00
- 장소 : 성균관대학교 산학협력관 러닝팩토리 (85133호)
- 인원 : 약 30명 내외 (4인 1팀 구성)
- 내용 : 4인 1팀이 되어 주어진 map 상에서 자율주행 및 미션 알고리즘 구현

## 설치
### 1. Docker를 이용한 개발 환경 구성 (Recommended)

Windows 환경에서도 다음의 링크를 참고하여 편하게 개발할 수 있습니다.
[Docker 환경 설치하기](https://github.com/jhforstudy/HEVEN_Hackathon/blob/master/InstallDocker.md)

### 2. Ubuntu에 패키지 직접 설치

* Ubuntu 20.04 멀티부팅 설치

https://carrido-hobbies-well-being.tistory.com/84

* ROS 설치 (상단의 *"noetic"* 클릭 후 진행)

http://wiki.ros.org/Installation/Ubuntu

* Dependencies 설치

    ```
    sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server
    ```

* ROS용 워크스페이스 생성

    ```
    mkdir catkin_ws && cd catkin_ws
    mkdir src && cd src
    ```
    
* 레포지토리 복제

    ```
    git clone https://github.com/s-duuu/2023_heven_hackathon.git
    ```

* 패키지 빌드

    ```
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## 최신 버전 업데이트

* 아래 명령어를 실행하여, "Already up to date." 라는 문구가 떠야 최신 버전임
    ```
    cd ~/catkin_ws/src/2023_heven_hackathon/
    git pull
    ```

## 실행

* 시뮬레이터 실행
    ```
    roslaunch racecar_simulator simulate.launch map_number:=1
    ```
    
* 자율주행 알고리즘 (brain) 실행
    ```
    roslaunch racecar_simulator brain.launch
    ```

* 수동 조작 노드 실행 (시뮬레이터 실행 후)
    ```
    roslaunch racecar_simulator teleop.launch
    ```
    
## 센서 값 호출 방법

* Brain.py에서 다음과 같이 호출하여 사용할 수 있음
    ```python
    lidar_data = self.db.lidar_data
    pose_data = self.db.pose_data
    ```

* LiDAR<br>
    ```
    변수명 : self.db.lidar_data
    형태 : list[360]
    값 설명 : 차량 후방을 기준으로 반시계방향으로 측정된 값
    ```
![라이다정보](https://user-images.githubusercontent.com/48710703/200983104-8a88354d-960b-4b2b-970f-fd6531710450.png)

* GPS + IMU (현재 차량의 global position)<br>
    ```
    변수명 : self.db.pose_data
    형태 : list[3]
    값 설명 : [map frame으로부터 차량의 x좌표, map frame으로부터 차량의 y좌표, x축 방향 기준 yaw 각도 (degree)]
    ```
![포즈정보](https://user-images.githubusercontent.com/48710703/200983112-4e640c43-f009-4d51-b6a7-d308253c548e.png)

## 미션에 필요한 정보 호출 방법

* 현재 미션 정보 <br>
    ```
    변수명 : self.db.current_mission
    형태 : 미션 번호
        (미션 없음 : 0)
        (주차 : PARKING_SPOT (1))
        (정지선 : STOP_LINE (2))
        (배달 출발 : DELIV_PICKUP (3))
        (배달 도착 : DELIV_DROPOFF (4))
    값 설명 :
        주변에 미션 존 없을 시 0
        혹은 해당 미션 번호 (숫자 혹은 변수명을 통해 확인 가능)
    ```

* 신호등 정보 <br>
    ```
    변수명 : self.db.traffic_light
    형태 : "None", "STOP", "LEFT", "RIGHT", "STRAIGHT"
    값 설명 :
        없을 시 "None"
        정지 시 "STOP"
        이후 도로 정보에 따라 "LEFT", "RIGHT", "STRAIGHT"
    ```

* 신호등 남은 시간 <br>
    ```
    변수명 : self.db.traffic_remaining_time
    형태 : float
    값 설명 :
        신호등 정보가 "STOP"일 때 남은 시간 (5.0 sec ~ 0.0 sec)
    ```
    
* 목표 주차 공간의 위치 <br>
    ```
    변수명 : self.db.target_park
    형태 : 1 or 2 or 3
    값 설명 :
        목표 주차 공간 번호
    ```
    
* 주차 공간의 정보 <br>
    ```
    변수명 : self.db.parking_list
    형태 : [[x1, y1, yaw1, num1], [x2, y2, yaw2, num2], ...]
    값 설명 :
        해당하는 주차 공간의 x, y 좌표, 방향 (degree), 주차 공간 번호
        여러개 존재 가능
    ```
    
* 배달 출발 공간의 위치 <br>
    ```
    변수명 : self.db.pickup_list
    형태 : [x, y, yaw, num]
    값 설명 :
        해당하는 배달 출발 공간의 x, y 좌표, 방향 (degree), 목표로 하는 배달 도착 공간 번호
    ```
    
* 배달 도착 공간의 정보 <br>
    ```
    변수명 : self.db.dropoff_list
    형태 : [[x1, y1, yaw1, num1], [x2, y2, yaw2, num2], ...]
    값 설명 :
        해당하는 배달 도착 공간의 x, y 좌표, 방향 (degree), 배달 도착 공간 번호
        여러개 존재 가능
    ```

## 시뮬레이터 사용 방법

### 1. 시뮬레이터 실행

* 실행해야 할 맵에 따라 `map_1.sh`, `map_2.sh`, `map_3.sh`, `map_4.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭<br>
![캡처3](https://user-images.githubusercontent.com/48710703/199907347-0ea16bc2-b3c3-4a2b-aaeb-b652642cb594.PNG)

* 시뮬레이터 기능<br>
① 충돌 횟수, 경과 시간 측정<br>
② 충돌 횟수, 경과 시간, 차량 위치 초기화<br>
③ 시뮬레이터 화면<br>
&nbsp;&nbsp;&nbsp;&nbsp;좌클릭 - 화면 회전<br>
&nbsp;&nbsp;&nbsp;&nbsp;우클릭 - 화면 확대<br>
&nbsp;&nbsp;&nbsp;&nbsp;스크롤 클릭 - 화면 이동<br>
④ 시점 설정<br>
&nbsp;&nbsp;&nbsp;&nbsp;기본은 2D view이며 **Orbit(rviz)** 으로 변경 시 3D view 가능
![캡처4](https://user-images.githubusercontent.com/48710703/199908144-21a49b19-d5ba-4ae3-9c8c-605305b7932b.PNG)

### 2. 알고리즘 테스트

* `brain.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭<br>
아래의 문구와 함께 **brain.py**가 실행됨
![5](https://user-images.githubusercontent.com/48710703/199909682-9c98e999-167f-4233-93a8-761018de8c94.PNG)
                       
### 3. 차량 수동 조작
* `joystick.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭<br>
아래의 문구와 함께 조이스틱이 실행됨<br>
열린 터미널을 클릭한 후,<br>
&nbsp;&nbsp;&nbsp;&nbsp;좌/우 화살표 - 차량 조향각 왼쪽/오른쪽 증가<br>
&nbsp;&nbsp;&nbsp;&nbsp;상/하 화살표 - 차량 속도 높임/낮춤<br>
&nbsp;&nbsp;&nbsp;&nbsp;스페이스 바 - 차량 정지<br>
&nbsp;&nbsp;&nbsp;&nbsp;탭 키 - 조향각 초기화<br>
**주의** : brain.sh 와 따로 사용할 것 (제어 명령이 중복되어 문제 발생)
![캡처](https://user-images.githubusercontent.com/48710703/200274414-608ace90-05d1-4a65-8747-ead89e63efd6.PNG)

### 4. 신호등 방향 및 주차 공간 위치 변경

* 대회 진행 시, 신호등 방향과 주차 공간을 랜덤하게 선택하여 진행할 예정
(모든 팀이 동일한 방향으로)

* `parameter_list.py` 의 윗 부분을 수정하면 Map의 정보를 변경하여 테스트할 수 있음.

```python
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
...
```

* 변경사항 저장 후, `map_1.sh`, `map_2.sh`, `map_3.sh`, `map_4.sh` 를 다시 실행할 것!

### 5. 차량 자율주행 제어 방법
* `brain.py` 의 main 함수에서 ``angle``, ``speed`` 를 결정하여 return하면 그에 맞게 차량이 움직임

* ``angle``, ``speed``는 다음과 같이 결정하면 됨

```python
angle = 0   # 20(LEFT) ~ -20(RIGHT), degree
speed = 1   # 0 ~ 4, m/s
```
