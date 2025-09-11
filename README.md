# quadruped_ROS2

 - Orin Nx (jetpack 5.1.2)

## humble 버전 설치 리스트



- sudo apt install ros-humble-tf2-geometry-msgs

- sudo apt install libpcap-dev

- sudo apt-get install ros-humble-realsense2-camera



## 라이다 센서 드라이버 설치



- git clone https://github.com/Livox-SDK/livox_ros_driver2.git

- git clone https://github.com/Livox-SDK/Livox-SDK2.git





### Livox-SDK2 빌드



```

cd ~/ros2_ws/src/livox_ros_driver2/livox_sdk2

mkdir build && cd build

cmake ..

make -j$(nproc)

sudo make install

```



### livox_ros_driver2 빌드



```

cd ~/ros2_ws/src/livox_ros_driver2

source /opt/ros/humble/setup.sh

./build.sh humble

colcon build --symlink-install

```



## slam navigation tool install



### Fast_lio2 설치 진행



참고 : https://github.com/hku-mars/FAST_LIO/tree/ROS2



```

git clone --recursive https://github.com/Ericsii/FAST_LIO.git fast_lio

```



```
cd <ros2_ws>
. install/setup.bash # use setup.zsh if use zsh
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml

ros2 launch livox_ros_driver2 msg_MID360_launch.py


```



### ros2 navigation설치 진행



```

sudo apt update

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

```

* Localization: ndt_scan_matcher (전역 맵 기반)

* Navigation: nav2 (전역/지역 경로 계획 프레임워크)

  * Global Planner: SmacPlanner2D/Hybrid-A*

  * Local Planner: DWB (단순) or TEB (복잡 장애물 대응)

* Elevation Mapping: elevation_mapping_cupy

  * LiDAR 포인트 클라우드 → 2.5D height map

  * Nav2 costmap layer로 연동 (경사·계단을 traversable vs not 구분)

* Footstep Planner (optional): 4족 보행 로봇의 발 디딜 좌표까지 계획 → Kinova Gen2 arm까지 연동하면 “계단 잡고 오르기”도 가능


## 주행 구성 예상 순서

🔹 실행 흐름

  1. FAST-LIO2: /lio/odom (odometry)

  2. autoware: localization

  4. Elevation Map: /elevation_map (LiDAR 기반 높이지도)

  5. Nav2: 전역 경로는 평면 경로, 로컬 costmap은 elevation 정보로 보정 → 계단/경사로 주행 허용

  6. Locomotion Controller: 4족 로봇의 실제 보행 제어기 (ROS2 Control + custom gait controller)

## file WorkSpace



```

ros2_ws/

└── src/

    ├── robot_control/

    │   ├── package.xml

    │   ├── setup.py

    │   ├── setup.cfg

    │   └── robot_control/

    │       ├── __init__.py

    │       ├── move_point_server.py

    │       ├── home_server.py

    │       ├── docking_server.py

    │       ├── patrol_server.py

    │       ├── emergency_server.py

    │       └── stop_server.py

    │

    ├── robot_interfaces/

    │   ├── package.xml

    │   ├── CMakeLists.txt

    │   ├── action/

    │   │   ├── MovePoint.action

    │   │   ├── GoHome.action

    │   │   ├── Docking.action

    │   │   └── Patrol.action

    │   └── srv/

    │       ├── EmergencyStop.srv

    │       └── StopMotion.srv

    │

    ├── transfer/ 

    │   ├── include/

    │   │   └── protocol.hpp

    │   ├── launch/

    │   │   └── transfer_launch.py

    │   ├── src/

    │   │   ├── Jetson2App.cpp

    │   │   ├── Jetson2Motion.cpp

    │   │   └── SensorChecker.cpp

    │   ├── CMakeLists.txt

    │   └── package.xml

    │

    │── transfer_interfaces/

    │   ├── msg/

    │   │   ├── MotionComplexCMD.msg

    │   │   └── MotionSimpleCMD.msg

    │   ├── CMakeLists.txt

    │   └── package.xml

    │

    ├── robot_manager/

    │   ├── package.xml

    │   ├── setup.py

    │   └── robot_manager/

    │       └── robot_manager_node.py

    │

    └── manager_interfaces/

        ├── package.xml

        ├── CMakeLists.txt

        └── action/

            └── ExecuteTask.action

```
