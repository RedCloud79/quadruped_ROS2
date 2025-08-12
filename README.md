# quadruped_ROS2

 

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



### lio-sam-ros2 설치 진행



참고 : https://github.com/TixiaoShan/LIO-SAM/tree/ros2



```

sudo apt install ros-<ros2-version>-perception-pcl \

  	   ros-<ros2-version>-pcl-msgs \

  	   ros-<ros2-version>-vision-opencv \

  	   ros-<ros2-version>-xacro



# Add GTSAM-PPA

sudo add-apt-repository ppa:borglab/gtsam-release-4.1

sudo apt install libgtsam-dev libgtsam-unstable-dev

```



```

cd ~/ros2_ws/src

git clone https://github.com/TixiaoShan/LIO-SAM.git

cd lio-sam

git checkout ros2

cd ..

colcon build

```



### ros2 navigation설치 진행



```

sudo apt update

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

```



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

<img width="799" height="910" alt="Screenshot from 2025-08-12 09-55-06" src="https://github.com/user-attachments/assets/a4c5e5d3-90f0-4047-ab03-4e34fd81096b" />

<img width="800" height="917" alt="Screenshot from 2025-08-12 09-59-52" src="https://github.com/user-attachments/assets/e87783a0-17b3-4cb3-ad2c-059c88841e09" />

```
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
[INFO] [1754961141.534522378] [view_frames]: Generating graph in frames.pdf file...
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
Error:   TF_NO_FRAME_ID: Ignoring transform with child_frame_id "livox_frame"  from authority "default_authority" because frame_id not set
         at line 226 in ./src/buffer_core.cpp
[INFO] [1754961141.547447234] [view_frames]: Result:tf2_msgs.srv.FrameGraph_Response(frame_yaml="odom: \n  parent: 'map'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nFL_FOOT: \n  parent: 'FL_SHANK'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nFR_FOOT: \n  parent: 'FR_SHANK'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nHL_FOOT: \n  parent: 'HL_SHANK'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nHR_FOOT: \n  parent: 'HR_SHANK'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nlivox_frame: \n  parent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nTORSO: \n  parent: 'base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\nlidar_link: \n  parent: 'odom'\n  broadcaster: 'default_authority'\n  rate: 5.251\n  most_recent_transform: 1754961141.053801\n  oldest_transform: 1754961137.054348\n  buffer_length: 3.999\n")

```
