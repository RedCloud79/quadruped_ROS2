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

```


[livox_custommsg_adapter-1] 2025-08-14 07:03:04.110 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7411: open_and_lock_file failed -> Function open_port_internal


Could not determine the type for the passed topic
root@lite3:/home/user/ros2_ws# ros2 topic echo /livox/lidar
Cannot echo topic '/livox/lidar', as it contains more than one type: [livox_interfaces/msg/CustomMsg, sensor_msgs/msg/PointCloud2]


CFG=$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/config/MID360_config.json

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=livox_lidar_publisher2 \
  -p user_config_path:=$CFG \
  -p cmdline_str:=100000000000000 \
  -p output_data_type:=0 \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p frame_id:=livox_frame


root@lite3:/home/user/ros2_ws# ros2 param get /livox_lidar_publisher2 ouput_data_type
2025-08-14 07:37:27.754 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7411: open_and_lock_file failed -> Function open_port_internal
Parameter not set
root@lite3:/home/user/ros2_ws# ros2 param get /livox_lidar_publisher2 xfer_format
2025-08-14 07:37:37.601 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7411: open_and_lock_file failed -> Function open_port_internal
Integer value is: 0
root@lite3:/home/user/ros2_ws# ros2 param get /livox_lidar_publisher2 user_config_path
2025-08-14 07:37:48.856 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7411: open_and_lock_file failed -> Function open_port_internal
String value is: /home/user/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json


oot@lite3:/home/user/ros2_ws# ros2 topic info -v /livox/lidar
Type: ['livox_interfaces/msg/CustomMsg', 'sensor_msgs/msg/PointCloud2']

Publisher count: 1

Node name: livox_lidar_publisher2
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.52.fc.9b.3d.0c.b5.00.00.00.00.00.00.13.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: livox_custommsg_adapter
Node namespace: /
Topic type: livox_interfaces/msg/CustomMsg
Endpoint type: SUBSCRIPTION
GID: 01.0f.52.fc.af.3d.a4.b3.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite


   
```
