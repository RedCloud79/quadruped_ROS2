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



### Fast_lio2 설치 진행



참고 : https://github.com/hku-mars/FAST_LIO/tree/ROS2



```



```



```



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
In file included from /home/user/ros2_ws/src/transfer/src/Jetson2App.cpp:9:
/opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.h:35:2: warning: #warning This header is obsolete, please include tf2_geometry_msgs/tf2_geometry_msgs.hpp instead [-Wcpp]
   35 | #warning This header is obsolete, please include tf2_geometry_msgs/tf2_geometry_msgs.hpp instead
      |  ^~~~~~~
In file included from /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.h:37,
                 from /home/user/ros2_ws/src/transfer/src/Jetson2App.cpp:9:
/opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp:60:10: fatal error: tf2_ros/buffer_interface.hpp: No such file or directory
   60 | #include "tf2_ros/buffer_interface.hpp"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/jetson2app.dir/build.make:76: CMakeFiles/jetson2app.dir/src/Jetson2App.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:141: CMakeFiles/jetson2app.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< transfer [4.38s, exited with code 2]

```
