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
--- stderr: lio_sam                     
In file included from /home/user/ros2_ws/src/lio_sam/src/imuPreintegration.cpp:1:
/home/user/ros2_ws/src/lio_sam/include/lio_sam/utility.hpp:35:10: fatal error: pcl_conversions/pcl_conversions.h: No such file or directory
   35 | #include <pcl_conversions/pcl_conversions.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/lio_sam_imuPreintegration.dir/build.make:76: CMakeFiles/lio_sam_imuPreintegration.dir/src/imuPreintegration.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:658: CMakeFiles/lio_sam_imuPreintegration.dir/all] Error 2
gmake[1]: *** Waiting for unfinished jobs....
In file included from /home/user/ros2_ws/src/lio_sam/src/simpleGpsOdom.cpp:18:
/home/user/ros2_ws/src/lio_sam/include/lio_sam/utility.hpp:35:10: fatal error: pcl_conversions/pcl_conversions.h: No such file or directory
   35 | #include <pcl_conversions/pcl_conversions.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/lio_sam_simpleGpsOdom.dir/build.make:76: CMakeFiles/lio_sam_simpleGpsOdom.dir/src/simpleGpsOdom.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:714: CMakeFiles/lio_sam_simpleGpsOdom.dir/all] Error 2
In file included from /home/user/ros2_ws/src/lio_sam/src/imageProjection.cpp:1:
/home/user/ros2_ws/src/lio_sam/include/lio_sam/utility.hpp:35:10: fatal error: pcl_conversions/pcl_conversions.h: No such file or directory
   35 | #include <pcl_conversions/pcl_conversions.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/lio_sam_imageProjection.dir/build.make:76: CMakeFiles/lio_sam_imageProjection.dir/src/imageProjection.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:630: CMakeFiles/lio_sam_imageProjection.dir/all] Error 2
In file included from /home/user/ros2_ws/src/lio_sam/src/featureExtraction.cpp:1:
/home/user/ros2_ws/src/lio_sam/include/lio_sam/utility.hpp:35:10: fatal error: pcl_conversions/pcl_conversions.h: No such file or directory
   35 | #include <pcl_conversions/pcl_conversions.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/lio_sam_featureExtraction.dir/build.make:76: CMakeFiles/lio_sam_featureExtraction.dir/src/featureExtraction.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:602: CMakeFiles/lio_sam_featureExtraction.dir/all] Error 2
In file included from /home/user/ros2_ws/src/lio_sam/src/mapOptmization.cpp:1:
/home/user/ros2_ws/src/lio_sam/include/lio_sam/utility.hpp:35:10: fatal error: pcl_conversions/pcl_conversions.h: No such file or directory
   35 | #include <pcl_conversions/pcl_conversions.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/lio_sam_mapOptimization.dir/build.make:76: CMakeFiles/lio_sam_mapOptimization.dir/src/mapOptmization.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:686: CMakeFiles/lio_sam_mapOptimization.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< lio_sam [11.3s, exited with code 2]
Aborted  <<< livox_custommsg_adapter [15.9s]       
Aborted  <<< livox_ros_driver2 [26.9s] 

```
