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
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-08-12-00-49-20-427850-lite3-1111
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [static_transform_publisher-1]: process started with pid [1112]
[INFO] [robot_state_publisher-2]: process started with pid [1114]
[INFO] [lio_sam_imuPreintegration-3]: process started with pid [1116]
[INFO] [lio_sam_imageProjection-4]: process started with pid [1118]
[INFO] [lio_sam_featureExtraction-5]: process started with pid [1120]
[INFO] [lio_sam_mapOptimization-6]: process started with pid [1122]
[INFO] [rviz2-7]: process started with pid [1131]
[static_transform_publisher-1] [ERROR] [1754959760.621280844] [rcl]: Failed to parse global arguments
[static_transform_publisher-1] terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidROSArgsError'
[static_transform_publisher-1]   what():  failed to initialize rcl: Couldn't parse params file: '--params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'. Error: Error parsing a event near line 56, at ./src/parse.c:769, at ./src/rcl/arguments.c:406
[lio_sam_imuPreintegration-3] [ERROR] [1754959760.630212312] [rcl]: Failed to parse global arguments
[lio_sam_imuPreintegration-3] terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidROSArgsError'
[lio_sam_imuPreintegration-3]   what():  failed to initialize rcl: Couldn't parse params file: '--params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'. Error: Error parsing a event near line 56, at ./src/parse.c:769, at ./src/rcl/arguments.c:406
[lio_sam_imageProjection-4] [ERROR] [1754959760.629015457] [rcl]: Failed to parse global arguments
[lio_sam_imageProjection-4] terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidROSArgsError'
[lio_sam_imageProjection-4]   what():  failed to initialize rcl: Couldn't parse params file: '--params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'. Error: Error parsing a event near line 56, at ./src/parse.c:769, at ./src/rcl/arguments.c:406
[lio_sam_featureExtraction-5] [ERROR] [1754959760.628938207] [rcl]: Failed to parse global arguments
[lio_sam_featureExtraction-5] terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidROSArgsError'
[lio_sam_featureExtraction-5]   what():  failed to initialize rcl: Couldn't parse params file: '--params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'. Error: Error parsing a event near line 56, at ./src/parse.c:769, at ./src/rcl/arguments.c:406
[robot_state_publisher-2] [INFO] [1754959760.674895475] [robot_state_publisher]: got segment FL_FOOT
[robot_state_publisher-2] [INFO] [1754959760.675243449] [robot_state_publisher]: got segment FL_HIP
[robot_state_publisher-2] [INFO] [1754959760.675274362] [robot_state_publisher]: got segment FL_SHANK
[robot_state_publisher-2] [INFO] [1754959760.675287738] [robot_state_publisher]: got segment FL_THIGH
[robot_state_publisher-2] [INFO] [1754959760.675298650] [robot_state_publisher]: got segment FR_FOOT
[robot_state_publisher-2] [INFO] [1754959760.675307931] [robot_state_publisher]: got segment FR_HIP
[robot_state_publisher-2] [INFO] [1754959760.675317019] [robot_state_publisher]: got segment FR_SHANK
[robot_state_publisher-2] [INFO] [1754959760.675325979] [robot_state_publisher]: got segment FR_THIGH
[robot_state_publisher-2] [INFO] [1754959760.675335195] [robot_state_publisher]: got segment HL_FOOT
[robot_state_publisher-2] [INFO] [1754959760.675344091] [robot_state_publisher]: got segment HL_HIP
[robot_state_publisher-2] [INFO] [1754959760.675352955] [robot_state_publisher]: got segment HL_SHANK
[robot_state_publisher-2] [INFO] [1754959760.675361596] [robot_state_publisher]: got segment HL_THIGH
[robot_state_publisher-2] [INFO] [1754959760.675370588] [robot_state_publisher]: got segment HR_FOOT
[robot_state_publisher-2] [INFO] [1754959760.675379196] [robot_state_publisher]: got segment HR_HIP
[robot_state_publisher-2] [INFO] [1754959760.675388060] [robot_state_publisher]: got segment HR_SHANK
[robot_state_publisher-2] [INFO] [1754959760.675396796] [robot_state_publisher]: got segment HR_THIGH
[robot_state_publisher-2] [INFO] [1754959760.675405468] [robot_state_publisher]: got segment TORSO
[robot_state_publisher-2] [INFO] [1754959760.675413789] [robot_state_publisher]: got segment base_link
[robot_state_publisher-2] [INFO] [1754959760.675422717] [robot_state_publisher]: got segment livox_frame
[rviz2-7] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
[lio_sam_mapOptimization-6] [ERROR] [1754959760.856470037] [rcl]: Failed to parse global arguments
[lio_sam_mapOptimization-6] terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidROSArgsError'
[lio_sam_mapOptimization-6]   what():  failed to initialize rcl: Couldn't parse params file: '--params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'. Error: Error parsing a event near line 56, at ./src/parse.c:769, at ./src/rcl/arguments.c:406
[ERROR] [static_transform_publisher-1]: process has died [pid 1112, exit code -6, cmd '/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom --ros-args --params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'].
[ERROR] [lio_sam_imuPreintegration-3]: process has died [pid 1116, exit code -6, cmd '/home/user/ros2_ws/install/lio_sam/lib/lio_sam/lio_sam_imuPreintegration --ros-args -r __node:=lio_sam_imuPreintegration --params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'].
[ERROR] [lio_sam_featureExtraction-5]: process has died [pid 1120, exit code -6, cmd '/home/user/ros2_ws/install/lio_sam/lib/lio_sam/lio_sam_featureExtraction --ros-args -r __node:=lio_sam_featureExtraction --params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'].
[ERROR] [lio_sam_imageProjection-4]: process has died [pid 1118, exit code -6, cmd '/home/user/ros2_ws/install/lio_sam/lib/lio_sam/lio_sam_imageProjection --ros-args -r __node:=lio_sam_imageProjection --params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'].
[rviz2-7] [INFO] [1754959761.182538802] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-7] [INFO] [1754959761.183108285] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-7] [INFO] [1754959761.214773150] [rviz2]: Stereo is NOT SUPPORTED
[ERROR] [lio_sam_mapOptimization-6]: process has died [pid 1122, exit code -6, cmd '/home/user/ros2_ws/install/lio_sam/lib/lio_sam/lio_sam_mapOptimization --ros-args -r __node:=lio_sam_mapOptimization --params-file /home/user/ros2_ws/install/lio_sam/share/lio_sam/config/params.yaml'].
```
