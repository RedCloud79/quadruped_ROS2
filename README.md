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
tep 11/12 : RUN ./build.sh humble
 ---> Running in a79234366a12
Working Path: /home/user/ros2_ws/src/livox_ros_driver2
ROS version is: ROS2
Starting >>> livox_ros_driver2
Starting >>> livox_sdk2
--- stderr: livox_ros_driver2
CMake Error at CMakeLists.txt:224 (find_package):
  By not providing "Findament_cmake_auto.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "ament_cmake_auto", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake_auto"
  with any of the following names:

    ament_cmake_autoConfig.cmake
    ament_cmake_auto-config.cmake

  Add the installation prefix of "ament_cmake_auto" to CMAKE_PREFIX_PATH or
  set "ament_cmake_auto_DIR" to a directory containing one of the above
  files.  If "ament_cmake_auto" provides a separate development package or
  SDK, be sure it has been installed.


---
Failed   <<< livox_ros_driver2 [0.63s, exited with code 1]
Aborted  <<< livox_sdk2 [0.62s]

Summary: 0 packages finished [1.02s]
  1 package failed: livox_ros_driver2
  1 package aborted: livox_sdk2
  2 packages had stderr output: livox_ros_driver2 livox_sdk2
 ---> Removed intermediate container a79234366a12
 ---> 3d35a991c692
Step 12/12 : RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc &&     echo 'source /home/user/ros2_ws/install/setup.bash' >> ~/.bashrc




```
```
FROM arm64v8/ros:humble

RUN apt update && apt install -y \
    software-properties-common \
    sudo \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    libpcap-dev \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rviz2 \
    ros-humble-tf2-geometry-msgs \
    ros-humble-realsense2-camera \
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs \
    ros-humble-vision-opencv \
    ros-humble-xacro \
    ros-humble-topic-tools\
    ros-humble-pointcloud-to-laserscan\
    ros-humble-ament-cmake-auto \
    ros-humble-rqt-tf-tree \
    nano \
    x11-apps\
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt update && apt install -y \
    libgtsam-dev \
    libgtsam-unstable-dev
    
# Livox-SDK2 Download
WORKDIR /home/user/ros2_ws/src
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git

WORKDIR /home/user/ros2_ws/src/Livox-SDK2
RUN mkdir build && cd build && cmake .. && make -j$(nproc) && make install

COPY ./ros2_ws/src/livox_ros_driver2 /home/user/ros2_ws/src/livox_ros_driver2
SHELL ["/bin/bash", "-lc"]
# build livox_lidar2
WORKDIR /home/user/ros2_ws/src/livox_ros_driver2
RUN ./build.sh humble

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /home/user/ros2_ws/install/setup.bash' >> ~/.bashrc

```
