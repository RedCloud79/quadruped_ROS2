FROM arm64v8/ros:humble

RUN apt update && apt install -y \
    software-properties-common sudo git curl wget build-essential cmake \
    python3-colcon-common-extensions python3-pip \
    libpcap-dev libpcl-dev \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rviz2 \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-realsense2-camera \
    ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-pcl-conversions \
    ros-humble-vision-opencv ros-humble-xacro \
    ros-humble-topic-tools ros-humble-pointcloud-to-laserscan \
    ros-humble-cv-bridge ros-humble-robot-state-publisher \
    ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-pcl-ros \
    ros-humble-ament-cmake-auto ros-humble-rqt-tf-tree \
    nano x11-apps && \
    rm -rf /var/lib/apt/lists/*

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
RUN source /opt/ros/humble/setup.bash && ./build.sh humble

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /home/user/ros2_ws/install/setup.bash' >> ~/.bashrc

