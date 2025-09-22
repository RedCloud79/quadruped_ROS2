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
    ros-humble-autoware-ndt-scan-matcher \
    ros-humble-autoware-ekf-localizer \
    ros-humble-autoware-map-loader \
    ros-humble-autoware-lint-common \
    ros-humble-autoware-cmake \
    ros-humble-test-msgs ros-humble-tf2-sensor-msgs \
    ros-humble-launch-testing-ament-cmake \
    nano x11-apps \
    libopencv-dev python3-opencv \
    libsuitesparse-dev libeigen3-dev \
    qtdeclarative5-dev qt5-qmake qtbase5-dev && \
    rm -rf /var/lib/apt/lists/*

RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt update && apt install -y \
    libgtsam-dev \
    libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_WITH_MARCH_NATIVE=OFF \
             -DBUILD_SHARED_LIBS=ON \
             -DG2O_BUILD_APPS=OFF \
             -DG2O_BUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && \
    cd /tmp && rm -rf g2o

ENV CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

WORKDIR /home/user/ros2_ws/src
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git

WORKDIR /home/user/ros2_ws/src/Livox-SDK2
RUN mkdir build && cd build && cmake .. && make -j$(nproc) && make install

COPY ./ros2_ws/src/livox_ros_driver2 /home/user/ros2_ws/src/livox_ros_driver2
SHELL ["/bin/bash", "-lc"]

WORKDIR /home/user/ros2_ws/src/livox_ros_driver2
RUN source /opt/ros/humble/setup.bash && ./build.sh humble

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /home/user/ros2_ws/install/setup.bash' >> ~/.bashrc
