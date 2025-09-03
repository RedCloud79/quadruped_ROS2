FROM arm64v8/ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

RUN apt update && apt install -y \
    software-properties-common sudo git curl wget build-essential cmake \
    python3-colcon-common-extensions python3-pip python3-vcstool \
    ros-humble-rmw-cyclonedds-cpp \
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
    geographiclib-tools \
    nano x11-apps && \
    rm -rf /var/lib/apt/lists/*

RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt update && apt install -y \
    libgtsam-dev \
    libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*


WORKDIR /home/user/ros2_ws/src

# --- Livox-SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /home/user/ros2_ws/src/Livox-SDK2
RUN mkdir build && cd build && cmake .. && make -j"$(nproc)" && make install

COPY ./ros2_ws/src/livox_ros_driver2 /home/user/ros2_ws/src/livox_ros_driver2
COPY ./ros2_ws/src/autoware.universe /home/user/ros2_ws/src/autoware.universe

WORKDIR /home/user/ros2_ws
RUN rosdep init || true && rosdep update
RUN vcs import src < src/autoware.universe/build_depends_humble.repos

RUN rosdep install --from-paths src -r -y --rosdistro humble

SHELL ["/bin/bash", "-lc"]

WORKDIR /home/user/ros2_ws/src/livox_ros_driver2
RUN source /opt/ros/humble/setup.bash && ./build.sh humble

WORKDIR /home/user/ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to tier4_localization_launch autoware_map_tf_generator

RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /home/user/ros2_ws/install/setup.bash' >> /root/.bashrc
