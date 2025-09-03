FROM arm64v8/ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# universe 활성화(일부 deps가 universe에 있음) + 기본툴
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common ca-certificates curl gnupg lsb-release sudo git wget \
    build-essential cmake \
    python3-rosdep python3-colcon-common-extensions python3-pip python3-vcstool \
    && add-apt-repository -y universe

# (옵션) ROS2 APT 키 재등록 - 베이스 이미지에 있어도 중복 무해
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros2.list

# 필요한 ROS/시스템 패키지 설치 (여기서는 인덱스 삭제하지 말고 유지)
RUN apt-get update && apt-get install -y \
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
    nano x11-apps

# GTSAM 4.1
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt-get update && apt-get install -y libgtsam-dev libgtsam-unstable-dev

WORKDIR /home/user/ros2_ws/src

# Livox-SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /home/user/ros2_ws/src/Livox-SDK2
RUN mkdir build && cd build && cmake .. && make -j"$(nproc)" && make install

# 사용자 소스 복사
COPY ./ros2_ws/src/livox_ros_driver2 /home/user/ros2_ws/src/livox_ros_driver2
COPY ./ros2_ws/src/autoware.universe /home/user/ros2_ws/src/autoware.universe

WORKDIR /home/user/ros2_ws

# rosdep DB 준비
RUN rosdep init || true && rosdep update

# Autoware 빌드 의존 리포들 가져오기
RUN vcs import src < src/autoware.universe/build_depends_humble.repos

# 🔧 핵심 수정 1: 인덱스 최신화
RUN apt-get update
# 🔧 핵심 수정 2: 소스는 무시하고 시스템 의존성만 설치
RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

SHELL ["/bin/bash", "-lc"]

# livox_ros_driver2 빌드
WORKDIR /home/user/ros2_ws/src/livox_ros_driver2
RUN source /opt/ros/humble/setup.bash && ./build.sh humble

# Autoware (필요 패키지만 예시)
WORKDIR /home/user/ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to tier4_localization_launch autoware_map_tf_generator

# 런타임 설정
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /home/user/ros2_ws/install/setup.bash' >> /root/.bashrc

# (마지막에만 슬림화)
RUN rm -rf /var/lib/apt/lists/*
