FROM arm64v8/ros:humble

# === Base tools & ROS pkgs (원래 있던 것 유지) ===
RUN apt update && apt install -y \
    software-properties-common sudo git curl wget build-essential cmake ninja-build \
    python3-colcon-common-extensions python3-pip \
    # ----- [ADDED] NDT 빌드/의존 해결용 -----
    python3-vcstool python3-rosdep \
    # ---------------------------------------
    libpcap-dev libpcl-dev \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rviz2 \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-realsense2-camera \
    ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-pcl-conversions \
    ros-humble-vision-opencv ros-humble-xacro \
    ros-humble-topic-tools ros-humble-pointcloud-to-laserscan \
    ros-humble-ament-cmake-auto ros-humble-rqt-tf-tree \
    # ----- [ADDED] EKF 융합용 -----
    ros-humble-robot-localization \
    # ------------------------------
    nano x11-apps && \
    rm -rf /var/lib/apt/lists/*

# === GTSAM (사용 중이면 유지) ===
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt update && apt install -y \
    libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# === rosdep 초기화 ===
RUN rosdep init || true && rosdep update

# === 워크스페이스 ===
ENV WS=/home/user/ros2_ws
WORKDIR ${WS}/src

# === Livox SDK2 ===
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR ${WS}/src/Livox-SDK2
RUN mkdir -p build && cd build && cmake -GNinja .. && ninja && ninja install

# === livox_ros_driver2 (사용자 소스 복사) ===
COPY ./ros2_ws/src/livox_ros_driver2 ${WS}/src/livox_ros_driver2
SHELL ["/bin/bash", "-lc"]
WORKDIR ${WS}/src/livox_ros_driver2
RUN source /opt/ros/humble/setup.bash && ./build.sh humble

# === Autoware Universe: Localization 최소 세트만 ===
WORKDIR ${WS}/src
# 전체 레포 클론(간단) — 이후 필요한 패키지만 선택 빌드
RUN git clone https://github.com/autowarefoundation/autoware.universe.git

# Humble 의존 소스 import
WORKDIR ${WS}
RUN vcs import src < src/autoware.universe/build_depends_humble.repos

# 의존성 설치
RUN rosdep update && rosdep install --rosdistro humble --from-paths src --ignore-src -y -r

# 선택 빌드 (Localization 최소 구성)
# - autoware_ndt_scan_matcher: NDT 본체
# - map_loader: PCD map 로더
# - pointcloud_preprocessor: 다운샘플/필터
# - tier4_map_msgs / tier4_localization_msgs: 메시지
# - tier4_localization_launch: 런치 모음
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --packages-select autoware_ndt_scan_matcher map_loader pointcloud_preprocessor \
                        tier4_map_msgs tier4_localization_msgs tier4_localization_launch \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=ON

# === 편의: 자동 source ===
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo "source ${WS}/install/setup.bash" >> /root/.bashrc

WORKDIR ${WS}
