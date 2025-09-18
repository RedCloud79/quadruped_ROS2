import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_dir = get_package_share_directory('quadruped_bringup')

    # Fast-LIO 실행
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'odom_slam.launch.py')
        ])
    )

    # Map Loader 실행 (3D PCD Map)
    map_loader_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('autoware_map_loader'),
                'launch', 'pointcloud_map_loader.launch.xml'
            )
        ]),
        launch_arguments={
            # 기본 맵 경로 (nav2_bringup/maps_3d 안에서 관리)
            'pointcloud_map_path': os.path.join(bringup_dir, 'maps_3d', 'office.pcd'),
            'pointcloud_map_metadata_path': os.path.join(bringup_dir, 'maps_3d', 'office_metadata.yaml'),
        }.items()
    )

    # NDT Scan Matcher 실행
    ndt_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('autoware_ndt_scan_matcher'),
                'launch', 'ndt_scan_matcher.launch.xml'
            )
        ])
    )

    # EKF Localizer 실행
    ekf_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('autoware_ekf_localizer'),
                'launch', 'ekf_localizer.launch.xml'
            )
        ])
    )

    # Nav2 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(my_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items(),
    )

    return LaunchDescription([
        fast_lio_launch,
        map_loader_launch,   # ✅ 맵 로더 먼저 실행
        ndt_launch,
        ekf_launch,
        nav2_launch
    ])
