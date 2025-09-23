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
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('autoware_map_loader'), 'launch', 'pointcloud_map_loader.launch.py')
        ])
    )

    # NDT Scan Matcher 실행
    ndt_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('autoware_ndt_scan_matcher'),
                'launch', 'ndt_scan_matcher.launch.xml'
            )
        )
    )

    # EKF Localizer 실행
    ekf_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('autoware_ekf_localizer'),
                'launch', 'ekf_localizer.launch.xml'
            )
        )
    )

    # Nav2 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
        ]),
    )

    return LaunchDescription([
        fast_lio_launch,
        map_loader_launch,
        ndt_launch,
        ekf_launch,
        nav2_launch
    ])
