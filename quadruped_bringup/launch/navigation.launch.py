import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    ndt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('autoware_ndt_scan_matcher'),
                'launch', 'ndt_scan_matcher.launch.py'
            )
        ])
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('autoware_ekf_localizer'),
                'launch', 'ekf_localizer.launch.py'
            )
        ])
    )

    # Nav2 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(my_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items(),
    )

    return LaunchDescription([
        fast_lio_launch,
        ndt_launch,
        ekf_launch,
        nav2_launch
    ])
