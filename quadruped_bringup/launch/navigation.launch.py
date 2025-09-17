import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_dir = get_package_share_directory('quadruped_bringup')

    # Fast-LIO 실행 (SLAM Odometry)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('fast_lio'),
                'launch', 'odom_slam.launch.py'
            )
        ])
    )

    # Autoware Localization 실행 (NDT + EKF + Pose initializer 등)
    autoware_localization_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('autoware_core_localization'),
                'launch', 'autoware_core_localization.launch.xml'
            )
        )
    )

    # Nav2 실행 (Global + Local Planner, Costmap, Controller)
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
        autoware_localization_launch,
        nav2_launch
    ])
