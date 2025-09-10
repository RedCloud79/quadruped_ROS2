import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_dir = get_package_share_directory('quadruped_bringup')

    return LaunchDescription([
        # EKF (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(my_dir, 'config', 'ekf_nav2.yaml')]
        ),

        # Fast-LIO (odometry)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('fast_lio'),
                    'launch', 'odom.launch.py'
                )
            ])
        ),

        # Autoware Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('tier4_localization_launch'),
                    'launch', 'localization.launch.xml'
                )
            ])
        ),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'params_file': os.path.join(my_dir, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'false'
            }.items(),
        ),
    ])
