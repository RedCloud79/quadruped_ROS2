# launch/navigation.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 이 패키지의 경로 찾기
    pkg_share = get_package_share_directory('robot_navigation')

    # Nav2 파라미터 파일 경로
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    # 지도 파일 경로
    map_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    # Nav2의 bringup 런치 파일 포함하기
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'false',
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_bringup_launch,
        # 여기에 로봇의 다른 노드(예: robot_description)를 함께 실행시킬 수 있습니다.
    ])
