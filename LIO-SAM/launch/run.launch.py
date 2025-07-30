import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    robot_description_pkg_path = get_package_share_directory('robot_description')
    robot_navigation_pkg_path = get_package_share_directory('robot_navigation')
    lio_sam_pkg_path = get_package_share_directory('lio_sam')

    urdf_file = os.path.join(robot_description_pkg_path, 'urdf', 'Lite3.urdf')
    params_file = os.path.join(robot_navigation_pkg_path, 'params', 'params.yaml')
    rviz_config_file = os.path.join(robot_navigation_pkg_path, 'rviz', 'slam.rviz') # 'rviz' 폴더 생성 필요

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    imu_preintegration_node = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[params_file],
        output='screen'
    )

    image_projection_node = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[params_file],
        output='screen'
    )

    feature_extraction_node = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[params_file],
        output='screen'
    )

    map_optimization_node = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[params_file],
        output='screen'
    )

    # 5. RViz2를 실행합니다.
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_launch,
        imu_preintegration_node,
        image_projection_node,
        feature_extraction_node,
        map_optimization_node,
        rviz2_node,
        # 여기에 transfer_launch.py나 teleop 노드를 포함시켜 한 번에 실행할 수 있습니다.
    ])