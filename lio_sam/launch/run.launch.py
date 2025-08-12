# lio_sam.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('lio_sam')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])
    urdf_path     = PathJoinSubstitution([pkg_share, 'config', 'Lite3.urdf'])
    rviz_path     = PathJoinSubstitution([pkg_share, 'config', 'rviz2.rviz'])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the ROS 2 parameters YAML for lio_sam'
    )
    params_file = ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)

    return LaunchDescription([
        params_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['cat ', urdf_path])}],
        ),

        Node(
            package='lio_sam', executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='lio_sam', executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='lio_sam', executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='lio_sam', executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization', output='screen',
            parameters=[params_file],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_path], output='screen',
        ),
    ])
