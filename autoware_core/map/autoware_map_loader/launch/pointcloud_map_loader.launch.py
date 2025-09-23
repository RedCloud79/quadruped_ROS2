from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    nav2_share_dir = get_package_share_directory('nav2_bringup')

    default_map_pcd = os.path.join(nav2_share_dir, 'maps_3d', 'map.pcd')
    default_map_metadata = os.path.join(nav2_share_dir, 'maps_2d', 'map.yaml')

    map_pcd = LaunchConfiguration('map_pcd')
    map_metadata = LaunchConfiguration('map_metadata')

    declare_map_pcd = DeclareLaunchArgument(
        'map_pcd',
        default_value=default_map_pcd,
        description='Path to pointcloud map file (.pcd)'
    )

    declare_map_metadata = DeclareLaunchArgument(
        'map_metadata',
        default_value=default_map_metadata,
        description='Path to pointcloud metadata yaml'
    )

    param_path = os.path.join(
        get_package_share_directory('autoware_map_loader'),
        'config/pointcloud_map_loader.param.yaml'
    )

    pointcloud_map_loader = Node(
        package='autoware_map_loader',
        executable='autoware_pointcloud_map_loader',
        name='pointcloud_map_loader',
        output='screen',
        remappings=[
            ('output/pointcloud_map', '/map/pointcloud_map'),
            ('service/get_partial_pcd_map', '/map/get_partial_pointcloud_map'),
            ('service/get_selected_pcd_map', '/map/get_selected_pointcloud_map'),
        ],
        parameters=[
            param_path,
            {
                'pcd_paths_or_directory': [ParameterValue(map_pcd, value_type=str)],
                'pcd_metadata_path': ParameterValue(map_metadata, value_type=str)
            }
        ]
    )

    return LaunchDescription([
        declare_map_pcd,
        declare_map_metadata,
        pointcloud_map_loader
    ])
