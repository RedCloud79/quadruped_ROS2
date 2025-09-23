from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Arguments
    map_pcd = LaunchConfiguration('map_pcd')
    map_metadata = LaunchConfiguration('map_metadata')

    default_map_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'maps_3d'
    )

    declare_map_pcd = DeclareLaunchArgument(
        'map_pcd',
        default_value=os.path.join(default_map_dir, 'map.pcd'),
        description='Path to pointcloud map file'
    )

    declare_map_metadata = DeclareLaunchArgument(
        'map_metadata',
        default_value=os.path.join(default_map_dir, 'map.yaml'),
        description='Path to pointcloud metadata yaml'
    )

    # Param file path
    param_path = os.path.join(
        get_package_share_directory('autoware_map_loader'),
        'config/pointcloud_map_loader.param.yaml'
    )

    # Node
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
                # LaunchConfiguration -> string 변환 후 리스트로
                'pcd_paths_or_directory': PythonExpression(['[ "', map_pcd, '" ]']),
                'pcd_metadata_path': map_metadata
            }
        ]
    )

    return LaunchDescription([
        declare_map_pcd,
        declare_map_metadata,
        pointcloud_map_loader
    ])
