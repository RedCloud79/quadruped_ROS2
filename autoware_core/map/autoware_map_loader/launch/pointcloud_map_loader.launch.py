from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    map_pcd = LaunchConfiguration('map_pcd').perform(context)
    map_metadata = LaunchConfiguration('map_metadata').perform(context)

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
                'pcd_paths_or_directory': [map_pcd],   # string array
                'pcd_metadata_path': map_metadata      # string
            }
        ]
    )

    return [pointcloud_map_loader]


def generate_launch_description():
    default_map_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'maps_3d'
    )

    declare_map_pcd = DeclareLaunchArgument(
        'map_pcd',
        default_value=os.path.join(default_map_dir, 'map_pg.pcd'),
        description='Path to pointcloud map file'
    )

    declare_map_metadata = DeclareLaunchArgument(
        'map_metadata',
        default_value=os.path.join(default_map_dir, 'map_pg.yaml'),
        description='Path to pointcloud metadata yaml'
    )

    return LaunchDescription([
        declare_map_pcd,
        declare_map_metadata,
        OpaqueFunction(function=launch_setup)
    ])
