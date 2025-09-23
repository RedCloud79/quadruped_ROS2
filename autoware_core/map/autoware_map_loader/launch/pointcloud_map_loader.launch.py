from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    autoware_map_loader_dir = get_package_share_directory('autoware_map_loader')

    pointcloud_map_path = os.path.join(nav2_bringup_dir, 'maps_3d', 'office.pcd')
    pointcloud_map_metadata_path = os.path.join(nav2_bringup_dir, 'maps_3d', 'office_metadata.yaml')
    param_path = os.path.join(autoware_map_loader_dir, 'config', 'pointcloud_map_loader.param.yaml')

    return LaunchDescription([
        Node(
            package='autoware_map_loader',
            executable='autoware_pointcloud_map_loader',
            name='pointcloud_map_loader',
            output='screen',
            parameters=[{
                'pcd_paths_or_directory': [pointcloud_map_path],
                'pcd_metadata_path': pointcloud_map_metadata_path
            }, param_path],
            remappings=[
                ('output/pointcloud_map', '/map/pointcloud_map'),
                ('service/get_partial_pcd_map', '/map/get_partial_pointcloud_map'),
                ('service/get_selected_pcd_map', '/map/get_selected_pointcloud_map'),
            ]
        )
    ])
