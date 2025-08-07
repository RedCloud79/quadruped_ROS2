import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments (equivalent to <arg> in ROS1)
    return LaunchDescription([
        DeclareLaunchArgument('lvx_file_path', default_value='livox_test.lvx', description='Path to LVX file'),
        DeclareLaunchArgument('bd_list', default_value='100000000000000', description='Board ID list'),
        DeclareLaunchArgument('xfer_format', default_value='0', description='Transfer format'),
        DeclareLaunchArgument('multi_topic', default_value='0', description='Multi-topic mode'),
        DeclareLaunchArgument('data_src', default_value='0', description='Data source'),
        DeclareLaunchArgument('publish_freq', default_value='10.0', description='Publish frequency'),
        DeclareLaunchArgument('output_type', default_value='0', description='Output type'),
        DeclareLaunchArgument('rviz_enable', default_value='false', description='Enable RViz'),
        DeclareLaunchArgument('rosbag_enable', default_value='false', description='Enable rosbag recording'),
        DeclareLaunchArgument('cmdline_arg', default_value=LaunchConfiguration('bd_list'), description='Board ID'),
        DeclareLaunchArgument('msg_frame_id', default_value='livox_frame', description='Frame ID for messages'),
        DeclareLaunchArgument('lidar_bag', default_value='true', description='Enable lidar bag recording'),
        DeclareLaunchArgument('imu_bag', default_value='true', description='Enable imu bag recording'),

        # Parameters for livox driver node
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher2',
            output='screen',
            parameters=[{
                'xfer_format': LaunchConfiguration('xfer_format'),
                'multi_topic': LaunchConfiguration('multi_topic'),
                'data_src': LaunchConfiguration('data_src'),
                'publish_freq': LaunchConfiguration('publish_freq'),
                'output_data_type': LaunchConfiguration('output_type'),
                'cmdline_str': LaunchConfiguration('bd_list'),
                'cmdline_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path': os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json'),
                'frame_id': LaunchConfiguration('msg_frame_id'),
                'enable_lidar_bag': LaunchConfiguration('lidar_bag'),
                'enable_imu_bag': LaunchConfiguration('imu_bag'),
                'use_ros_time': True,
                'ros_time_override': True
            }],
            # remappings=[('/livox/lidar', '/rslidar_points')]
        ),

        # Static transform publisher for original livox transform
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base2livox',
        #     output='screen',
        #     arguments=['0.16', '0', '0.13', '0', '0.35', '0', '/base_link', '/livox_frame']
        # ),

        # Static transform publisher for new flat lidar frame
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='base2flat_lidar',
        #    output='screen',
        #    arguments=['0', '0', '0', '0', '0', '0', '1', '/base_link', '/flat_lidar_frame']
        #),

        # Relay lidar raw data to lidar_point_cloud
        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name='relay_lidar_to_pointcloud',
        #     output='screen',
        #     arguments=['/rslidar_points', '/lidar_point_cloud']
        # ),

        # Pointcloud to LaserScan node
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2_to_scan',
            parameters=[
                {'target_frame': 'flat_lidar_frame'},
                {'transform_tolerance': 0.1},
                {'min_height': -0.1},
                {'max_height': 0.9},
                {'angle_min': -3.14},
                {'angle_max': 3.14},
                {'angle_increment': 0.0087},
                {'scan_time': 0.1},
                {'range_min': 0.1},
                {'range_max': 30.0},
                {'use_inf': True}
            ],
            remappings=[('cloud_in', '/lidar_point_cloud'), ('scan', '/scan')]
        ),

        # Optional rosbag recording
        Node(
            package='rosbag2',
            executable='record',
            name='record',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rosbag_enable')),
            arguments=['-a']
        )
    ])
