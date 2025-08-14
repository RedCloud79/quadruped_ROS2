import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lvx_file_path', default_value='livox_test.lvx'),
        DeclareLaunchArgument('bd_list',       default_value='100000000000000'),
        DeclareLaunchArgument('xfer_format',   default_value='0'),  # 0: Livox Custom
        DeclareLaunchArgument('multi_topic',   default_value='0'),
        DeclareLaunchArgument('data_src',      default_value='0'),
        DeclareLaunchArgument('publish_freq',  default_value='10.0'),

        DeclareLaunchArgument('output_data_type', default_value='0'),
        DeclareLaunchArgument('rviz_enable',      default_value='false'),
        DeclareLaunchArgument('rosbag_enable',    default_value='false'),
        DeclareLaunchArgument('cmdline_arg',      default_value=LaunchConfiguration('bd_list')),
        DeclareLaunchArgument('msg_frame_id',     default_value='livox_frame'),
        DeclareLaunchArgument('lidar_bag',        default_value='true'),
        DeclareLaunchArgument('imu_bag',          default_value='true'),

        DeclareLaunchArgument('pc2_topic',        default_value='/livox/pointcloud'),


        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher2',
            output='screen',
            condition=UnlessCondition(PythonExpression([
                "'", LaunchConfiguration('output_data_type'), "' == '1'"
            ])),
            parameters=[{
                'xfer_format':       LaunchConfiguration('xfer_format'),
                'multi_topic':       LaunchConfiguration('multi_topic'),
                'data_src':          LaunchConfiguration('data_src'),
                'publish_freq':      LaunchConfiguration('publish_freq'),
                'output_data_type':  '0',  
                'cmdline_str':       LaunchConfiguration('bd_list'),
                'cmdline_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path':  os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json'),
                'frame_id':          LaunchConfiguration('msg_frame_id'),
                'enable_lidar_bag':  LaunchConfiguration('lidar_bag'),
                'enable_imu_bag':    LaunchConfiguration('imu_bag'),
                'use_ros_time':      True,
                'ros_time_override': True,
            }],

        ),

        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher2_pc2',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('output_data_type'), "' == '1'"
            ])),
            parameters=[{
                'xfer_format':       LaunchConfiguration('xfer_format'),
                'multi_topic':       LaunchConfiguration('multi_topic'),
                'data_src':          LaunchConfiguration('data_src'),
                'publish_freq':      LaunchConfiguration('publish_freq'),
                'output_data_type':  '1',  
                'cmdline_str':       LaunchConfiguration('bd_list'),
                'cmdline_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path':  os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json'),
                'frame_id':          LaunchConfiguration('msg_frame_id'),
                'enable_lidar_bag':  LaunchConfiguration('lidar_bag'),
                'enable_imu_bag':    LaunchConfiguration('imu_bag'),
                'use_ros_time':      True,
                'ros_time_override': True,
            }],
            remappings=[('/livox/lidar', LaunchConfiguration('pc2_topic'))],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2_to_scan',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('output_data_type'), "' == '1'"
            ])),
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
                {'use_inf': True},
            ],
            remappings=[('cloud_in', LaunchConfiguration('pc2_topic')), ('scan', '/scan')]
        ),

        # ===== (선택) rosbag 기록 =====
        # Node(
        #     package='rosbag2',
        #     executable='record',
        #     name='record',
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('rosbag_enable')),
        #     arguments=['-a']
        # ),
    ])