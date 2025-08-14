import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

# 안전한 캐스팅 헬퍼
INT   = lambda name:  PythonExpression(["int(",  LaunchConfiguration(name), ")"])      # e.g., INT('output_data_type')
FLOAT = lambda name:  PythonExpression(["float(",LaunchConfiguration(name), ")"])      # e.g., FLOAT('publish_freq')
BOOL  = lambda name:  PythonExpression(["str(",  LaunchConfiguration(name), ").lower() in ['1','true','yes']"])  # e.g., BOOL('lidar_bag')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lvx_file_path', default_value='livox_test.lvx'),
        DeclareLaunchArgument('bd_list',       default_value='100000000000000'),
        DeclareLaunchArgument('xfer_format',   default_value='0'),  # 0: Livox Custom
        DeclareLaunchArgument('multi_topic',   default_value='0'),
        DeclareLaunchArgument('data_src',      default_value='0'),
        DeclareLaunchArgument('publish_freq',  default_value='10.0'),
        DeclareLaunchArgument('output_data_type', default_value='0'),  # 0: CustomMsg, 1: PointCloud2
        DeclareLaunchArgument('rviz_enable',      default_value='false'),
        DeclareLaunchArgument('rosbag_enable',    default_value='false'),
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
                "int(", LaunchConfiguration('output_data_type'), ") == 1"
            ])),
            parameters=[{
                'xfer_format':       INT('xfer_format'),
                'multi_topic':       INT('multi_topic'),
                'data_src':          INT('data_src'),
                'publish_freq':      FLOAT('publish_freq'),
                'output_data_type':  INT('output_data_type'),  # 기대값 0
                'cmdline_str':       LaunchConfiguration('bd_list'),
                'cmdline_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path':  os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json'),
                'frame_id':          LaunchConfiguration('msg_frame_id'),
                'enable_lidar_bag':  BOOL('lidar_bag'),
                'enable_imu_bag':    BOOL('imu_bag'),
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
                "int(", LaunchConfiguration('output_data_type'), ") == 1"
            ])),
            parameters=[{
                'xfer_format':       INT('xfer_format'),
                'multi_topic':       INT('multi_topic'),
                'data_src':          INT('data_src'),
                'publish_freq':      FLOAT('publish_freq'),
                'output_data_type':  INT('output_data_type'),  # 기대값 1
                'cmdline_str':       LaunchConfiguration('bd_list'),
                'cmdline_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path':  os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json'),
                'frame_id':          LaunchConfiguration('msg_frame_id'),
                'enable_lidar_bag':  BOOL('lidar_bag'),
                'enable_imu_bag':    BOOL('imu_bag'),
                'use_ros_time':      True,
                'ros_time_override': True,
            }],
            remappings=[('/livox/lidar', LaunchConfiguration('pc2_topic'))],
        ),
    ])