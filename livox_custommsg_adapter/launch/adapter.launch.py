from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('in_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('out_topic', default_value='/lio_sam/points'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('time_scale', default_value='1e-6'),  # µs → s
        DeclareLaunchArgument('use_msg_header_time', default_value='true'),

        Node(
            package='livox_custommsg_adapter',
            executable='livox_custommsg_adapter',
            name='livox_custommsg_adapter',
            parameters=[{
                'in_topic': LaunchConfiguration('in_topic'),
                'out_topic': LaunchConfiguration('out_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'time_scale': LaunchConfiguration('time_scale'),
                'use_msg_header_time': LaunchConfiguration('use_msg_header_time'),
            }],
            output='screen')
    ])