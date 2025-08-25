from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'broadcast_code',
            default_value='',
            description='Broadcast code of Livox LiDAR'
        ),
        DeclareLaunchArgument(
            'output_data_type',
            default_value='0',
            description='0: output to ros, 1: output to rosbag'
        ),
        DeclareLaunchArgument(
            'xfer_format',
            default_value='0',
            description='0: custom msg, 1: PointCloud2'
        ),
        DeclareLaunchArgument(
            'multi_topic',
            default_value='0',
            description='0: one topic, 1: multi topic'
        ),
        DeclareLaunchArgument(
            'data_src',
            default_value='0',
            description='0: raw ethernet, 1: rosbag file'
        ),
        DeclareLaunchArgument(
            'publish_freq',
            default_value='10.0',
            description='publish frequency in Hz'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='livox_frame',
            description='Frame ID for published data'
        ),

        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher2',
            output='screen',
            parameters=[{
                'broadcast_code': LaunchConfiguration('broadcast_code'),
                'output_data_type': LaunchConfiguration('output_data_type'),
                'xfer_format': LaunchConfiguration('xfer_format'),
                'multi_topic': LaunchConfiguration('multi_topic'),
                'data_src': LaunchConfiguration('data_src'),
                'publish_freq': LaunchConfiguration('publish_freq'),
                'frame_id': LaunchConfiguration('frame_id'),
            }]
        )
    ])
