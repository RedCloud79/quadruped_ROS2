import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    lvx_file_path      = LaunchConfiguration('lvx_file_path')
    bd_list            = LaunchConfiguration('bd_list')
    xfer_format        = LaunchConfiguration('xfer_format')
    multi_topic        = LaunchConfiguration('multi_topic')
    data_src           = LaunchConfiguration('data_src')
    publish_freq       = LaunchConfiguration('publish_freq')
    output_data_type   = LaunchConfiguration('output_data_type')  # 0: CustomMsg, 1: PointCloud2
    msg_frame_id       = LaunchConfiguration('msg_frame_id')
    lidar_bag          = LaunchConfiguration('lidar_bag')
    imu_bag            = LaunchConfiguration('imu_bag')
    pc2_topic          = LaunchConfiguration('pc2_topic')
    user_config_path   = LaunchConfiguration('user_config_path')

    return LaunchDescription([
        DeclareLaunchArgument('lvx_file_path', default_value='livox_test.lvx'),
        DeclareLaunchArgument('bd_list',       default_value='100000000000000'),
        DeclareLaunchArgument('xfer_format',   default_value='0'),   # 0: Livox Custom
        DeclareLaunchArgument('multi_topic',   default_value='0'),
        DeclareLaunchArgument('data_src',      default_value='0'),   # 0: LiDAR 실시간, 1: LVX 파일
        DeclareLaunchArgument('publish_freq',  default_value='10.0'),
        DeclareLaunchArgument('output_data_type', default_value='0'),  # 기본은 CustomMsg
        DeclareLaunchArgument('msg_frame_id',     default_value='livox_frame'),
        DeclareLaunchArgument('lidar_bag',        default_value='true'),
        DeclareLaunchArgument('imu_bag',          default_value='true'),
        DeclareLaunchArgument('pc2_topic',        default_value='/livox/pointcloud'),
        DeclareLaunchArgument(
            'user_config_path',
            default_value=os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'config', 'MID360_config.json'
            )
        ),

        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher2',
            output='screen',
            condition=UnlessCondition(PythonExpression([
                "int(", output_data_type, ") == 1"
            ])),
            parameters=[{
                'xfer_format':       ParameterValue(xfer_format,     value_type=int),
                'multi_topic':       ParameterValue(multi_topic,     value_type=int),
                'data_src':          ParameterValue(data_src,        value_type=int),
                'publish_freq':      ParameterValue(publish_freq,    value_type=float),
                'output_data_type':  ParameterValue(0,               value_type=int),
                'cmdline_str':       bd_list,
                'cmdline_file_path': lvx_file_path,
                'user_config_path':  user_config_path,
                'frame_id':          msg_frame_id,
                'enable_lidar_bag':  ParameterValue(lidar_bag, value_type=bool),
                'enable_imu_bag':    ParameterValue(imu_bag,   value_type=bool),
                'use_ros_time':      True,
                'ros_time_override': True,
            }],
        ),

        # Node(
        #     package='livox_ros_driver2',
        #     executable='livox_ros_driver2_node',
        #     name='livox_lidar_publisher2_pc2',
        #     output='screen',
        #     condition=IfCondition(PythonExpression([
        #         "int(", output_data_type, ") == 1"
        #     ])),
        #     parameters=[{
        #         'xfer_format':       ParameterValue(xfer_format,     value_type=int),
        #         'multi_topic':       ParameterValue(multi_topic,     value_type=int),
        #         'data_src':          ParameterValue(data_src,        value_type=int),
        #         'publish_freq':      ParameterValue(publish_freq,    value_type=float),
        #         'output_data_type':  ParameterValue(1,               value_type=int),
        #         'cmdline_str':       bd_list,
        #         'cmdline_file_path': lvx_file_path,
        #         'user_config_path':  user_config_path,
        #         'frame_id':          msg_frame_id,
        #         'enable_lidar_bag':  ParameterValue(lidar_bag, value_type=bool),
        #         'enable_imu_bag':    ParameterValue(imu_bag,   value_type=bool),
        #         'use_ros_time':      True,
        #         'ros_time_override': True,
        #     }],
        #     remappings=[
        #         ('livox/lidar', pc2_topic),
        #         ('/livox/lidar', pc2_topic),
        #     ],
        # ),
    ])