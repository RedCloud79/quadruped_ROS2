import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # ✅ 추가

def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # ✅ xacro 실행 결과를 robot_description으로 전달
    robot_desc = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    # ✅ 공통 오버라이드(별칭 포함)
    common_override = {
        'lidarFrame':     'livox_frame',
        'baselinkFrame':  'livox_frame',   # (임시 안정화)
        'baseLinkFrame':  'livox_frame',   # 별칭
        'odometryFrame':  'odom',
        'odomFrame':      'odom',          # 별칭
        'mapFrame':       'map',
    }

    return LaunchDescription([
        params_declare,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        Node(
            package='lio_sam', executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration', output='screen',
            parameters=[parameter_file, common_override],
        ),
        Node(
            package='lio_sam', executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection', output='screen',
            parameters=[parameter_file, common_override],
        ),
        Node(
            package='lio_sam', executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction', output='screen',
            parameters=[parameter_file, common_override],
        ),
        Node(
            package='lio_sam', executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization', output='screen',
            parameters=[parameter_file, common_override],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])
