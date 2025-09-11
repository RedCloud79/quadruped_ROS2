import os
from launch import LaunchDescription
from launch.frontend import Parser
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_dir = get_package_share_directory('quadruped_bringup')

    return LaunchDescription([
        # EKF (robot_localization)
        # Node(
        #     package='robot_localization',
        #     executable='ekf_localization_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[os.path.join(my_dir, 'config', 'ekf_nav2.yaml')]
        # ),

        # Fast-LIO (odometry)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('fast_lio'),
                    'launch', 'odom_slam.launch.py'
                )
            ])
        ),

        # Autoware Localization
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('tier4_localization_launch'),
                    'launch', 'localization.launch.xml'
                )
            ),
            launch_arguments={
                'pose_source': 'ndt',
                'twist_source': 'ndt',
                'initial_pose': '0.0 0.0 0.0 0.0',
                'system_run_mode': 'localization',

                # ndt scan matcher params
                'ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/ndt_scan_matcher/crop_box_filter_measurement_range.param.yaml'),
                'ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_downsample_filter_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/ndt_scan_matcher/voxel_grid_downsample_filter.param.yaml'),
                'ndt_scan_matcher/pointcloud_preprocessor/random_downsample_filter_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/ndt_scan_matcher/random_downsample_filter.param.yaml'),
                'ndt_scan_matcher/ndt_scan_matcher_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/ndt_scan_matcher/ndt_scan_matcher.param.yaml'),

                # other modules
                'localization_error_monitor_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/localization_error_monitor/localization_error_monitor.param.yaml'),
                'ekf_localizer_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/ekf_localizer/ekf_localizer.param.yaml'),
                'stop_filter_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/stop_filter/stop_filter.param.yaml'),
                'pose_instability_detector_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/pose_instability_detector/pose_instability_detector.param.yaml'),
                'pose_initializer_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/pose_initializer/pose_initializer.param.yaml'),
                'eagleye_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/eagleye/eagleye.param.yaml'),
                'ar_tag_based_localizer_param_path':
                    os.path.join(get_package_share_directory('tier4_localization_launch'),
                                'config/ar_tag_based_localizer/ar_tag_based_localizer.param.yaml'),
            }.items(),
        ),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'params_file': os.path.join(my_dir, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'false'
            }.items(),
        ),
    ])
