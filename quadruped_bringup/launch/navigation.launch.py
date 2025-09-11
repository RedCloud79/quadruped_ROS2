from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 패키지 경로
    from ament_index_python.packages import get_package_share_directory
    tier4_localization_launch = get_package_share_directory('tier4_localization_launch')

    # 공통 config path
    config_path = os.path.join(tier4_localization_launch, 'config')

    # 필요 argument 선언
    lidar_marker_param = DeclareLaunchArgument(
        'lidar_marker_localizer_param_path',
        default_value=os.path.join(config_path, 'lidar_marker_localizer.param.yaml'),
        description='Param file for lidar marker localizer'
    )

    crop_box_param = DeclareLaunchArgument(
        'crop_box_filter_measurement_range_param_path',
        default_value=os.path.join(config_path, 'crop_box_filter.param.yaml'),
        description='Param file for crop box filter'
    )

    # Include 개별 launch.xml (경로 수정 완료 ✅)
    lidar_marker_localizer = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(tier4_localization_launch, 'launch', 'pose_twist_estimator', 'lidar_marker_localizer.launch.xml')
        ),
        launch_arguments={
            'lidar_marker_localizer_param_path': LaunchConfiguration('lidar_marker_localizer_param_path'),
            'crop_box_filter_measurement_range_param_path': LaunchConfiguration('crop_box_filter_measurement_range_param_path')
        }.items()
    )

    ndt_scan_matcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(tier4_localization_launch, 'launch', 'pose_twist_estimator', 'ndt_scan_matcher.launch.xml')
        )
    )

    pose_twist_estimator = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(tier4_localization_launch, 'launch', 'pose_twist_estimator', 'pose_twist_estimator.launch.xml')
        )
    )

    # 전체 launch description 반환
    return LaunchDescription([
        lidar_marker_param,
        crop_box_param,
        lidar_marker_localizer,
        ndt_scan_matcher,
        pose_twist_estimator
    ])
