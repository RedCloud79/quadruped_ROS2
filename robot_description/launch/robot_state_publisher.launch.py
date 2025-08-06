import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')

    urdf_file_name = 'Lite3.urdf' 
    # xacro_file_name = 'robot.urdf.xacro'
    urdf_file_path = os.path.join(pkg_path, 'urdf', urdf_file_name)

    # Xacro 파일을 XML 문자열로 변환
    # robot_description_config = xacro.process_file(xacro_file_path)
    # robot_description_xml = robot_description_config.toxml()

    with open(urdf_file_path, 'r') as f:
        robot_description_xml = f.read()

    params = {'robot_description': robot_description_xml, 'use_sim_time': False}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])