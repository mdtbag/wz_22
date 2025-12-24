from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='robot_control',
            executable='lidar_publisher',
            name='lidar_publisher',
            parameters=[params_file]
        ),
        Node(
            package='robot_control',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[params_file]
        ),
        Node(
            package='robot_control',
            executable='robot_service',
            name='robot_service'
        ),
        Node(
            package='robot_control',
            executable='move_action_server',
            name='move_action_server'
        ),
    ])
