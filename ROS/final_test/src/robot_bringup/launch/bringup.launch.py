from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_tracker',
            executable='tracker_node',
            name='robot_tracker',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='robot_solver',
            executable='solver_node',
            name='robot_solver',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='robot_controller',
            executable='collect_server',
            name='robot_controller',
            output='screen',
            parameters=[params],
        ),
    ])