from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='sim'),
        Node(package='turtle_chase', executable='turtle_chase_node',
             parameters=[{'target_name': 'turtle2'}]),
        Node(package='turtle_chase', executable='turtle_escape',
             parameters=[{'self_name': 'turtle2', 'chaser_name': 'turtle1'}])
    ])