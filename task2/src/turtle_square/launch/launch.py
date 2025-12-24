from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    eg_arg = DeclareLaunchArgument(
        'eg', default_value='10'
    )
    length_arg = DeclareLaunchArgument(
        'length', default_value='1.0'
    )

    eg = LaunchConfiguration('eg')
    length = LaunchConfiguration('length')

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    turtle_square_node = Node(
        package='turtle_square', 
        executable='turtle_square_node',  
        name='turtle_square',
        output='screen',
        parameters=[{'eg': 20, 'length': length}]
    )

    # turtle_circle_node = Node(
    #     package='turtle_square', 
    #     executable='turtle_circle_node',  
    #     name='turtle_circle',
    #     output='screen',
    #     parameters=[{'length': 3.0}]
    # )

    return LaunchDescription([
        eg_arg,
        length_arg,
        turtlesim_node,
        turtle_square_node,
        # turtle_circle
    ])