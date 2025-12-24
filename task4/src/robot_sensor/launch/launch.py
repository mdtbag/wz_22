from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_sensor',
            executable='robot_sensor_node',
            name='node_1',
            parameters=[{'frequency':2.0,'data_file_path':"/home/zqj/Desktop/zhouquanjing_202504080705/task4/src/robot_sensor/data/data.txt",'observation_frame':"base_link",'loop_data':True}]
        ),
    ])
