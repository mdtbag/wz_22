from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # sir 节点
        Node(
            package='hahac',
            executable='hahac_i',
            name='sir'
        ),
        # 多个 So 节点
        Node(
            package='hahac',
            executable='hahac_s',
            name='So_1'
        ),
        Node(
            package='hahac',
            executable='hahac_s',
            name='So_2'
        ),
        Node(
            package='hahac',
            executable='hahac_s',
            name='So_3'
        ),
        # 可继续添加 So_4, So_5 ...
        Node(
            package='hahac',
            executable='hahac_s',
            name='So_4'
        ),
        Node(
            package='hahac',
            executable='hahac_s',
            name='So_5'
        )
    ])