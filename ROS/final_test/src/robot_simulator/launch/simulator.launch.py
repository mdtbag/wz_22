from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('robot_simulator')
    
    # 参数文件路径
    params_file = os.path.join(pkg_share, 'config', 'simulator_params.yaml')
    
    # URDF 文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # 声明启动参数
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to the simulator parameters file'
    )
    
    # 读取 URDF 内容（使用 xacro 处理）
    robot_description = Command(['xacro ', urdf_file])
    
    # Simulator 节点
    simulator_node = Node(
        package='robot_simulator',
        executable='simulator_node',
        name='simulator_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True
    )
    
    # Robot State Publisher 节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        emulate_tty=True
    )
    
    return LaunchDescription([
        declare_params_file,
        simulator_node,
        robot_state_publisher
    ])
