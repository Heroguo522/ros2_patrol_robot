from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('patrol_robot')
    config_file = os.path.join(pkg_share_dir, 'config', 'patrol_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # 注意真机时使用系统时间，把true改为false

    return LaunchDescription([
        # 将声明的参数添加到 LaunchDescription 中，这样才能从外部接收
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # 巡逻节点
        Node(
            package='patrol_robot',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}] 
        ),
        # 语音播放节点
        Node(
            package='patrol_robot',
            executable='audio_player_node',
            name='audio_player_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}] # 使用 use_sim_time 可以同步'gazebo模拟世界'时间戳
        )
    ])