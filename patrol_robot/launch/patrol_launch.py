from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  pkg_share_dir = get_package_share_directory('patrol_robot')
  patrol_config = os.path.join(pkg_share_dir, 'config', 'patrol_config.yaml')
  capture_config = os.path.join(pkg_share_dir, 'config', 'capture_config.yaml')

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true'),

    Node(
      package='patrol_robot',
      executable='patrol_node',
      name='patrol_node',
      output='screen',
      parameters=[patrol_config, {'use_sim_time': use_sim_time}],
    ),
    Node(
      package='patrol_robot',
      executable='audio_player_node',
      name='audio_player_node',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}],
    ),
    Node(
      package='patrol_robot',
      executable='capture_image_node',
      name='capture_image_node',
      output='screen',
      parameters=[capture_config, {'use_sim_time': use_sim_time}],
    ),
  ])
