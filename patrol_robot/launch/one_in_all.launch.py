import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # --- 1. 获取各个功能包的共享目录路径 ---
    
    # 包含 Gazebo 仿真的包
    robot_description_pkg_dir = get_package_share_directory('my_robot_description')
    
    # 包含 Nav2 导航的包
    robot_navigation2_pkg_dir = get_package_share_directory('robot_navigation2')

    # 包含巡逻应用的包
    patrol_robot_pkg_dir = get_package_share_directory('patrol_robot')

    # --- 2. 构造三个 launch 文件的完整路径 ---
    
    # 文件1：Gazebo 仿真
    gazebo_sim_launch_file = os.path.join(
        robot_description_pkg_dir, 'launch', 'gazebo_sim.launch.py'
    )

    # 文件2：Nav2 导航
    navigation2_launch_file = os.path.join(
        robot_navigation2_pkg_dir, 'launch', 'navigation2.launch.py'
    )

    # 文件3：巡逻应用
    patrol_launch_file = os.path.join(
        patrol_robot_pkg_dir, 'launch', 'patrol_launch.py'
    )

    # --- 3. 声明可以在命令行中配置的参数 ---
    
    # 让 use_sim_time 成为一个可配置的参数，默认值为 'true'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- 4. 定义包含动作 (Include Actions) ---

    # 包含动作1: 启动 Gazebo 仿真和机器人
    launch_simulation_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_sim_launch_file)
        # gazebo_sim.launch.py 内部应该已经处理了 use_sim_time，通常无需从外部传递
    )

    # 包含动作2: 启动 Nav2 导航系统
    launch_navigation_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation2_launch_file),
        # 将顶层的 use_sim_time 参数传递给 navigation2.launch.py
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 包含动作3: 启动巡逻和语音节点
    launch_patrol_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(patrol_launch_file)
        # 同样，我们也需要确保 patrol_launch.py 能接收和使用 use_sim_time
        # (将在下一步中说明如何修改 patrol_launch.py)
    )
    
    # --- 5. 组合成最终的 LaunchDescription ---

    return LaunchDescription([
        # 声明此 launch 文件接受 use_sim_time 参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # 依次执行包含动作
        launch_simulation_action,
        launch_navigation_action,
        launch_patrol_action
    ])