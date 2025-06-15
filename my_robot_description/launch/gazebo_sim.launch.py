import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory  
import os
from launch_ros.actions import Node
import launch_ros.parameter_descriptions

def generate_launch_description():

    urdf_package_path = get_package_share_directory('my_robot_description') 
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'robot.urdf.xacro')  
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'robot_model.rviz')
    default_gazebo_wolrd_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')
    
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_path,
        description='加载的模型文件路径'
    )

    command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])  
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)   

    action_robot_state_pubisher = launch_ros.actions.Node(
        package='robot_state_publisher',    # 可以把urdf文件通过话题robot_description发布!
        executable='robot_state_publisher',    
        parameters=[{'robot_description':robot_description_value}]  
    )

    # ros2_control yaml配置文件中定义了关节广播控制器，这里通过命令行启用
    action_load_joint_control = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller robot_joint_state_broadcaster --set-state active'.split(),
        output='screen'
    )

    # 启用两轮差速控制器
    action_load_diff_driver_control = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller robot_diff_driver_controller --set-state active'.split(),
        output='screen'
    )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(   # 包含其它launch文件
        # ros2 launch gazebo_ros gazebo.launch.py world:=xxx.world
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"]  # 注意是数组
        ),
        launch_arguments=[('world', default_gazebo_wolrd_path), ('verbose', 'true')]
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]    # 相当于直接在后面添加命令
    )

    action_spawn_entity = launch_ros.actions.Node(    # 在Gazebo生成机器人模型
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'robot']
    )


    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period = 0.0, actions = [action_declare_arg_mode_path]),
        launch.actions.TimerAction(period = 0.2, actions = [action_robot_state_pubisher]),
        launch.actions.TimerAction(period = 0.4, actions = [action_launch_gazebo]),
        #launch.actions.TimerAction(period = 0.6, actions = [action_rviz_node]),
        launch.actions.TimerAction(period = 0.8, actions = [action_spawn_entity]),
        launch.actions.TimerAction(period = 1.0, actions = [action_load_joint_control]),
        launch.actions.TimerAction(period = 1.2, actions = [action_load_diff_driver_control]),
    ])    

    return launch.LaunchDescription([
        action_group
    ])