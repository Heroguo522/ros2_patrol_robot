# 为什么需要launch启动
# ->rviz默认读取tf数据,因将urdf定义的相对位置通过joint_state_publisher和robot_state_publisher发布tf
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory   # 找到包的shared目录(记得在CMake install shared)
import os
# 注意：该代码只用于展示机器人模型
# 使用gazebo启动后建议单独打开rviz或写入gazebo启动文件内
import launch_ros.parameter_descriptions

def generate_launch_description():
    # 获取默认urdf路径
    urdf_package_path = get_package_share_directory('my_robot_description')  # 返回my_robot_description/share/fishbot_description
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'robot.urdf.xacro')    
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    
    # 声明urdf目录参数,可修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_path,
        description='加载的模型文件路径'
    )

    #通过文件路径获取内容转换为参数对象
    command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])   # 该命令返回执行后内容,有空格 有空格 有空格!!!! 请注意cat->xacro可行但源码ros2无法安装,请在3rd_party运行转化
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)   # 转化成launch参数值对象

    action_robot_state_pubisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]   # 节点运行要求,参数是urdf内容,执行方式为: --ros-args -p robot_description:
    )

    action_joint_state_pubisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]    # 相当于直接在后面添加命令
    )

    action_group = launch.actions.GroupAction([

        launch.actions.TimerAction(period = 0.0, actions = [action_declare_arg_mode_path]),
        launch.actions.TimerAction(period = 0.5, actions = [action_robot_state_pubisher]),  
        launch.actions.TimerAction(period = 1.0, actions = [action_joint_state_pubisher]),
        launch.actions.TimerAction(period = 1.5, actions = [action_rviz_node]),
    ])    

    return launch.LaunchDescription([
        action_group
    ])


# action_joint_state_pubisher
# 寻找一个名为 robot_description 的ROS参数，读取URDF，从而知道机器人上有哪些非固定的关节（比如 continuous 或 revolute 类型的关节）
# 它会持续地向一个名为 /joint_states 的话题发布“当前，左轮转了0度，右轮转了0度，手臂关节1也转了0度“
# 在真机或带有控制插件的仿真中，我们不会启动 joint_state_publisher 或 joint_state_publisher_gui
# 取而代之的是，ros2_control 框架中的 joint_state_broadcaster 会接管发布 /joint_states 的任务，它发布的是来自编码器或仿真物理引擎的真实数据
# robot_description参数来源：两者共享了同一个“命名空间”下的参数，joint_state_publisher内部已经写好了self.get_parameter('robot_description').get_parameter_value().string_value

# action_robot_state_publisher
# 寻找一个名为 robot_description 的ROS参数，订阅 /joint_states 话题，从而正向运动学计算，它会对机器人模型中的每一个连杆重复这个计算过程，形成一个完整的“位姿树”
#  它会将这个计算出的、完整的、实时的“位姿树”以 TF 消息的形式发布到 /tf 和 /tf_static 话题上
# 它读取URDF，看到 base_link -> laser_link 是 fixed 关节。它想：“这个关系永远不变。” 于是，它计算一次这个TF，然后发布到 /tf_static 话题
# 它又看到 base_link -> left_wheel_link 是 continuous 关节。它想：“这个关系会变，我得听 /joint_states 的。” 于是，每当 /joint_states 有新消息来，它就结合新角度重新计算这个TF，然后发布到 /tf 话题

# 总结：
# 控制指令:
# 你 (或者一个导航节点) 发布 Twist 消息到 /cmd_vel。
# 仿真器执行与状态反馈:
# Gazebo插件 订阅 /cmd_vel。
# 它在Gazebo物理引擎中驱动轮子转动。
# 它从物理引擎中读取轮子仿真的当前角度。
# 它将这些角度打包成 JointState 消息，发布到 /joint_states 话题。
# TF计算与广播:
# robot_state_publisher 节点在启动时已经从参数服务器读取了 robot_description (URDF)。
# 它持续订阅 /joint_states 话题。
# 当新消息到达时:
# 它结合URDF中的静态尺寸和 JointState 消息中的动态角度，进行正向运动学计算。
# 它把计算出的固定关节的TF发布到 /tf_static (只发一次)。
# 它把计算出的可动关节的TF发布到 /tf (持续更新)。
# 最终使用者:
# RViz2 订阅 /tf 和 /tf_static。
# 根据收到的TF数据，实时地、正确地在3D世界中绘制出机器人的每一个连杆（link），让我们看到一个能动的机器人模型。
# 而真机会根据编码器使用 joint_state_broadcaster 接管发布 /joint_states 的任务