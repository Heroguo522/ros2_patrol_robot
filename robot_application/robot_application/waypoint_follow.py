from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time

def main():
    # 1. 初始化 ROS
    rclpy.init()

    # 2. 创建 BasicNavigator 实例
    nav = BasicNavigator()

    # 等待 Nav2 完全激活
    nav.waitUntilNav2Active()
    nav.get_logger().info("Nav2 已激活，开始路点导航...")

    # 3. 定义路点数据
    # 将所有路点的核心数据（坐标和姿态）集中存放在一个列表中。
    # 每个元组的格式为: (位置x, 位置y, 姿态orientation.z, 姿态orientation.w)
    # 姿态z和w通常一起使用来表示绕Z轴的旋转（偏航角）。
    waypoints_data = [
        (2.0, 1.0, 0.0,   1.0),      # 路点1: 指向正X轴
        (4.0, 3.0, 0.707, 0.707),    # 路点2: 旋转90度，指向正Y轴
        (1.0, 4.0, 1.0,   0.0),      # 路点3: 旋转180度，指向负X轴
        (0.5, 0.5, 0.0,   1.0)       # 路点4: 回到原点附近，指向正X轴
    ]

    # 4. 通过循环创建 PoseStamped 对象列表
    waypoints = []
    for x, y, oz, ow in waypoints_data:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # 2D导航中Z坐标通常为0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        waypoints.append(pose)

    # 5. 发送路点列表
    nav.get_logger().info(f"准备发送 {len(waypoints)} 个路点...")
    nav.followWaypoints(waypoints)

    # 6. 监控任务执行状态
    i = 0
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback and i != feedback.current_waypoint:
            i = feedback.current_waypoint
            nav.get_logger().info(f'正在前往路点 {i+1}/{len(waypoints)} ...')
        time.sleep(0.5)

    # 7. 获取并打印最终结果
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('任务成功: 所有路点都已到达!')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('任务被取消!')
    elif result == TaskResult.FAILED:
        nav.get_logger().error('任务失败!')
    else:
        nav.get_logger().info(f'任务状态未知: {result}')

    # 8. 清理
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()