from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator   # 一个关联了nav的便捷节点
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()  # 等待导航可用
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 3.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f"预计剩余距离: {feedback.distance_remaining} | 预计剩余时间: {feedback.estimated_time_remaining.sec}")
        # nav.cancelTask() 取消任务
    result = nav.getResult()
    nav.get_logger().info(f"导航结果: {result}")

    # 不用spin，因为节点内函数已经spin了