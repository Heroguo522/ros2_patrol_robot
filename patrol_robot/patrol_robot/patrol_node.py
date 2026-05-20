#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from patrol_robot.skills.capture_image_skill import CaptureImageSkill
from patrol_robot.skills.navigate_skill import NavigateSkill
from patrol_robot.skills.speak_skill import SpeakSkill
from patrol_robot.task_manager import TaskManager


class PatrolNode(BasicNavigator):
  """巡逻编排节点：TaskManager + Skill 客户端，不含相机与语音实现。"""

  def __init__(self):
    super().__init__('patrol_node')

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.declare_parameter('initial_pose.x', 0.0)
    self.declare_parameter('initial_pose.y', 0.0)
    self.declare_parameter('initial_pose.yaw', 0.0)
    self.declare_parameter('patrol_points', rclpy.Parameter.Type.STRING_ARRAY)

    navigate_skill = NavigateSkill(self, self, self.tf_buffer)
    speak_skill = SpeakSkill(self)
    capture_skill = CaptureImageSkill(self)
    self.task_manager = TaskManager(
      self, navigate_skill, speak_skill, capture_skill)

    self.get_logger().info('巡逻节点已初始化 (TaskManager + Skills)')


def main(args=None):
  rclpy.init(args=args)
  patrol_node = PatrolNode()

  executor = MultiThreadedExecutor()
  executor.add_node(patrol_node)
  spin_thread = threading.Thread(target=executor.spin, daemon=True)
  spin_thread.start()

  try:
    patrol_node.task_manager.run()
  except KeyboardInterrupt:
    patrol_node.get_logger().info('用户中断, 正在关闭...')
  finally:
    patrol_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
