#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from patrol_interfaces.msg import RobotStatus
from patrol_interfaces.srv import ControlPatrol, SubmitPatrolTask
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from patrol_robot.skills.capture_image_skill import CaptureImageSkill
from patrol_robot.skills.navigate_skill import NavigateSkill
from patrol_robot.skills.speak_skill import SpeakSkill
from patrol_robot.task_manager import TaskManager


class PatrolNode(BasicNavigator):
  """巡逻编排节点：TaskManager + Skill + 远程任务服务。"""

  def __init__(self):
    super().__init__('patrol_node')

    self.declare_parameter('robot_id', 'robot_001')
    self.robot_id = self.get_parameter('robot_id').value

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.declare_parameter('initial_pose.x', 0.0)
    self.declare_parameter('initial_pose.y', 0.0)
    self.declare_parameter('initial_pose.yaw', 0.0)
    self.declare_parameter('patrol_points', rclpy.Parameter.Type.STRING_ARRAY)

    self._status_pub = self.create_publisher(RobotStatus, '/robot/status', 10)

    navigate_skill = NavigateSkill(self, self, self.tf_buffer)
    speak_skill = SpeakSkill(self)
    capture_skill = CaptureImageSkill(self)
    self.task_manager = TaskManager(
      self,
      navigate_skill,
      speak_skill,
      capture_skill,
      status_publisher=self._publish_robot_status,
      robot_id=self.robot_id,
    )

    self._submit_srv = self.create_service(
      SubmitPatrolTask, 'submit_patrol_task', self._handle_submit_patrol_task)
    self._control_srv = self.create_service(
      ControlPatrol, 'control_patrol', self._handle_control_patrol)

    self.get_logger().info('巡逻节点已初始化 (TaskManager + 远程任务服务)')

  def _publish_robot_status(self, msg: RobotStatus) -> None:
    self._status_pub.publish(msg)

  def _handle_submit_patrol_task(self, request, response):
    initial_pose = None
    if request.use_initial_pose:
      initial_pose = (
        request.initial_pose_x,
        request.initial_pose_y,
        request.initial_pose_yaw,
      )
    ok, message = self.task_manager.submit_patrol_task(
      request.task_id,
      list(request.waypoints),
      initial_pose,
    )
    response.success = ok
    response.message = message
    return response

  def _handle_control_patrol(self, request, response):
    ok, message = self.task_manager.control_patrol(request.action)
    response.success = ok
    response.message = message
    return response


def main(args=None):
  rclpy.init(args=args)
  patrol_node = PatrolNode()

  executor = MultiThreadedExecutor()
  executor.add_node(patrol_node)
  spin_thread = threading.Thread(target=executor.spin, daemon=True)
  spin_thread.start()

  task_thread = threading.Thread(
    target=patrol_node.task_manager.run, daemon=True)
  task_thread.start()

  try:
    task_thread.join()
  except KeyboardInterrupt:
    patrol_node.get_logger().info('用户中断, 正在关闭...')
  finally:
    patrol_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
