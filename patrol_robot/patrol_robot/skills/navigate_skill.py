import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class NavigateSkill(Skill):
  def __init__(self, node: Node, navigator: BasicNavigator, tf_buffer: Buffer):
    super().__init__(node, 'navigate')
    self._navigator = navigator
    self._tf_buffer = tf_buffer
    if not node.has_parameter('fault_recovery.nav_timeout_sec'):
      node.declare_parameter('fault_recovery.nav_timeout_sec', 120.0)
    self._nav_timeout_sec = float(node.get_parameter('fault_recovery.nav_timeout_sec').value)

  def execute(self, target_pose: PoseStamped, **kwargs) -> SkillResult:
    logger = self._node.get_logger()
    current_pose = self._get_current_pose()
    if current_pose is None:
      return SkillResult(
        SkillStatus.FAILED,
        '导航前 TF 不可用',
        fault_code='TF_UNAVAILABLE',
        details={'frame': 'map->base_link'},
      )
    logger.info(
      f'开始导航: (x={target_pose.pose.position.x:.2f}, '
      f'y={target_pose.pose.position.y:.2f})')
    self._navigator.goToPose(target_pose)
    start = time.monotonic()

    while not self._navigator.isTaskComplete():
      elapsed = time.monotonic() - start
      if elapsed > self._nav_timeout_sec:
        self._navigator.cancelTask()
        return SkillResult(
          SkillStatus.FAILED,
          f'导航超时({self._nav_timeout_sec:.1f}s)',
          fault_code='NAV_TIMEOUT',
          details={'timeout_sec': self._nav_timeout_sec},
        )
      feedback = self._navigator.getFeedback()
      current_pose = self._get_current_pose()
      if feedback and current_pose:
        logger.info(
          f'导航中... 位置: ({current_pose.pose.position.x:.2f}, '
          f'{current_pose.pose.position.y:.2f}) | '
          f'剩余距离: {feedback.distance_remaining:.2f} m | '
          f'预计: {feedback.estimated_time_remaining.sec} s')
      time.sleep(1)

    result = self._navigator.getResult()
    if result == TaskResult.SUCCEEDED:
      logger.info('导航成功')
      return SkillResult(SkillStatus.SUCCEEDED, '导航成功')
    if result == TaskResult.CANCELED:
      logger.warn('导航被取消')
      return SkillResult(SkillStatus.CANCELED, '导航被取消')
    if result == TaskResult.FAILED:
      logger.error('导航失败')
      return SkillResult(
        SkillStatus.FAILED,
        '导航失败',
        fault_code='NAV_FAILED',
        details={
          'target_x': target_pose.pose.position.x,
          'target_y': target_pose.pose.position.y,
        },
      )
    return SkillResult(SkillStatus.FAILED, '无效的导航结果', fault_code='NAV_FAILED')

  def cancel(self) -> None:
    self._navigator.cancelTask()

  def clear_costmaps(self) -> None:
    self._navigator.clearAllCostmaps()

  def _get_current_pose(self) -> PoseStamped | None:
    try:
      transform = self._tf_buffer.lookup_transform(
        'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
      pose = PoseStamped()
      pose.header.frame_id = 'map'
      pose.header.stamp = self._node.get_clock().now().to_msg()
      pose.pose.position.x = transform.transform.translation.x
      pose.pose.position.y = transform.transform.translation.y
      pose.pose.position.z = transform.transform.translation.z
      pose.pose.orientation = transform.transform.rotation
      return pose
    except Exception as e:
      self._node.get_logger().warn(f'无法获取当前位姿: {e}')
      return None
