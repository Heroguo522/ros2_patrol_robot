import time
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from patrol_robot.skills.base import SkillResult, SkillStatus
from patrol_robot.skills.capture_image_skill import CaptureImageSkill
from patrol_robot.skills.navigate_skill import NavigateSkill
from patrol_robot.skills.speak_skill import SpeakSkill
from patrol_robot.utils.pose_utils import parse_patrol_points, pose_from_xyyaw


class PatrolTaskState(Enum):
  BOOTSTRAP = auto()
  PATROLLING = auto()
  NAVIGATING = auto()
  AT_WAYPOINT = auto()
  RETRY_WAIT = auto()
  FINISHED = auto()


class TaskManager:
  def __init__(
    self,
    node: BasicNavigator,
    navigate_skill: NavigateSkill,
    speak_skill: SpeakSkill,
    capture_skill: CaptureImageSkill,
  ):
    self._node = node
    self._navigate = navigate_skill
    self._speak = speak_skill
    self._capture = capture_skill
    self._logger = node.get_logger()

    self._node.declare_parameter('navigate_retry_wait_sec', 60.0)
    self._node.declare_parameter('waypoint_stabilize_sec', 2.0)
    self._node.declare_parameter('inter_waypoint_delay_sec', 3.0)

    self._retry_wait_sec = self._node.get_parameter(
      'navigate_retry_wait_sec').value
    self._stabilize_sec = self._node.get_parameter(
      'waypoint_stabilize_sec').value
    self._inter_delay_sec = self._node.get_parameter(
      'inter_waypoint_delay_sec').value

    self._patrol_points: list[PoseStamped] = []
    self._point_index = 0
    self._state = PatrolTaskState.BOOTSTRAP

  def _init_robot_pose(self) -> None:
    self._logger.info('正在初始化机器人位姿...')
    init_x = self._node.get_parameter('initial_pose.x').value
    init_y = self._node.get_parameter('initial_pose.y').value
    init_yaw = self._node.get_parameter('initial_pose.yaw').value
    initial_pose = pose_from_xyyaw(self._node, init_x, init_y, init_yaw)
    self._node.setInitialPose(initial_pose)
    self._logger.info(
      f'初始位姿: x={init_x}, y={init_y}, yaw={init_yaw}')

  def load_patrol_route(self) -> bool:
    self._patrol_points = parse_patrol_points(self._node, self._logger)
    return len(self._patrol_points) > 0

  def _run_speak(self, text: str) -> SkillResult:
    result = self._speak.execute(text=text)
    if not result.succeeded:
      self._logger.warn(f'语音技能未成功: {result.message}')
    return result

  def _run_waypoint_actions(self) -> None:
    self._logger.info('导航成功, 执行到点动作链')
    self._run_speak('已到达目标点, 准备拍照')
    time.sleep(self._stabilize_sec)
    capture_result = self._capture.execute()
    if not capture_result.succeeded:
      self._logger.warn(f'拍照未成功: {capture_result.message}')
    time.sleep(1.0)
    self._run_speak('拍照完成')

  def run(self) -> None:
    self._init_robot_pose()
    time.sleep(1.0)
    self._node.waitUntilNav2Active()
    self._logger.info('Nav2 已激活')

    if not self.load_patrol_route():
      self._logger.error('没有可用巡逻点, 任务退出')
      return

    self._state = PatrolTaskState.PATROLLING
    self._run_speak('巡逻任务开始')

    while rclpy.ok():
      if self._state == PatrolTaskState.FINISHED:
        break

      target = self._patrol_points[self._point_index]
      total = len(self._patrol_points)
      self._logger.info(
        f'--- 前往巡逻点 {self._point_index + 1}/{total} ---')

      self._state = PatrolTaskState.NAVIGATING
      nav_result = self._navigate.execute(target_pose=target)

      if nav_result.status == SkillStatus.SUCCEEDED:
        self._state = PatrolTaskState.AT_WAYPOINT
        self._run_waypoint_actions()
      else:
        self._state = PatrolTaskState.RETRY_WAIT
        self._run_speak('导航失败, 请检查环境或地图, 将在一分钟后重试')
        self._logger.warn(
          f'导航失败, 等待 {self._retry_wait_sec:.0f} 秒后重试当前点...')
        time.sleep(self._retry_wait_sec)
        continue

      self._point_index = (self._point_index + 1) % total
      self._state = PatrolTaskState.PATROLLING

      if total > 1:
        self._run_speak('三秒后前往下一个目标点')
        time.sleep(self._inter_delay_sec)
      else:
        self._logger.info('已完成单点巡逻, 任务结束')
        self._state = PatrolTaskState.FINISHED
