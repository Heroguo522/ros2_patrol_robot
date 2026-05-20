import threading
import time
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from patrol_interfaces.msg import RobotStatus

from patrol_robot.skills.base import SkillResult, SkillStatus
from patrol_robot.skills.capture_image_skill import CaptureImageSkill
from patrol_robot.skills.navigate_skill import NavigateSkill
from patrol_robot.skills.speak_skill import SpeakSkill
from patrol_robot.utils.pose_utils import parse_patrol_points, parse_waypoint_strings


class PatrolTaskState(Enum):
  BOOTSTRAP = auto()
  IDLE = auto()
  PATROLLING = auto()
  NAVIGATING = auto()
  AT_WAYPOINT = auto()
  RETRY_WAIT = auto()
  PAUSED = auto()
  FINISHED = auto()


IOT_STATE_MAP = {
  PatrolTaskState.BOOTSTRAP: 'initializing',
  PatrolTaskState.IDLE: 'idle',
  PatrolTaskState.PATROLLING: 'idle',
  PatrolTaskState.NAVIGATING: 'navigating',
  PatrolTaskState.AT_WAYPOINT: 'at_waypoint',
  PatrolTaskState.RETRY_WAIT: 'retry_wait',
  PatrolTaskState.PAUSED: 'paused',
  PatrolTaskState.FINISHED: 'finished',
}


class TaskManager:
  def __init__(
    self,
    node: BasicNavigator,
    navigate_skill: NavigateSkill,
    speak_skill: SpeakSkill,
    capture_skill: CaptureImageSkill,
    status_publisher,
    robot_id: str = 'robot_001',
  ):
    self._node = node
    self._navigate = navigate_skill
    self._speak = speak_skill
    self._capture = capture_skill
    self._publish_status = status_publisher
    self._robot_id = robot_id
    self._logger = node.get_logger()
    self._lock = threading.Lock()

    self._node.declare_parameter('navigate_retry_wait_sec', 60.0)
    self._node.declare_parameter('waypoint_stabilize_sec', 2.0)
    self._node.declare_parameter('inter_waypoint_delay_sec', 3.0)
    self._node.declare_parameter('auto_start_local_patrol', True)
    self._node.declare_parameter('default_task_id', 'local_patrol')

    self._retry_wait_sec = self._node.get_parameter('navigate_retry_wait_sec').value
    self._stabilize_sec = self._node.get_parameter('waypoint_stabilize_sec').value
    self._inter_delay_sec = self._node.get_parameter('inter_waypoint_delay_sec').value
    self._auto_start = self._node.get_parameter('auto_start_local_patrol').value

    self._patrol_points: list[PoseStamped] = []
    self._point_index = 0
    self._state = PatrolTaskState.BOOTSTRAP
    self._task_id = self._node.get_parameter('default_task_id').value
    self._paused = False
    self._cancel_requested = False
    self._fault_code = ''
    self._patrol_active = False
    self._pending_start = False
    self._pending_waypoints: list[str] | None = None
    self._pending_task_id: str | None = None
    self._pending_initial_pose: tuple[float, float, float] | None = None

  @property
  def iot_state(self) -> str:
    if self._fault_code:
      return 'fault'
    return IOT_STATE_MAP.get(self._state, 'idle')

  def _set_state(self, state: PatrolTaskState) -> None:
    with self._lock:
      self._state = state
    self._emit_status()

  def _emit_status(self) -> None:
    with self._lock:
      msg = RobotStatus()
      msg.robot_id = self._robot_id
      msg.task_id = self._task_id
      msg.state = self.iot_state
      msg.waypoint_index = self._point_index
      msg.waypoint_total = len(self._patrol_points)
      msg.fault_code = self._fault_code
      msg.battery_percent = 0.0
      msg.stamp = self._node.get_clock().now().to_msg()
    self._publish_status(msg)

  def _init_robot_pose(
    self,
    x: float | None = None,
    y: float | None = None,
    yaw: float | None = None,
  ) -> None:
    init_x = x if x is not None else self._node.get_parameter('initial_pose.x').value
    init_y = y if y is not None else self._node.get_parameter('initial_pose.y').value
    init_yaw = yaw if yaw is not None else self._node.get_parameter('initial_pose.yaw').value
    from patrol_robot.utils.pose_utils import pose_from_xyyaw
    initial_pose = pose_from_xyyaw(self._node, init_x, init_y, init_yaw)
    self._node.setInitialPose(initial_pose)
    self._logger.info(f'初始位姿: x={init_x}, y={init_y}, yaw={init_yaw}')

  def load_patrol_route(self, waypoints: list[str] | None = None) -> bool:
    if waypoints is not None:
      self._patrol_points = parse_waypoint_strings(
        self._node, self._logger, waypoints)
    else:
      self._patrol_points = parse_patrol_points(self._node, self._logger)
    self._point_index = 0
    self._emit_status()
    return len(self._patrol_points) > 0

  def submit_patrol_task(
    self,
    task_id: str,
    waypoints: list[str],
    initial_pose: tuple[float, float, float] | None = None,
  ) -> tuple[bool, str]:
    if not waypoints:
      return False, 'waypoints 不能为空'
    with self._lock:
      self._pending_start = True
      self._pending_waypoints = list(waypoints)
      self._pending_task_id = task_id
      self._pending_initial_pose = initial_pose
      self._cancel_requested = True
    return True, '任务已排队, 将尽快启动'

  def control_patrol(self, action: int) -> tuple[bool, str]:
    from patrol_interfaces.srv import ControlPatrol
    if action == ControlPatrol.Request.PAUSE:
      with self._lock:
        self._paused = True
      self._set_state(PatrolTaskState.PAUSED)
      return True, '巡逻已暂停'
    if action == ControlPatrol.Request.RESUME:
      with self._lock:
        if not self._patrol_active:
          return False, '当前无进行中的巡逻'
        self._paused = False
      self._set_state(PatrolTaskState.PATROLLING)
      return True, '巡逻已恢复'
    if action == ControlPatrol.Request.CANCEL:
      with self._lock:
        self._cancel_requested = True
        self._patrol_active = False
      self._navigate.cancel()
      self._fault_code = ''
      self._set_state(PatrolTaskState.IDLE)
      return True, '巡逻已取消'
    return False, f'未知 action: {action}'

  def _apply_pending_task(self) -> bool:
    with self._lock:
      if not self._pending_start or not self._pending_waypoints:
        return False
      waypoints = self._pending_waypoints
      task_id = self._pending_task_id or self._task_id
      initial_pose = self._pending_initial_pose
      self._pending_start = False
      self._pending_waypoints = None
      self._pending_task_id = None
      self._pending_initial_pose = None
      self._cancel_requested = False
      self._paused = False
      self._fault_code = ''

    self._task_id = task_id
    if initial_pose is not None:
      self._init_robot_pose(*initial_pose)
    if not self.load_patrol_route(waypoints):
      return False
    self._patrol_active = True
    self._logger.info(f'远程任务已加载: {task_id}, 共 {len(self._patrol_points)} 个点')
    return True

  def _run_speak(self, text: str) -> SkillResult:
    result = self._speak.execute(text=text)
    if not result.succeeded:
      self._logger.warn(f'语音技能未成功: {result.message}')
    return result

  def _run_waypoint_actions(self) -> None:
    if self._check_cancel_or_pause():
      return
    self._logger.info('导航成功, 执行到点动作链')
    self._run_speak('已到达目标点, 准备拍照')
    if self._check_cancel_or_pause():
      return
    time.sleep(self._stabilize_sec)
    capture_result = self._capture.execute()
    if not capture_result.succeeded:
      self._fault_code = 'CAPTURE_FAILED'
      self._emit_status()
      self._logger.warn(f'拍照未成功: {capture_result.message}')
    time.sleep(1.0)
    self._run_speak('拍照完成')

  def _check_cancel_or_pause(self) -> bool:
    with self._lock:
      if self._cancel_requested:
        return True
    while True:
      with self._lock:
        if self._cancel_requested:
          return True
        if not self._paused:
          return False
      time.sleep(0.2)

  def _patrol_loop(self) -> None:
    while rclpy.ok():
      if self._apply_pending_task():
        self._run_speak('巡逻任务开始')

      with self._lock:
        active = self._patrol_active
        state = self._state

      if not active or state == PatrolTaskState.FINISHED:
        if state == PatrolTaskState.FINISHED:
          break
        time.sleep(0.2)
        continue

      if self._check_cancel_or_pause():
        with self._lock:
          if self._cancel_requested:
            self._patrol_active = False
            self._set_state(PatrolTaskState.IDLE)
            continue

      if not self._patrol_points:
        time.sleep(0.2)
        continue

      target = self._patrol_points[self._point_index]
      total = len(self._patrol_points)
      self._logger.info(f'--- 前往巡逻点 {self._point_index + 1}/{total} ---')

      self._set_state(PatrolTaskState.NAVIGATING)
      nav_result = self._navigate.execute(target_pose=target)

      if self._check_cancel_or_pause():
        continue

      if nav_result.status == SkillStatus.SUCCEEDED:
        self._set_state(PatrolTaskState.AT_WAYPOINT)
        self._fault_code = ''
        self._run_waypoint_actions()
      else:
        self._fault_code = 'NAV_FAILED'
        self._set_state(PatrolTaskState.RETRY_WAIT)
        self._run_speak('导航失败, 请检查环境或地图, 将在一分钟后重试')
        self._logger.warn(
          f'导航失败, 等待 {self._retry_wait_sec:.0f} 秒后重试当前点...')
        time.sleep(self._retry_wait_sec)
        continue

      self._point_index = (self._point_index + 1) % total
      self._set_state(PatrolTaskState.PATROLLING)

      if total > 1:
        self._run_speak('三秒后前往下一个目标点')
        time.sleep(self._inter_delay_sec)
      else:
        self._logger.info('已完成单点巡逻, 任务结束')
        self._patrol_active = False
        self._set_state(PatrolTaskState.FINISHED)
        break

  def run(self) -> None:
    self._set_state(PatrolTaskState.BOOTSTRAP)
    self._init_robot_pose()
    time.sleep(1.0)
    self._node.waitUntilNav2Active()
    self._logger.info('Nav2 已激活')
    self._set_state(PatrolTaskState.IDLE)

    if self._auto_start and self.load_patrol_route():
      with self._lock:
        self._patrol_active = True
      self._logger.info('auto_start_local_patrol: 使用 YAML 路点启动')
    else:
      self._logger.info('等待远程任务 (submit_patrol_task 或 MQTT)')

    self._patrol_loop()
