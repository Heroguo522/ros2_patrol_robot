import math
import os
import threading
import time
from enum import Enum, auto
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator
from patrol_interfaces.msg import RobotStatus

from patrol_robot.orchestrator import (
  ExecutionContext,
  SkillRegistry,
  TaskDef,
  TaskLibrary,
  TaskLoader,
  TaskOrchestrator,
)
from patrol_robot.faults.fault_manager import FaultManager
from patrol_robot.faults.recovery_policy import RecoveryPolicy
from patrol_robot.skills.base import SkillResult, SkillStatus
from patrol_robot.skills.capture_image_skill import CaptureImageSkill
from patrol_robot.skills.detect_anomaly_skill import DetectAnomalySkill
from patrol_robot.skills.detect_workpiece_skill import WorkpieceDetectSkill
from patrol_robot.skills.navigate_skill import NavigateSkill
from patrol_robot.skills.report_skill import ReportSkill
from patrol_robot.skills.screw_driving_skill import ScrewDrivingSkill
from patrol_robot.skills.speak_skill import SpeakSkill
from patrol_robot.utils.pose_utils import pose_from_xyyaw


class PatrolTaskState(Enum):
  IDLE = auto()
  RUNNING = auto()
  PAUSED = auto()
  RECOVERING = auto()
  FAILED = auto()
  COMPLETED = auto()


class TaskManager:
  def __init__(
    self,
    node: BasicNavigator,
    navigate_skill: NavigateSkill,
    speak_skill: SpeakSkill,
    capture_skill: CaptureImageSkill,
    detect_skill: DetectAnomalySkill,
    workpiece_skill: WorkpieceDetectSkill,
    screw_skill: ScrewDrivingSkill,
    report_skill: ReportSkill,
    status_publisher,
    fault_event_publisher,
    robot_id: str = 'robot_001',
  ):
    self._node = node
    self._navigate = navigate_skill
    self._speak = speak_skill
    self._capture = capture_skill
    self._detect = detect_skill
    self._workpiece = workpiece_skill
    self._screw = screw_skill
    self._report = report_skill
    self._publish_status = status_publisher
    self._publish_fault_event = fault_event_publisher
    self._robot_id = robot_id
    self._logger = node.get_logger()
    self._lock = threading.Lock()

    self._node.declare_parameter('task_config_dir', '')
    self._node.declare_parameter('auto_start_task', True)
    self._node.declare_parameter('default_task_name', 'legacy_room_patrol')
    self._node.declare_parameter('task_completion_hold_sec', 5.0)
    self._declare_recovery_parameters()

    self._task_completion_hold_sec = float(
      self._node.get_parameter('task_completion_hold_sec').value)
    self._auto_start = self._node.get_parameter('auto_start_task').value

    self._library: TaskLibrary | None = None
    self._registry: SkillRegistry | None = None
    self._current_task: TaskDef | None = None
    self._point_index = 0
    self._step_index = 0
    self._step_total = 0
    self._current_step_type = ''
    self._current_station = ''
    self._state = PatrolTaskState.IDLE
    self._task_id = ''
    self._paused = False
    self._cancel_requested = False
    self._fault_code = ''
    self._patrol_active = False
    self._pending_start = False
    self._pending_task_name: str | None = None
    self._pending_task_id: str | None = None
    self._recovery_policy = RecoveryPolicy.from_node(self._node)
    self._fault_manager = FaultManager(
      node=self._node,
      robot_id=self._robot_id,
      event_publisher=self._publish_fault_event,
      recovery_policy=self._recovery_policy,
      navigate_skill=self._navigate,
      set_state_callback=self._set_state_by_name,
      set_fault_code_callback=self._set_fault_code,
    )
    self._orchestrator = TaskOrchestrator(
      self._logger,
      status_callback=self._on_step_status,
      fault_manager=self._fault_manager,
    )

  def _set_state(self, state: PatrolTaskState) -> None:
    with self._lock:
      self._state = state
    self._emit_status()

  def _set_state_by_name(self, state: str) -> None:
    mapping = {
      'idle': PatrolTaskState.IDLE,
      'running': PatrolTaskState.RUNNING,
      'paused': PatrolTaskState.PAUSED,
      'recovering': PatrolTaskState.RECOVERING,
      'failed': PatrolTaskState.FAILED,
      'completed': PatrolTaskState.COMPLETED,
    }
    if state not in mapping:
      self._logger.warn(f'忽略未知状态: {state}')
      return
    self._set_state(mapping[state])

  def _set_fault_code(self, fault_code: str) -> None:
    with self._lock:
      self._fault_code = fault_code
    self._emit_status()

  def _state_text(self) -> str:
    return self._state.name.lower()

  def _emit_status(self) -> None:
    with self._lock:
      msg = RobotStatus()
      msg.robot_id = self._robot_id
      msg.task_id = self._task_id
      msg.state = self._state_text()
      msg.waypoint_index = self._point_index
      msg.waypoint_total = self._step_total
      msg.step_index = self._step_index
      msg.step_total = self._step_total
      msg.current_step_type = self._current_step_type
      msg.fault_code = self._fault_code
      msg.battery_percent = 0.0
      msg.stamp = self._node.get_clock().now().to_msg()
    self._publish_status(msg)

  def _resolve_task_dir(self) -> str:
    configured = str(self._node.get_parameter('task_config_dir').value).strip()
    if configured:
      return configured
    try:
      share = get_package_share_directory('patrol_robot')
      return os.path.join(share, 'config')
    except Exception:
      source_fallback = Path(__file__).resolve().parents[1] / 'config'
      return str(source_fallback)

  def _declare_recovery_parameters(self) -> None:
    defaults = {
      'fault_recovery.nav_retry_max_attempts': 3,
      'fault_recovery.nav_retry_initial_wait_sec': 3.0,
      'fault_recovery.nav_retry_backoff_factor': 2.0,
      'fault_recovery.nav_timeout_sec': 120.0,
      'fault_recovery.nav_clear_costmap_on_retry': True,
      'fault_recovery.service_wait_timeout_sec': 2.0,
      'fault_recovery.capture_retry_max_attempts': 1,
      'fault_recovery.capture_retry_wait_sec': 2.0,
      'fault_recovery.capture_failure_blocks_task_default': False,
      'fault_recovery.camera_no_image_blocks_task_default': False,
      'fault_recovery.image_stale_timeout_sec': 5.0,
      'fault_recovery.tts_failure_blocks_task': False,
    }
    for key, value in defaults.items():
      if not self._node.has_parameter(key):
        self._node.declare_parameter(key, value)

  def _load_library(self) -> TaskLibrary:
    loader = TaskLoader(self._logger)
    base_dir = self._resolve_task_dir()
    library = loader.load_library(base_dir)
    self._logger.info(f'DSL 配置目录: {base_dir}')
    return library

  def _build_registry(self, library: TaskLibrary) -> SkillRegistry:
    registry = SkillRegistry(library.stations)
    registry.register('navigate', self._run_navigate_step)
    registry.register('speak', self._run_speak_step)
    registry.register('capture_image', self._run_capture_step)
    registry.register('wait', self._run_wait_step)
    registry.register('detect_anomaly', self._run_detect_step)
    registry.register('detect_workpiece', self._run_detect_workpiece_step)
    registry.register('screw_drive', self._run_screw_drive_step)
    registry.register('report', self._run_report_step)
    return registry

  def _init_robot_pose(
    self,
    x: float | None = None,
    y: float | None = None,
    yaw: float | None = None,
  ) -> None:
    init_x = x if x is not None else self._node.get_parameter('initial_pose.x').value
    init_y = y if y is not None else self._node.get_parameter('initial_pose.y').value
    init_yaw = yaw if yaw is not None else self._node.get_parameter('initial_pose.yaw').value
    initial_pose = pose_from_xyyaw(self._node, init_x, init_y, init_yaw)
    self._node.setInitialPose(initial_pose)
    self._logger.info(f'初始位姿: x={init_x}, y={init_y}, yaw={init_yaw}')

  def _on_step_status(self, ctx: ExecutionContext) -> None:
    self._step_index = ctx.step_index
    self._step_total = ctx.step_total
    self._current_step_type = ctx.current_step_type
    self._current_station = ctx.current_station or ''
    self._point_index = min(ctx.step_index, max(ctx.step_total - 1, 0))
    if self._state not in (PatrolTaskState.PAUSED, PatrolTaskState.RECOVERING):
      self._set_state(PatrolTaskState.RUNNING)

  def _run_navigate_step(self, step, ctx: ExecutionContext) -> SkillResult:
    if self._registry is None:
      return SkillResult(SkillStatus.FAILED, 'registry 未初始化')
    target = str(step.params.get('target', '')).strip()
    if not target:
      return SkillResult(SkillStatus.FAILED, 'navigate 缺少 target')
    station = self._registry.station(target)
    ctx.current_station = target
    target_pose = pose_from_xyyaw(
      self._node,
      station.x,
      station.y,
      math.radians(station.yaw_deg),
    )
    result = self._navigate.execute(target_pose=target_pose)
    return result

  def _run_speak_step(self, step, ctx: ExecutionContext) -> SkillResult:
    text = str(step.params.get('text', '')).strip()
    if not text:
      return SkillResult(SkillStatus.FAILED, 'speak 缺少 text')
    return self._speak.execute(text=text)

  def _run_capture_step(self, step, ctx: ExecutionContext) -> SkillResult:
    save_tag = str(step.params.get('save_tag', 'patrol_image')).strip()
    ctx.last_image_path = None
    result = self._capture.execute(filename_prefix=save_tag or 'patrol_image')
    if result.succeeded:
      ctx.last_image_path = result.message
    return result

  def _run_wait_step(self, step, ctx: ExecutionContext) -> SkillResult:
    try:
      seconds = float(step.params.get('seconds', 1.0))
    except (TypeError, ValueError):
      return SkillResult(SkillStatus.FAILED, 'wait seconds 非法')
    wait_ms = max(0.0, seconds)
    start = time.monotonic()
    while (time.monotonic() - start) < wait_ms:
      if self._cancel_requested:
        return SkillResult(SkillStatus.CANCELED, '任务已取消')
      while self._paused and not self._cancel_requested:
        time.sleep(0.2)
      time.sleep(0.1)
    return SkillResult(SkillStatus.SUCCEEDED, f'wait {wait_ms:.1f}s')

  def _run_detect_step(self, step, ctx: ExecutionContext) -> SkillResult:
    model = str(step.params.get('model', 'mock_detector')).strip()
    return self._detect.execute(model=model, context=ctx)

  def _run_detect_workpiece_step(self, step, ctx: ExecutionContext) -> SkillResult:
    model = str(step.params.get('model', 'mock_workpiece_detector')).strip()
    expected_state = str(
      step.params.get('expected_state', 'ready_for_screw')).strip()
    mock_state = str(step.params.get('mock_state', '')).strip()
    return self._workpiece.execute(
      model=model,
      expected_state=expected_state or 'ready_for_screw',
      mock_state=mock_state,
      context=ctx,
    )

  def _run_screw_drive_step(self, step, ctx: ExecutionContext) -> SkillResult:
    target = str(step.params.get('target', ctx.current_station or '')).strip()
    try:
      screw_count = int(step.params.get('screw_count', 4))
      torque_nm = float(step.params.get('torque_nm', 1.2))
      timeout_sec = float(step.params.get('timeout_sec', 15.0))
    except (TypeError, ValueError):
      return SkillResult(
        SkillStatus.FAILED,
        'screw_drive 参数非法',
        fault_code='STEP_EXCEPTION',
      )
    return self._screw.execute(
      target=target,
      screw_count=screw_count,
      torque_nm=torque_nm,
      timeout_sec=timeout_sec,
      context=ctx,
    )

  def _run_report_step(self, step, ctx: ExecutionContext) -> SkillResult:
    channel = str(step.params.get('channel', 'log')).strip()
    summary = bool(step.params.get('summary', False))
    return self._report.execute(channel=channel, context=ctx, summary=summary)

  def submit_task(
    self,
    task_name: str,
    task_id: str,
  ) -> tuple[bool, str]:
    if not task_name:
      return False, 'task_name 不能为空'
    with self._lock:
      self._pending_start = True
      self._pending_task_name = task_name
      self._pending_task_id = task_id
      self._cancel_requested = True
      self._orchestrator.cancel()
    return True, f'任务 {task_name} 已排队'

  def control_patrol(self, action: int) -> tuple[bool, str]:
    from patrol_interfaces.srv import ControlPatrol
    if action == ControlPatrol.Request.PAUSE:
      with self._lock:
        self._paused = True
        self._orchestrator.set_paused(True)
      self._set_state(PatrolTaskState.PAUSED)
      return True, '巡逻已暂停'
    if action == ControlPatrol.Request.RESUME:
      with self._lock:
        if not self._patrol_active:
          return False, '当前无进行中的巡逻'
        self._paused = False
        self._orchestrator.set_paused(False)
      self._set_state(PatrolTaskState.RUNNING)
      return True, '巡逻已恢复'
    if action == ControlPatrol.Request.CANCEL:
      with self._lock:
        self._cancel_requested = True
        self._patrol_active = False
        self._orchestrator.cancel()
      self._navigate.cancel()
      self._screw.cancel()
      self._fault_code = ''
      self._set_state(PatrolTaskState.IDLE)
      return True, '巡逻已取消'
    return False, f'未知 action: {action}'

  def _apply_pending_task(self) -> tuple[bool, str]:
    with self._lock:
      if not self._pending_start or not self._pending_task_name:
        return False, ''
      task_name = self._pending_task_name
      task_id = self._pending_task_id or task_name
      self._pending_start = False
      self._pending_task_name = None
      self._pending_task_id = None
      self._cancel_requested = False
      self._paused = False
      self._patrol_active = True
      self._fault_code = ''
      self._orchestrator.reset()
    self._set_state(PatrolTaskState.RUNNING)
    return True, f'{task_name}:{task_id}'

  def _patrol_loop(self) -> None:
    while rclpy.ok():
      started, task_ref = self._apply_pending_task()
      if started:
        task_name, task_id = task_ref.split(':', 1)
        if self._library is None or self._registry is None:
          self._logger.error('Task library 未初始化')
          self._set_state(PatrolTaskState.IDLE)
          continue
        task = self._library.tasks.get(task_name)
        if task is None:
          self._logger.error(f'未找到任务: {task_name}')
          self._set_state(PatrolTaskState.IDLE)
          continue
        self._current_task = task
        self._task_id = task_id
        ctx = ExecutionContext(task_id=task_id, task_name=task.name)
        ok, message = self._orchestrator.run_task(task, self._registry, ctx)
        if ok:
          self._fault_code = ''
          self._set_state(PatrolTaskState.COMPLETED)
          time.sleep(max(self._task_completion_hold_sec, 0.0))
          self._set_state(PatrolTaskState.IDLE)
          self._logger.info(f'任务完成: {task_name}')
        else:
          if '任务已取消' in message:
            self._set_state(PatrolTaskState.IDLE)
          else:
            self._fault_code = self._fault_code or 'TASK_FAILED'
            if self._state != PatrolTaskState.FAILED:
              self._set_state(PatrolTaskState.FAILED)
          self._logger.warn(message)
        with self._lock:
          self._patrol_active = False
          self._paused = False
          self._cancel_requested = False
          self._orchestrator.reset()
        continue

      with self._lock:
        active = self._patrol_active
      if not active:
        time.sleep(0.2)
        continue
      time.sleep(0.1)

  def run(self) -> None:
    self._library = self._load_library()
    self._registry = self._build_registry(self._library)
    self._init_robot_pose()
    time.sleep(1.0)
    self._node.waitUntilNav2Active()
    self._logger.info('Nav2 已激活')
    self._set_state(PatrolTaskState.IDLE)

    default_task_name = self._node.get_parameter('default_task_name').value
    if self._auto_start and default_task_name:
      with self._lock:
        self._patrol_active = True
        self._pending_start = True
        self._pending_task_name = default_task_name
        self._pending_task_id = ''
      self._logger.info(f'auto_start_task: {default_task_name}')
    else:
      self._logger.info('等待远程任务 (submit_patrol_task/start_task)')

    self._patrol_loop()
