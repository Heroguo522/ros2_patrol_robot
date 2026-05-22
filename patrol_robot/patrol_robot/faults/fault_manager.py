import time
from typing import Callable

from patrol_robot.faults.fault_event import build_fault_event
from patrol_robot.faults.fault_types import (
  FaultContext,
  RecoveryAction,
  RecoveryOutcome,
  RecoveryResult,
)
from patrol_robot.faults.recovery_policy import RecoveryPolicy
from patrol_robot.skills.base import SkillResult


class FaultManager:
  def __init__(
    self,
    node,
    robot_id: str,
    event_publisher,
    recovery_policy: RecoveryPolicy,
    navigate_skill,
    set_state_callback: Callable[[str], None],
    set_fault_code_callback: Callable[[str], None],
  ):
    self._node = node
    self._robot_id = robot_id
    self._event_publisher = event_publisher
    self._policy = recovery_policy
    self._navigate_skill = navigate_skill
    self._set_state = set_state_callback
    self._set_fault_code = set_fault_code_callback
    self._logger = node.get_logger()

  def handle_skill_failure(
    self,
    ctx,
    step,
    result: SkillResult,
    retry_callback: Callable[[], SkillResult],
  ) -> RecoveryResult:
    fault_code = result.fault_code or 'STEP_EXCEPTION'
    self._set_fault_code(fault_code)
    fault_ctx = FaultContext(
      task_id=ctx.task_id,
      task_name=ctx.task_name,
      step_type=ctx.current_step_type or step.type,
      step_index=ctx.step_index,
      step_total=ctx.step_total,
      station=ctx.current_station or '',
      fault_code=fault_code,
      message=result.message or fault_code,
      details=result.details,
    )
    required = bool(getattr(step, 'required', False))
    decision = self._policy.resolve(fault_code, step.type, required)
    self._publish_lifecycle('fault_detected', fault_ctx, decision, 0, result.message)

    if decision.action == RecoveryAction.CONTINUE:
      self._publish_lifecycle(
        'recovery_succeeded',
        fault_ctx,
        decision,
        0,
        result.message or '降级继续执行',
      )
      return RecoveryResult(
        outcome=RecoveryOutcome.CONTINUE,
        attempt=0,
        max_attempts=decision.max_attempts,
        message=result.message or 'continue',
      )

    if decision.action == RecoveryAction.SKIP_STEP:
      self._publish_lifecycle(
        'recovery_succeeded',
        fault_ctx,
        decision,
        0,
        result.message or '跳过当前步骤',
      )
      return RecoveryResult(
        outcome=RecoveryOutcome.SKIP_STEP,
        attempt=0,
        max_attempts=decision.max_attempts,
        message=result.message or 'skip_step',
      )

    if decision.action == RecoveryAction.ABORT_TASK:
      self._set_state('failed')
      self._publish_lifecycle('recovery_failed', fault_ctx, decision, 0, result.message)
      self._publish_lifecycle(
        'task_failed',
        fault_ctx,
        decision,
        0,
        f'{fault_code} 导致任务失败',
      )
      return RecoveryResult(
        outcome=RecoveryOutcome.ABORT_TASK,
        attempt=0,
        max_attempts=decision.max_attempts,
        message=result.message or 'abort_task',
      )

    self._set_state('recovering')
    self._publish_lifecycle(
      'recovery_started',
      fault_ctx,
      decision,
      0,
      f'{fault_code} 进入恢复流程',
    )

    for attempt in range(1, decision.max_attempts + 1):
      self._prepare_retry(step.type, attempt)
      wait_sec = decision.wait_sec * (decision.backoff_factor ** (attempt - 1))
      if wait_sec > 0:
        time.sleep(wait_sec)
      self._publish_lifecycle(
        'recovery_retrying',
        fault_ctx,
        decision,
        attempt,
        f'第 {attempt}/{decision.max_attempts} 次重试',
      )
      retry_result = retry_callback()
      if retry_result.succeeded:
        self._set_fault_code('')
        self._set_state('running')
        self._publish_lifecycle(
          'recovery_succeeded',
          fault_ctx,
          decision,
          attempt,
          retry_result.message or '重试成功',
        )
        return RecoveryResult(
          outcome=RecoveryOutcome.RETRY_STEP,
          attempt=attempt,
          max_attempts=decision.max_attempts,
          message=retry_result.message or 'retry_step succeeded',
        )

    self._set_state('failed')
    self._publish_lifecycle(
      'recovery_failed',
      fault_ctx,
      decision,
      decision.max_attempts,
      f'重试耗尽({decision.max_attempts})',
    )
    self._publish_lifecycle(
      'task_failed',
      fault_ctx,
      decision,
      decision.max_attempts,
      f'{fault_code} 恢复失败，任务中止',
    )
    return RecoveryResult(
      outcome=RecoveryOutcome.ABORT_TASK,
      attempt=decision.max_attempts,
      max_attempts=decision.max_attempts,
      message='recovery exhausted',
    )

  def _prepare_retry(self, step_type: str, attempt: int) -> None:
    if step_type != 'navigate':
      return
    try:
      self._navigate_skill.cancel()
      if self._policy.params.get('nav_clear_costmap_on_retry', False):
        self._navigate_skill.clear_costmaps()
    except Exception as exc:
      self._logger.warn(f'导航恢复预处理失败(attempt={attempt}): {exc}')

  def _publish_lifecycle(
    self,
    event_type: str,
    ctx: FaultContext,
    decision,
    attempt: int,
    message: str,
  ) -> None:
    msg = build_fault_event(
      robot_id=self._robot_id,
      event_type=event_type,
      ctx=ctx,
      decision=decision,
      attempt=attempt,
      message=message,
    )
    msg.stamp = self._node.get_clock().now().to_msg()
    self._event_publisher.publish(msg)
