import time

from patrol_robot.orchestrator.execution_context import ExecutionContext
from patrol_robot.orchestrator.skill_registry import SkillRegistry
from patrol_robot.orchestrator.task_definition import TaskDef
from patrol_robot.skills.base import SkillResult, SkillStatus


class TaskOrchestrator:
  def __init__(self, logger, status_callback):
    self._logger = logger
    self._status_callback = status_callback
    self._paused = False
    self._cancelled = False

  def set_paused(self, paused: bool) -> None:
    self._paused = paused

  def cancel(self) -> None:
    self._cancelled = True

  def reset(self) -> None:
    self._cancelled = False
    self._paused = False

  def run_task(
    self,
    task: TaskDef,
    registry: SkillRegistry,
    ctx: ExecutionContext,
  ) -> tuple[bool, str]:
    for idx, step in enumerate(task.steps):
      if self._cancelled:
        return False, '任务已取消'

      while self._paused and not self._cancelled:
        time.sleep(0.2)
      if self._cancelled:
        return False, '任务已取消'

      ctx.step_index = idx
      ctx.step_total = len(task.steps)
      ctx.current_step_type = step.type
      self._status_callback(ctx)

      try:
        result = registry.execute(step, ctx)
      except Exception as e:
        if step.optional:
          self._logger.warn(
            f'可选步骤 {step.type} 异常已忽略: {e}')
          continue
        return self._handle_failure(task, step.type, str(e))

      if result.status == SkillStatus.CANCELED:
        return False, result.message or '任务已取消'

      if not result.succeeded:
        if step.optional:
          self._logger.warn(
            f'可选步骤 {step.type} 失败已忽略: {result.message}')
          continue
        return self._handle_failure(task, step.type, result.message)
    return True, '任务执行完成'

  def _handle_failure(
    self,
    task: TaskDef,
    step_type: str,
    reason: str,
  ) -> tuple[bool, str]:
    mode = task.on_failure or 'retry_step'
    self._logger.warn(f'步骤 {step_type} 失败: {reason}, 策略: {mode}')
    if mode == 'skip_step' and step_type != 'navigate':
      return True, 'skip_step'
    if mode == 'abort_task':
      return False, f'{step_type} 失败，中止任务: {reason}'
    return False, f'{step_type} 失败: {reason}'
