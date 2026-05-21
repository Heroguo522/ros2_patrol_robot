from typing import Callable

from patrol_robot.orchestrator.execution_context import ExecutionContext
from patrol_robot.orchestrator.task_definition import StationDef, StepDef
from patrol_robot.skills.base import SkillResult


class SkillRegistry:
  def __init__(self, stations: dict[str, StationDef]):
    self._stations = stations
    self._handlers: dict[str, Callable[[StepDef, ExecutionContext], SkillResult]] = {}

  def register(
    self,
    step_type: str,
    handler: Callable[[StepDef, ExecutionContext], SkillResult],
  ) -> None:
    self._handlers[step_type] = handler

  def execute(self, step: StepDef, ctx: ExecutionContext) -> SkillResult:
    handler = self._handlers.get(step.type)
    if handler is None:
      raise ValueError(f'未注册 step type: {step.type}')
    return handler(step, ctx)

  def station(self, name: str) -> StationDef:
    if name not in self._stations:
      raise ValueError(f'未知站点: {name}')
    return self._stations[name]
