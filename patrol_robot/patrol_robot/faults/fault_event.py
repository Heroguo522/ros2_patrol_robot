import json

from patrol_interfaces.msg import FaultEvent

from patrol_robot.faults.fault_types import FaultContext, RecoveryDecision


def build_fault_event(
  robot_id: str,
  event_type: str,
  ctx: FaultContext,
  decision: RecoveryDecision,
  attempt: int = 0,
  message: str = '',
) -> FaultEvent:
  msg = FaultEvent()
  msg.robot_id = robot_id
  msg.task_id = ctx.task_id
  msg.task_name = ctx.task_name
  msg.event_type = event_type
  msg.fault_code = ctx.fault_code
  msg.fault_category = decision.category.value
  msg.severity = decision.severity.value
  msg.recovery_action = decision.action.value
  msg.attempt = max(int(attempt), 0)
  msg.max_attempts = max(int(decision.max_attempts), 0)
  msg.step_type = ctx.step_type
  msg.step_index = max(int(ctx.step_index), 0)
  msg.step_total = max(int(ctx.step_total), 0)
  msg.station = ctx.station
  msg.message = message or ctx.message
  msg.details_json = json.dumps(ctx.details, ensure_ascii=False, separators=(',', ':'))
  return msg
