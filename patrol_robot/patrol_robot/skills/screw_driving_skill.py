import json
import time

from patrol_robot.adapters.arm_adapter import ArmAdapter
from patrol_robot.adapters.tool_adapter import ToolAdapter
from patrol_robot.orchestrator.execution_context import ExecutionContext
from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class ScrewDrivingSkill(Skill):
  READY_STATE = 'ready_for_screw'

  def __init__(self, node, arm: ArmAdapter, tool: ToolAdapter):
    super().__init__(node, 'screw_drive')
    self._arm = arm
    self._tool = tool
    self._cancelled = False

  def execute(
    self,
    target: str,
    screw_count: int,
    torque_nm: float,
    timeout_sec: float = 15.0,
    context: ExecutionContext | None = None,
    **kwargs,
  ) -> SkillResult:
    self._cancelled = False
    if context is None:
      return SkillResult(
        SkillStatus.FAILED,
        '缺少 ExecutionContext',
        fault_code='STEP_EXCEPTION',
      )

    station = target or context.current_station or ''
    context.current_station = station
    workpiece = context.last_workpiece or context.vars.get('last_workpiece') or {}
    if not self._is_workpiece_ready(workpiece):
      result = self._build_skipped_result(station, screw_count, torque_nm, workpiece)
      self._record_result(context, result)
      self._node.get_logger().warn(
        f'工位 {station} 工件状态不满足打螺丝条件，跳过当前工位')
      return SkillResult(SkillStatus.SUCCEEDED, json.dumps(result, ensure_ascii=False))

    start = time.monotonic()
    screw_total = max(int(screw_count), 0)
    actual_torque: list[float] = []
    failed_indices: list[int] = []
    failure_reason = ''

    ready = self._arm.move_to_ready_pose(station)
    if not ready.success:
      result = self._build_failed_result(
        station,
        screw_total,
        torque_nm,
        actual_torque,
        failed_indices,
        ready.message,
        time.monotonic() - start,
      )
      self._record_result(context, result)
      return SkillResult(SkillStatus.SUCCEEDED, json.dumps(result, ensure_ascii=False))

    enabled = self._tool.enable()
    if not enabled.success:
      result = self._build_failed_result(
        station,
        screw_total,
        torque_nm,
        actual_torque,
        failed_indices,
        enabled.message,
        time.monotonic() - start,
      )
      self._safe_retract()
      self._record_result(context, result)
      return SkillResult(SkillStatus.SUCCEEDED, json.dumps(result, ensure_ascii=False))

    for index in range(screw_total):
      if self._cancelled:
        self._tool.stop()
        self._arm.stop()
        return SkillResult(SkillStatus.CANCELED, '打螺丝任务已取消')

      move = self._arm.move_to_screw_pose(station, index)
      if not move.success:
        failed_indices.append(index)
        failure_reason = move.message
        break

      driven = self._tool.drive_screw(
        torque_nm=torque_nm,
        timeout_sec=timeout_sec,
        station=station,
        screw_index=index,
      )
      actual_torque.append(driven.actual_torque_nm)
      if not driven.success:
        failed_indices.append(index)
        failure_reason = driven.message
        break

    self._tool.disable()
    self._safe_retract()

    duration = time.monotonic() - start
    if failed_indices:
      result = self._build_failed_result(
        station,
        screw_total,
        torque_nm,
        actual_torque,
        failed_indices,
        failure_reason or 'screw_drive_failed',
        duration,
      )
    else:
      result = {
        'station': station,
        'result': 'success',
        'screw_count': screw_total,
        'success_count': screw_total,
        'failed_count': 0,
        'target_torque_nm': float(torque_nm),
        'actual_torque_nm': actual_torque,
        'duration_sec': round(duration, 3),
        'message': 'mock screw driving completed',
      }

    self._record_result(context, result)
    self._node.get_logger().info(
      f'工位 {station} 打螺丝完成: result={result["result"]}, '
      f'success={result["success_count"]}, failed={result["failed_count"]}')
    return SkillResult(SkillStatus.SUCCEEDED, json.dumps(result, ensure_ascii=False))

  def cancel(self) -> None:
    self._cancelled = True
    self._tool.stop()
    self._arm.stop()

  def _is_workpiece_ready(self, workpiece: object) -> bool:
    if not isinstance(workpiece, dict):
      return False
    return (
      workpiece.get('state') == self.READY_STATE
      and bool(workpiece.get('matched', False))
    )

  def _build_skipped_result(
    self,
    station: str,
    screw_count: int,
    torque_nm: float,
    workpiece: object,
  ) -> dict:
    reason = 'workpiece_not_ready'
    if isinstance(workpiece, dict) and workpiece.get('reason'):
      reason = str(workpiece['reason'])
    return {
      'station': station,
      'result': 'skipped',
      'reason': reason,
      'screw_count': max(int(screw_count), 0),
      'success_count': 0,
      'failed_count': 0,
      'target_torque_nm': float(torque_nm),
      'actual_torque_nm': [],
      'duration_sec': 0.0,
      'message': reason,
    }

  def _build_failed_result(
    self,
    station: str,
    screw_count: int,
    torque_nm: float,
    actual_torque: list[float],
    failed_indices: list[int],
    failure_reason: str,
    duration_sec: float,
  ) -> dict:
    success_count = len(actual_torque)
    if failed_indices and failed_indices[0] < success_count:
      success_count = failed_indices[0]
    failed_count = max(screw_count - success_count, 1 if screw_count else 0)
    result = 'partial_failed' if success_count > 0 else 'failed'
    return {
      'station': station,
      'result': result,
      'screw_count': screw_count,
      'success_count': success_count,
      'failed_count': failed_count,
      'failed_indices': failed_indices,
      'target_torque_nm': float(torque_nm),
      'actual_torque_nm': actual_torque,
      'duration_sec': round(duration_sec, 3),
      'failure_reason': failure_reason,
      'message': failure_reason,
    }

  def _record_result(self, context: ExecutionContext, result: dict) -> None:
    context.last_screw_result = result
    context.vars['last_screw_result'] = result
    context.station_results.append(result)
    context.vars['station_results'] = context.station_results

  def _safe_retract(self) -> None:
    try:
      self._arm.retract()
    except Exception as e:
      self._node.get_logger().warn(f'mock 机械臂回收异常: {e}')
