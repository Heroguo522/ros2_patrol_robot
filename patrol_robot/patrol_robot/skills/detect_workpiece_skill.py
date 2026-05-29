import json

from patrol_robot.orchestrator.execution_context import ExecutionContext
from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class WorkpieceDetectSkill(Skill):
  READY_STATE = 'ready_for_screw'

  def __init__(
    self,
    node,
    default_state: str = READY_STATE,
    station_overrides_json: str = '',
  ):
    super().__init__(node, 'detect_workpiece')
    self._default_state = default_state or self.READY_STATE
    self._station_overrides = self._load_station_overrides(station_overrides_json)

  def execute(
    self,
    model: str = 'mock_workpiece_detector',
    expected_state: str = READY_STATE,
    context: ExecutionContext | None = None,
    mock_state: str = '',
    **kwargs,
  ) -> SkillResult:
    if context is None:
      return SkillResult(
        SkillStatus.FAILED,
        '缺少 ExecutionContext',
        fault_code='STEP_EXCEPTION',
      )

    station = context.current_station or ''
    image_path = context.last_image_path
    if not image_path:
      workpiece = self._build_result(
        station=station,
        state='unknown',
        expected_state=expected_state,
        confidence=0.0,
        model=model,
        image_path='',
        reason='image_missing',
      )
      context.last_workpiece = workpiece
      context.vars['last_workpiece'] = workpiece
      self._node.get_logger().warn(
        f'工件识别跳过: station={station}, reason=image_missing')
      return SkillResult(SkillStatus.SUCCEEDED, '工件识别跳过: 缺少图像')

    state = (
      mock_state
      or self._station_overrides.get(station)
      or self._default_state
      or self.READY_STATE
    )
    confidence = 0.96 if state == expected_state else 0.82
    workpiece = self._build_result(
      station=station,
      state=state,
      expected_state=expected_state,
      confidence=confidence,
      model=model,
      image_path=image_path,
      reason='' if state == expected_state else 'workpiece_not_ready',
    )
    context.last_workpiece = workpiece
    context.vars['last_workpiece'] = workpiece
    self._node.get_logger().info(
      f'工件识别完成: station={station}, state={state}, '
      f'expected={expected_state}, matched={workpiece["matched"]}')
    return SkillResult(SkillStatus.SUCCEEDED, json.dumps(workpiece, ensure_ascii=False))

  def _build_result(
    self,
    station: str,
    state: str,
    expected_state: str,
    confidence: float,
    model: str,
    image_path: str,
    reason: str,
  ) -> dict:
    matched = state == expected_state
    result = {
      'station': station,
      'state': state,
      'expected_state': expected_state,
      'matched': matched,
      'confidence': float(confidence),
      'model': model or 'mock_workpiece_detector',
      'image_path': image_path,
    }
    if reason:
      result['reason'] = reason
    return result

  def _load_station_overrides(self, value: str) -> dict[str, str]:
    if not value:
      return {}
    try:
      data = json.loads(value)
    except json.JSONDecodeError as e:
      self._node.get_logger().warn(f'工件 mock 配置 JSON 无效: {e}')
      return {}
    if not isinstance(data, dict):
      self._node.get_logger().warn('工件 mock 配置必须是 JSON object')
      return {}
    return {str(k): str(v) for k, v in data.items()}
