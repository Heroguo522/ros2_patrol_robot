from patrol_robot.orchestrator.execution_context import ExecutionContext
from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class DetectAnomalySkill(Skill):
  def __init__(self, node):
    super().__init__(node, 'detect_anomaly')

  def execute(
    self,
    model: str = 'mock_detector',
    context: ExecutionContext | None = None,
    **kwargs,
  ) -> SkillResult:
    if context is None:
      return SkillResult(SkillStatus.FAILED, '缺少 ExecutionContext', fault_code='STEP_EXCEPTION')
    if not context.last_image_path:
      return SkillResult(
        SkillStatus.FAILED,
        '缺少图像输入',
        fault_code='CAMERA_NO_IMAGE',
      )

    result = {
      'is_anomaly': False,
      'score': 0.12,
      'label': 'normal',
      'model': model or 'mock_detector',
      'image': context.last_image_path,
    }
    context.last_anomaly = result
    self._node.get_logger().info(
      f'异常检测({result["model"]})完成: anomaly={result["is_anomaly"]}, '
      f'score={result["score"]:.2f}')
    return SkillResult(SkillStatus.SUCCEEDED, 'mock detection done')
