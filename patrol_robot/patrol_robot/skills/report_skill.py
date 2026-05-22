import json

from patrol_interfaces.msg import TaskReport

from patrol_robot.orchestrator.execution_context import ExecutionContext
from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class ReportSkill(Skill):
  def __init__(self, node):
    super().__init__(node, 'report')
    self._report_pub = node.create_publisher(TaskReport, '/robot/task_report', 10)

  def execute(
    self,
    channel: str = 'log',
    context: ExecutionContext | None = None,
    **kwargs,
  ) -> SkillResult:
    if context is None:
      return SkillResult(SkillStatus.FAILED, '缺少 ExecutionContext', fault_code='STEP_EXCEPTION')

    try:
      anomaly = context.last_anomaly or {}
      payload = {
        'task_id': context.task_id,
        'task_name': context.task_name,
        'step_type': context.current_step_type,
        'station': context.current_station,
        'image_path': context.last_image_path,
        'anomaly': anomaly.get('is_anomaly', False),
        'anomaly_score': float(anomaly.get('score', 0.0)),
        'model': anomaly.get('model', ''),
      }
      payload_json = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))

      report_msg = TaskReport()
      report_msg.task_id = context.task_id
      report_msg.task_name = context.task_name
      report_msg.step_type = context.current_step_type
      report_msg.station = context.current_station or ''
      report_msg.image_path = context.last_image_path or ''
      report_msg.anomaly = bool(anomaly.get('is_anomaly', False))
      report_msg.anomaly_score = float(anomaly.get('score', 0.0))
      report_msg.payload_json = payload_json
      report_msg.stamp = self._node.get_clock().now().to_msg()
      self._report_pub.publish(report_msg)

      if channel == 'log':
        self._node.get_logger().info(f'task_report: {payload_json}')
      return SkillResult(SkillStatus.SUCCEEDED, payload_json)
    except Exception as e:
      return SkillResult(
        SkillStatus.FAILED,
        str(e),
        fault_code='REPORT_FAILED',
        details={'exception': str(e)},
      )
