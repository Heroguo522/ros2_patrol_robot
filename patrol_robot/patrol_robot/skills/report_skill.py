import json

from patrol_interfaces.msg import CompositeTaskReport, TaskReport

from patrol_robot.orchestrator.execution_context import ExecutionContext
from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class ReportSkill(Skill):
  def __init__(self, node):
    super().__init__(node, 'report')
    self._report_pub = node.create_publisher(TaskReport, '/robot/task_report', 10)
    self._composite_report_pub = node.create_publisher(
      CompositeTaskReport, '/robot/composite_task_report', 10)

  def execute(
    self,
    channel: str = 'log',
    context: ExecutionContext | None = None,
    summary: bool = False,
    **kwargs,
  ) -> SkillResult:
    if context is None:
      return SkillResult(SkillStatus.FAILED, '缺少 ExecutionContext', fault_code='STEP_EXCEPTION')

    try:
      if summary:
        return self._publish_summary(channel, context)

      anomaly = context.last_anomaly or {}
      workpiece = context.last_workpiece or context.vars.get('last_workpiece') or {}
      screw_result = (
        context.last_screw_result
        or context.vars.get('last_screw_result')
        or {}
      )
      station_result = self._station_result(workpiece, screw_result)
      payload = {
        'task_id': context.task_id,
        'task_name': context.task_name,
        'step_type': context.current_step_type,
        'station': context.current_station,
        'image_path': context.last_image_path,
        'anomaly': anomaly.get('is_anomaly', False),
        'anomaly_score': float(anomaly.get('score', 0.0)),
        'model': anomaly.get('model', ''),
        'workpiece': workpiece,
        'screw_driving': screw_result,
        'station_result': station_result,
        'station_results': context.station_results,
        'source': 'edge',
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
      if workpiece or screw_result:
        self._publish_composite_report(
          context=context,
          station_result=station_result,
          workpiece=workpiece,
          screw_result=screw_result,
          payload_json=payload_json,
        )

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

  def _publish_composite_report(
    self,
    context: ExecutionContext,
    station_result: str,
    workpiece: dict,
    screw_result: dict,
    payload_json: str,
  ) -> None:
    msg = CompositeTaskReport()
    msg.task_id = context.task_id
    msg.task_name = context.task_name
    msg.station = context.current_station or ''
    msg.station_result = station_result
    msg.image_path = context.last_image_path or ''
    msg.workpiece_json = json.dumps(
      workpiece, ensure_ascii=False, separators=(',', ':'))
    msg.screw_result_json = json.dumps(
      screw_result, ensure_ascii=False, separators=(',', ':'))
    msg.payload_json = payload_json
    msg.stamp = self._node.get_clock().now().to_msg()
    self._composite_report_pub.publish(msg)

  def _station_result(self, workpiece: object, screw_result: object) -> str:
    if isinstance(screw_result, dict) and screw_result.get('result'):
      return str(screw_result['result'])
    if isinstance(workpiece, dict) and workpiece:
      if not workpiece.get('matched', False):
        return 'skipped'
    return 'success'

  def _publish_summary(
    self,
    channel: str,
    context: ExecutionContext,
  ) -> SkillResult:
    station_results = list(context.station_results)
    summary_result = self._summary_result(station_results)
    payload = {
      'task_id': context.task_id,
      'task_name': context.task_name,
      'step_type': context.current_step_type,
      'station': '',
      'image_path': '',
      'station_result': summary_result,
      'station_results': station_results,
      'source': 'edge',
    }
    payload_json = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))

    report_msg = TaskReport()
    report_msg.task_id = context.task_id
    report_msg.task_name = context.task_name
    report_msg.step_type = context.current_step_type
    report_msg.station = ''
    report_msg.image_path = ''
    report_msg.anomaly = False
    report_msg.anomaly_score = 0.0
    report_msg.payload_json = payload_json
    report_msg.stamp = self._node.get_clock().now().to_msg()
    self._report_pub.publish(report_msg)

    msg = CompositeTaskReport()
    msg.task_id = context.task_id
    msg.task_name = context.task_name
    msg.station = ''
    msg.station_result = summary_result
    msg.image_path = ''
    msg.workpiece_json = '{}'
    msg.screw_result_json = '{}'
    msg.payload_json = payload_json
    msg.stamp = self._node.get_clock().now().to_msg()
    self._composite_report_pub.publish(msg)

    if channel == 'log':
      self._node.get_logger().info(f'composite_task_summary: {payload_json}')
    return SkillResult(SkillStatus.SUCCEEDED, payload_json)

  def _summary_result(self, station_results: list[dict]) -> str:
    if not station_results:
      return 'skipped'
    results = [str(item.get('result', 'unknown')) for item in station_results]
    if any(item in ('failed', 'partial_failed') for item in results):
      return 'partial_failed'
    if any(item == 'skipped' for item in results):
      return 'skipped'
    return 'success'
