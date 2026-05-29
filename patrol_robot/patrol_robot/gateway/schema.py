import json
from datetime import datetime, timezone
from typing import Any


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def dumps_compact(payload: dict[str, Any]) -> str:
  return json.dumps(payload, separators=(',', ':'), ensure_ascii=False)


def build_telemetry(
  robot_id: str,
  task_id: str,
  state: str,
  waypoint_index: int,
  waypoint_total: int,
  step_index: int,
  step_total: int,
  current_step_type: str,
  pose: dict[str, float] | None,
  battery: float,
  fault_code: str | None,
) -> str:
  waypoint_count = max(waypoint_total, 0)
  waypoint_label = (
    f'{waypoint_index + 1}/{waypoint_total}' if waypoint_count else '0/0')
  step_count = max(step_total, 0)
  step_label = f'{step_index + 1}/{step_total}' if step_count else '0/0'
  payload = {
    'robot_id': robot_id,
    'task_id': task_id,
    'state': state,
    'progress': {
      'waypoint_index': waypoint_index,
      'waypoint_total': waypoint_total,
      'waypoint_label': waypoint_label,
      'step_index': step_index,
      'step_total': step_total,
      'step_label': step_label,
      'current_step_type': current_step_type,
    },
    'pose': pose or {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'battery': round(battery, 1),
    'fault_code': fault_code if fault_code else None,
    'timestamp': utc_now_iso(),
    'source': 'edge',
  }
  return dumps_compact(payload)


def build_event(
  robot_id: str,
  event_type: str,
  task_id: str,
  message: str,
  from_state: str = '',
  to_state: str = '',
) -> str:
  payload = {
    'robot_id': robot_id,
    'event_type': event_type,
    'task_id': task_id,
    'message': message,
    'timestamp': utc_now_iso(),
  }
  if from_state:
    payload['from_state'] = from_state
  if to_state:
    payload['to_state'] = to_state
  return dumps_compact(payload)


def build_ack(
  command_id: str,
  accepted: bool,
  message: str,
  robot_id: str,
) -> str:
  return dumps_compact({
    'command_id': command_id,
    'accepted': accepted,
    'message': message,
    'robot_id': robot_id,
    'source': 'edge',
    'timestamp': utc_now_iso(),
  })


def build_online(robot_id: str, online: bool) -> str:
  return dumps_compact({
    'robot_id': robot_id,
    'online': online,
    'timestamp': utc_now_iso(),
  })


def build_task_report_event(robot_id: str, payload_json: str) -> str:
  payload = {
    'robot_id': robot_id,
    'event_type': 'task_report',
    'payload_json': payload_json,
    'timestamp': utc_now_iso(),
  }
  return dumps_compact(payload)


def build_composite_task_report_event(robot_id: str, payload_json: str) -> str:
  try:
    payload = json.loads(payload_json) if payload_json else {}
  except json.JSONDecodeError:
    payload = {'payload_json': payload_json}
  if not isinstance(payload, dict):
    payload = {'payload': payload}
  payload['robot_id'] = robot_id
  payload['event_type'] = 'composite_task_report'
  payload['timestamp'] = utc_now_iso()
  payload['source'] = 'edge'
  return dumps_compact(payload)


def build_fault_event(
  robot_id: str,
  task_id: str,
  task_name: str,
  event_type: str,
  fault_code: str,
  fault_category: str,
  severity: str,
  recovery_action: str,
  attempt: int,
  max_attempts: int,
  step_type: str,
  step_index: int,
  step_total: int,
  station: str,
  message: str,
  details_json: str,
) -> str:
  try:
    details = json.loads(details_json) if details_json else {}
  except json.JSONDecodeError:
    details = {'raw': details_json}
  payload = {
    'robot_id': robot_id,
    'task_id': task_id,
    'task_name': task_name,
    'event_type': event_type,
    'fault_code': fault_code,
    'fault_category': fault_category,
    'severity': severity,
    'recovery_action': recovery_action,
    'attempt': int(attempt),
    'max_attempts': int(max_attempts),
    'step_type': step_type,
    'step_index': int(step_index),
    'step_total': int(step_total),
    'station': station,
    'message': message,
    'details': details,
    'timestamp': utc_now_iso(),
    'source': 'edge',
  }
  return dumps_compact(payload)
