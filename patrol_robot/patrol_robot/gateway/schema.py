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
  pose: dict[str, float] | None,
  battery: float,
  fault_code: str | None,
) -> str:
  total = max(waypoint_total, 1)
  idx = min(waypoint_index, total - 1) if waypoint_total else 0
  label = f'{idx + 1}/{waypoint_total}' if waypoint_total else '0/0'
  payload = {
    'robot_id': robot_id,
    'task_id': task_id,
    'state': state,
    'progress': {
      'waypoint_index': waypoint_index,
      'waypoint_total': waypoint_total,
      'waypoint_label': label,
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
