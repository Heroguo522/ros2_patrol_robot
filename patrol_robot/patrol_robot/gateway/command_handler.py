import json
import threading

import rclpy
from patrol_interfaces.srv import ControlPatrol, SubmitPatrolTask
from rclpy.node import Node

from patrol_robot.gateway.schema import build_ack, build_event


class CommandHandler:
  VALID_ACTIONS = {
    'start_patrol', 'update_patrol', 'pause_patrol', 'resume_patrol', 'cancel_patrol',
  }

  def __init__(self, node: Node, robot_id: str, on_event=None):
    self._node = node
    self._robot_id = robot_id
    self._on_event = on_event
    self._submit_client = node.create_client(SubmitPatrolTask, 'submit_patrol_task')
    self._control_client = node.create_client(ControlPatrol, 'control_patrol')
    self._seen_commands: dict[str, str] = {}
    self._lock = threading.Lock()

  def wait_for_services(self, timeout_sec: float = 30.0) -> None:
    self._submit_client.wait_for_service(timeout_sec=timeout_sec)
    self._control_client.wait_for_service(timeout_sec=timeout_sec)
    self._node.get_logger().info('已连接 patrol 任务服务')

  def handle_payload(self, payload: str) -> str:
    try:
      data = json.loads(payload)
    except json.JSONDecodeError as e:
      return build_ack('', False, f'JSON 无效: {e}', self._robot_id)

    command_id = data.get('command_id', '')
    action = data.get('action', '')

    with self._lock:
      if command_id and command_id in self._seen_commands:
        return self._seen_commands[command_id]

    if action not in self.VALID_ACTIONS:
      ack = build_ack(command_id, False, f'未知 action: {action}', self._robot_id)
      self._cache_ack(command_id, ack)
      return ack

    if action == 'cancel_patrol':
      ok, msg = self._call_control(ControlPatrol.Request.CANCEL)
    elif action == 'pause_patrol':
      ok, msg = self._call_control(ControlPatrol.Request.PAUSE)
    elif action == 'resume_patrol':
      ok, msg = self._call_control(ControlPatrol.Request.RESUME)
    elif action in ('start_patrol', 'update_patrol'):
      if action == 'update_patrol':
        self._call_control(ControlPatrol.Request.CANCEL)
      ok, msg = self._call_submit(data)
    else:
      ok, msg = False, 'unsupported'

    ack = build_ack(command_id, ok, msg, self._robot_id)
    self._cache_ack(command_id, ack)
    if self._on_event and ok:
      self._on_event(
        build_event(
          self._robot_id, 'command_received', data.get('task_id', ''),
          f'{action} accepted: {msg}'))
    return ack

  def _cache_ack(self, command_id: str, ack: str) -> None:
    if command_id:
      with self._lock:
        self._seen_commands[command_id] = ack

  def _call_submit(self, data: dict) -> tuple[bool, str]:
    if not self._submit_client.service_is_ready():
      return False, 'submit_patrol_task 不可用'
    req = SubmitPatrolTask.Request()
    req.task_id = data.get('task_id', 'remote_task')
    req.waypoints = list(data.get('waypoints', []))
    if not req.waypoints:
      return False, 'waypoints 不能为空'
    ip = data.get('initial_pose')
    if ip is not None:
      req.use_initial_pose = True
      req.initial_pose_x = float(ip.get('x', 0.0))
      req.initial_pose_y = float(ip.get('y', 0.0))
      req.initial_pose_yaw = float(ip.get('yaw', 0.0))
    else:
      req.use_initial_pose = False
    future = self._submit_client.call_async(req)
    rclpy.spin_until_future_complete(self._node, future, timeout_sec=15.0)
    if not future.done():
      return False, 'submit_patrol_task 超时'
    res = future.result()
    return res.success, res.message

  def _call_control(self, action: int) -> tuple[bool, str]:
    if not self._control_client.service_is_ready():
      return False, 'control_patrol 不可用'
    req = ControlPatrol.Request()
    req.action = action
    future = self._control_client.call_async(req)
    rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
    if not future.done():
      return False, 'control_patrol 超时'
    res = future.result()
    return res.success, res.message
