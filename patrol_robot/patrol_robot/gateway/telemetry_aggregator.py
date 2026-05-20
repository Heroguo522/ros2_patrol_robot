import math
import threading
import time

import rclpy
import tf_transformations
from patrol_interfaces.msg import RobotStatus
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TelemetryAggregator:
  def __init__(self, node: Node, tf_buffer: Buffer):
    self._node = node
    self._tf_buffer = tf_buffer
    self._lock = threading.Lock()
    self._last_status: RobotStatus | None = None
    self._last_status_time = 0.0
    self._battery = node.get_parameter('mock_battery_percent').value
    self._status_timeout = node.get_parameter('patrol_status_timeout_sec').value
    self._robot_id = node.get_parameter('robot_id').value

    battery_topic = node.get_parameter('battery_topic').value
    if battery_topic:
      node.create_subscription(
        BatteryState, battery_topic, self._battery_callback, 10)

    node.create_subscription(RobotStatus, '/robot/status', self._status_callback, 10)

  def _battery_callback(self, msg: BatteryState) -> None:
    pct = msg.percentage
    if pct >= 0.0:
      with self._lock:
        self._battery = pct * 100.0 if pct <= 1.0 else pct

  def _status_callback(self, msg: RobotStatus) -> None:
    with self._lock:
      self._last_status = msg
      self._last_status_time = time.monotonic()

  def _lookup_pose(self) -> dict[str, float] | None:
    try:
      transform = self._tf_buffer.lookup_transform(
        'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))
      x = transform.transform.translation.x
      y = transform.transform.translation.y
      quat = [
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
      ]
      _, _, yaw = tf_transformations.euler_from_quaternion(quat)
      return {'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 2)}
    except Exception:
      return None

  def snapshot(self) -> dict:
    with self._lock:
      status = self._last_status
      last_time = self._last_status_time
      battery = self._battery

    pose = self._lookup_pose()
    stale = (time.monotonic() - last_time) > self._status_timeout

    if status is None or stale:
      return {
        'robot_id': self._robot_id,
        'task_id': '',
        'state': 'offline',
        'waypoint_index': 0,
        'waypoint_total': 0,
        'pose': pose,
        'battery': battery,
        'fault_code': 'OFFLINE' if stale and status else None,
      }

    fault = status.fault_code if status.fault_code else None
    state = status.state
    if fault and state != 'fault':
      state = 'fault'

    return {
      'robot_id': status.robot_id or self._robot_id,
      'task_id': status.task_id,
      'state': state,
      'waypoint_index': status.waypoint_index,
      'waypoint_total': status.waypoint_total,
      'pose': pose,
      'battery': battery if battery > 0 else status.battery_percent,
      'fault_code': fault,
    }
