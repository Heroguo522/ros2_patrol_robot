#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from patrol_interfaces.msg import FaultEvent, TaskReport
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from patrol_robot.gateway.command_handler import CommandHandler
from patrol_robot.gateway.mqtt_transport import MqttTransport
from patrol_robot.gateway.schema import (
  build_event,
  build_fault_event,
  build_online,
  build_task_report_event,
  build_telemetry,
)
from patrol_robot.gateway.telemetry_aggregator import TelemetryAggregator


class RobotGatewayNode(Node):
  def __init__(self):
    super().__init__('robot_gateway_node')

    self.declare_parameter('robot_id', 'robot_001')
    self.declare_parameter('mqtt_broker_host', '127.0.0.1')
    self.declare_parameter('mqtt_broker_port', 1883)
    self.declare_parameter('mqtt_username', '')
    self.declare_parameter('mqtt_password', '')
    self.declare_parameter('mqtt_topic_prefix', 'robots/robot_001')
    self.declare_parameter('mqtt_client_id', 'robot_001_gateway')
    self.declare_parameter('telemetry_rate_hz', 1.0)
    self.declare_parameter('patrol_status_timeout_sec', 5.0)
    self.declare_parameter('mock_battery_percent', 78.0)
    self.declare_parameter('battery_topic', '')
    self.declare_parameter('mqtt_telemetry_retain', True)
    self.declare_parameter('mqtt_online_retain', True)
    self.declare_parameter('enable_mqtt', True)

    self._robot_id = self.get_parameter('robot_id').value
    self._last_iot_state = ''

    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self)
    self._aggregator = TelemetryAggregator(self, self._tf_buffer)
    self.create_subscription(
      TaskReport, '/robot/task_report', self._task_report_callback, 10)
    self.create_subscription(
      FaultEvent, '/robot/fault_event', self._fault_event_callback, 10)

    self._mqtt: MqttTransport | None = None
    if self.get_parameter('enable_mqtt').value:
      self._setup_mqtt()

    rate = self.get_parameter('telemetry_rate_hz').value
    self._timer = self.create_timer(1.0 / max(rate, 0.1), self._publish_telemetry)
    self.get_logger().info(f'IoT 网关已启动 robot_id={self._robot_id}')

  def _setup_mqtt(self) -> None:
    prefix = self.get_parameter('mqtt_topic_prefix').value
    self._cmd_handler = CommandHandler(
      self, self._robot_id, on_event=self._publish_event)
    self._cmd_handler.wait_for_services(timeout_sec=60.0)

    self._mqtt = MqttTransport(
      host=self.get_parameter('mqtt_broker_host').value,
      port=self.get_parameter('mqtt_broker_port').value,
      client_id=self.get_parameter('mqtt_client_id').value,
      topic_prefix=prefix,
      username=self.get_parameter('mqtt_username').value,
      password=self.get_parameter('mqtt_password').value,
      telemetry_retain=self.get_parameter('mqtt_telemetry_retain').value,
      online_retain=self.get_parameter('mqtt_online_retain').value,
      robot_id=self._robot_id,
      on_command=self._cmd_handler.handle_payload,
      logger=self.get_logger(),
    )
    self._mqtt.connect()
    self._mqtt.publish_online(build_online(self._robot_id, True))

  def _publish_event(self, payload: str) -> None:
    if self._mqtt:
      self._mqtt.publish_event(payload)

  def _task_report_callback(self, msg: TaskReport) -> None:
    if not self._mqtt:
      return
    self._mqtt.publish_event(
      build_task_report_event(self._robot_id, msg.payload_json))

  def _fault_event_callback(self, msg: FaultEvent) -> None:
    if not self._mqtt:
      return
    self._mqtt.publish_event(
      build_fault_event(
        robot_id=self._robot_id,
        task_id=msg.task_id,
        task_name=msg.task_name,
        event_type=msg.event_type,
        fault_code=msg.fault_code,
        fault_category=msg.fault_category,
        severity=msg.severity,
        recovery_action=msg.recovery_action,
        attempt=msg.attempt,
        max_attempts=msg.max_attempts,
        step_type=msg.step_type,
        step_index=msg.step_index,
        step_total=msg.step_total,
        station=msg.station,
        message=msg.message,
        details_json=msg.details_json,
      ))

  def _publish_telemetry(self) -> None:
    snap = self._aggregator.snapshot()
    state = snap['state']
    payload = build_telemetry(
      robot_id=snap['robot_id'],
      task_id=snap['task_id'],
      state=state,
      waypoint_index=snap['waypoint_index'],
      waypoint_total=snap['waypoint_total'],
      step_index=snap['step_index'],
      step_total=snap['step_total'],
      current_step_type=snap['current_step_type'],
      pose=snap['pose'],
      battery=snap['battery'],
      fault_code=snap['fault_code'],
    )
    if self._mqtt:
      self._mqtt.publish_telemetry(payload)
      if state != self._last_iot_state:
        self._mqtt.publish_event(
          build_event(
            self._robot_id,
            'state_changed',
            snap['task_id'],
            f'{self._last_iot_state} -> {state}',
            from_state=self._last_iot_state or 'unknown',
            to_state=state,
          ))
        self._last_iot_state = state

  def destroy_node(self) -> None:
    if self._mqtt:
      try:
        self._mqtt.publish_online(build_online(self._robot_id, False))
        self._mqtt.disconnect()
      except Exception:
        pass
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = RobotGatewayNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
