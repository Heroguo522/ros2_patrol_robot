import threading

import paho.mqtt.client as mqtt


class MqttTransport:
  def __init__(
    self,
    host: str,
    port: int,
    client_id: str,
    topic_prefix: str,
    username: str = '',
    password: str = '',
    telemetry_retain: bool = True,
    online_retain: bool = True,
    robot_id: str = 'robot_001',
    on_command=None,
    logger=None,
  ):
    self._host = host
    self._port = port
    self._client_id = client_id
    self._prefix = topic_prefix.rstrip('/')
    self._telemetry_retain = telemetry_retain
    self._online_retain = online_retain
    self._robot_id = robot_id
    self._on_command = on_command
    self._logger = logger
    self._username = username
    self._password = password
    self._client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
    self._connected = threading.Event()

    online_topic = f'{self._prefix}/online'
    lwt_payload = f'{{"robot_id":"{robot_id}","online":false}}'
    if online_retain:
      self._client.will_set(online_topic, lwt_payload, qos=1, retain=True)

    if username:
      self._client.username_pw_set(username, password or None)

    self._client.on_connect = self._on_connect
    self._client.on_message = self._on_message
    self._client.on_disconnect = self._on_disconnect

  @property
  def topic_telemetry(self) -> str:
    return f'{self._prefix}/telemetry'

  @property
  def topic_events(self) -> str:
    return f'{self._prefix}/events'

  @property
  def topic_command(self) -> str:
    return f'{self._prefix}/command'

  @property
  def topic_command_ack(self) -> str:
    return f'{self._prefix}/command/ack'

  @property
  def topic_online(self) -> str:
    return f'{self._prefix}/online'

  def connect(self) -> None:
    self._client.connect(self._host, self._port, keepalive=60)
    self._client.loop_start()
    if not self._connected.wait(timeout=10.0):
      raise RuntimeError(f'MQTT 连接超时: {self._host}:{self._port}')

  def disconnect(self) -> None:
    self._client.loop_stop()
    self._client.disconnect()

  def publish_telemetry(self, payload: str) -> None:
    self._client.publish(
      self.topic_telemetry, payload, qos=0, retain=self._telemetry_retain)

  def publish_event(self, payload: str) -> None:
    self._client.publish(self.topic_events, payload, qos=1, retain=False)

  def publish_ack(self, payload: str) -> None:
    self._client.publish(self.topic_command_ack, payload, qos=1, retain=False)

  def publish_online(self, payload: str) -> None:
    self._client.publish(
      self.topic_online, payload, qos=1, retain=self._online_retain)

  def _on_connect(self, client, userdata, flags, rc):
    if rc != 0:
      if self._logger:
        self._logger.error(f'MQTT 连接失败 rc={rc}')
      return
    client.subscribe(self.topic_command, qos=1)
    self._connected.set()
    if self._logger:
      self._logger.info(f'MQTT 已连接, 订阅 {self.topic_command}')

  def _on_disconnect(self, client, userdata, rc):
    self._connected.clear()
    if self._logger:
      self._logger.warn(f'MQTT 断开 rc={rc}')

  def _on_message(self, client, userdata, msg):
    if self._on_command and msg.topic == self.topic_command:
      try:
        ack = self._on_command(msg.payload.decode('utf-8'))
        if ack:
          self.publish_ack(ack)
      except Exception as e:
        if self._logger:
          self._logger.error(f'处理 MQTT 命令失败: {e}')
