import rclpy
from patrol_interfaces.srv import PlayAudio
from rclpy.node import Node

from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class SpeakSkill(Skill):
  SERVICE_NAME = 'play_audio_service'
  DEFAULT_TIMEOUT_SEC = 10.0

  def __init__(self, node: Node, timeout_sec: float = DEFAULT_TIMEOUT_SEC):
    super().__init__(node, 'speak')
    self._timeout_sec = timeout_sec
    self._client = node.create_client(PlayAudio, self.SERVICE_NAME)
    while not self._client.wait_for_service(timeout_sec=1.0):
      node.get_logger().info(f'等待语音服务 [{self.SERVICE_NAME}]...')
    node.get_logger().info(f'已连接语音服务 [{self.SERVICE_NAME}]')

  def execute(self, text: str = '', **kwargs) -> SkillResult:
    logger = self._node.get_logger()
    if not self._client.service_is_ready():
      return SkillResult(SkillStatus.FAILED, '语音服务不可用')

    logger.info(f"请求播放语音: '{text}'")
    request = PlayAudio.Request()
    request.text_to_speak = text
    future = self._client.call_async(request)
    rclpy.spin_until_future_complete(
      self._node, future, timeout_sec=self._timeout_sec)

    if not future.done():
      logger.error('调用语音服务超时')
      return SkillResult(SkillStatus.FAILED, '语音服务超时')

    try:
      response = future.result()
      if response.success:
        logger.info(f'语音: {response.message}')
        return SkillResult(SkillStatus.SUCCEEDED, response.message)
      logger.warn(f'语音播放失败: {response.message}')
      return SkillResult(SkillStatus.FAILED, response.message)
    except Exception as e:
      logger.error(f'调用语音服务异常: {e}')
      return SkillResult(SkillStatus.FAILED, str(e))
