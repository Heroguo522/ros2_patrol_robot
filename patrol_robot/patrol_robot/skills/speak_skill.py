import rclpy
from patrol_interfaces.srv import PlayAudio
from rclpy.node import Node

from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class SpeakSkill(Skill):
  SERVICE_NAME = 'play_audio_service'
  DEFAULT_TIMEOUT_SEC = 2.0

  def __init__(self, node: Node, timeout_sec: float = DEFAULT_TIMEOUT_SEC):
    super().__init__(node, 'speak')
    if not node.has_parameter('fault_recovery.service_wait_timeout_sec'):
      node.declare_parameter('fault_recovery.service_wait_timeout_sec', timeout_sec)
    self._timeout_sec = float(node.get_parameter('fault_recovery.service_wait_timeout_sec').value)
    self._client = node.create_client(PlayAudio, self.SERVICE_NAME)

  def execute(self, text: str = '', **kwargs) -> SkillResult:
    logger = self._node.get_logger()
    if not self._client.wait_for_service(timeout_sec=self._timeout_sec):
      return SkillResult(
        SkillStatus.FAILED,
        '语音服务不可用',
        fault_code='TTS_SERVICE_UNAVAILABLE',
        details={'service': self.SERVICE_NAME, 'wait_timeout_sec': self._timeout_sec},
      )

    logger.info(f"请求播放语音: '{text}'")
    request = PlayAudio.Request()
    request.text_to_speak = text
    future = self._client.call_async(request)
    rclpy.spin_until_future_complete(
      self._node, future, timeout_sec=self._timeout_sec)

    if not future.done():
      logger.error('调用语音服务超时')
      return SkillResult(
        SkillStatus.FAILED,
        '语音服务超时',
        fault_code='TTS_TIMEOUT',
        details={'timeout_sec': self._timeout_sec},
      )

    try:
      response = future.result()
      if response.success:
        logger.info(f'语音: {response.message}')
        return SkillResult(SkillStatus.SUCCEEDED, response.message)
      logger.warn(f'语音播放失败: {response.message}')
      return SkillResult(
        SkillStatus.FAILED,
        response.message,
        fault_code='TTS_FAILED',
      )
    except Exception as e:
      logger.error(f'调用语音服务异常: {e}')
      return SkillResult(
        SkillStatus.FAILED,
        str(e),
        fault_code='TTS_FAILED',
        details={'exception': str(e)},
      )
