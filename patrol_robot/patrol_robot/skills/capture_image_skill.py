import rclpy
from patrol_interfaces.srv import CaptureImage
from rclpy.node import Node

from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class CaptureImageSkill(Skill):
  SERVICE_NAME = 'capture_image_service'
  DEFAULT_TIMEOUT_SEC = 10.0
  DEFAULT_PREFIX = 'patrol_image'

  def __init__(self, node: Node, timeout_sec: float = DEFAULT_TIMEOUT_SEC):
    super().__init__(node, 'capture_image')
    self._timeout_sec = timeout_sec
    self._client = node.create_client(CaptureImage, self.SERVICE_NAME)
    while not self._client.wait_for_service(timeout_sec=1.0):
      node.get_logger().info(f'等待拍照服务 [{self.SERVICE_NAME}]...')
    node.get_logger().info(f'已连接拍照服务 [{self.SERVICE_NAME}]')

  def execute(self, filename_prefix: str = DEFAULT_PREFIX, **kwargs) -> SkillResult:
    logger = self._node.get_logger()
    if not self._client.service_is_ready():
      return SkillResult(SkillStatus.FAILED, '拍照服务不可用')

    request = CaptureImage.Request()
    request.filename_prefix = filename_prefix
    future = self._client.call_async(request)
    rclpy.spin_until_future_complete(
      self._node, future, timeout_sec=self._timeout_sec)

    if not future.done():
      logger.error('调用拍照服务超时')
      return SkillResult(SkillStatus.FAILED, '拍照服务超时')

    try:
      response = future.result()
      if response.success:
        logger.info(f'拍照成功: {response.saved_path}')
        return SkillResult(SkillStatus.SUCCEEDED, response.saved_path)
      logger.warn(f'拍照失败: {response.message}')
      return SkillResult(SkillStatus.FAILED, response.message)
    except Exception as e:
      logger.error(f'调用拍照服务异常: {e}')
      return SkillResult(SkillStatus.FAILED, str(e))
