import rclpy
from patrol_interfaces.srv import CaptureImage
from rclpy.node import Node

from patrol_robot.skills.base import Skill, SkillResult, SkillStatus


class CaptureImageSkill(Skill):
  SERVICE_NAME = 'capture_image_service'
  DEFAULT_TIMEOUT_SEC = 2.0
  DEFAULT_PREFIX = 'patrol_image'

  def __init__(self, node: Node, timeout_sec: float = DEFAULT_TIMEOUT_SEC):
    super().__init__(node, 'capture_image')
    if not node.has_parameter('fault_recovery.service_wait_timeout_sec'):
      node.declare_parameter('fault_recovery.service_wait_timeout_sec', timeout_sec)
    self._timeout_sec = float(node.get_parameter('fault_recovery.service_wait_timeout_sec').value)
    self._client = node.create_client(CaptureImage, self.SERVICE_NAME)

  def execute(self, filename_prefix: str = DEFAULT_PREFIX, **kwargs) -> SkillResult:
    logger = self._node.get_logger()
    if not self._client.wait_for_service(timeout_sec=self._timeout_sec):
      return SkillResult(
        SkillStatus.FAILED,
        '拍照服务不可用',
        fault_code='CAMERA_SERVICE_UNAVAILABLE',
        details={'service': self.SERVICE_NAME, 'wait_timeout_sec': self._timeout_sec},
      )

    request = CaptureImage.Request()
    request.filename_prefix = filename_prefix
    future = self._client.call_async(request)
    rclpy.spin_until_future_complete(
      self._node, future, timeout_sec=self._timeout_sec)

    if not future.done():
      logger.error('调用拍照服务超时')
      return SkillResult(
        SkillStatus.FAILED,
        '拍照服务超时',
        fault_code='CAPTURE_FAILED',
        details={'timeout_sec': self._timeout_sec},
      )

    try:
      response = future.result()
      if response.success:
        logger.info(f'拍照成功: {response.saved_path}')
        return SkillResult(SkillStatus.SUCCEEDED, response.saved_path)
      logger.warn(f'拍照失败: {response.message}')
      fault_code = self._map_fault_code(response.message)
      return SkillResult(
        SkillStatus.FAILED,
        response.message,
        fault_code=fault_code,
      )
    except Exception as e:
      logger.error(f'调用拍照服务异常: {e}')
      return SkillResult(
        SkillStatus.FAILED,
        str(e),
        fault_code='CAPTURE_FAILED',
        details={'exception': str(e)},
      )

  def _map_fault_code(self, message: str) -> str:
    if message == 'CAMERA_NO_IMAGE':
      return 'CAMERA_NO_IMAGE'
    if message == 'CAMERA_STALE_IMAGE':
      return 'CAMERA_STALE_IMAGE'
    if message == 'CAMERA_SERVICE_UNAVAILABLE':
      return 'CAMERA_SERVICE_UNAVAILABLE'
    return 'CAPTURE_FAILED'
