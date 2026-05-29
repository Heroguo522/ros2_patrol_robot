import time

from patrol_robot.adapters.arm_adapter import ArmAdapter, ArmResult


class MockArmAdapter(ArmAdapter):
  def __init__(
    self,
    logger,
    motion_delay_sec: float = 0.5,
    fail_station: str = '',
    fail_screw_index: int = -1,
  ):
    self._logger = logger
    self._motion_delay_sec = max(float(motion_delay_sec), 0.0)
    self._fail_station = fail_station
    self._fail_screw_index = int(fail_screw_index)
    self._stopped = False

  def move_to_ready_pose(self, station: str) -> ArmResult:
    return self._simulate_motion(
      f'机械臂进入工位 {station} 预备位',
      station=station,
      screw_index=-1,
    )

  def move_to_screw_pose(self, station: str, screw_index: int) -> ArmResult:
    return self._simulate_motion(
      f'机械臂移动到工位 {station} 第 {screw_index + 1} 颗螺丝位',
      station=station,
      screw_index=screw_index,
    )

  def retract(self) -> ArmResult:
    return self._simulate_motion('机械臂回收到安全位', station='', screw_index=-1)

  def stop(self) -> None:
    self._stopped = True
    self._logger.warn('mock 机械臂已停止')

  def _simulate_motion(
    self,
    message: str,
    station: str,
    screw_index: int,
  ) -> ArmResult:
    start = time.monotonic()
    if self._stopped:
      return ArmResult(False, 'mock 机械臂已停止')
    self._logger.info(f'[MockArm] {message}')
    time.sleep(self._motion_delay_sec)
    duration = time.monotonic() - start
    if self._should_fail(station, screw_index):
      reason = f'mock 机械臂动作失败: station={station}, screw_index={screw_index}'
      self._logger.warn(reason)
      return ArmResult(False, reason, duration)
    return ArmResult(True, message, duration)

  def _should_fail(self, station: str, screw_index: int) -> bool:
    if self._fail_station and station != self._fail_station:
      return False
    if self._fail_screw_index >= 0:
      return screw_index == self._fail_screw_index
    return bool(self._fail_station and station)
