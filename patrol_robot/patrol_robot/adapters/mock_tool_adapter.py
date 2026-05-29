import time

from patrol_robot.adapters.tool_adapter import ToolAdapter, ToolResult


class MockScrewToolAdapter(ToolAdapter):
  def __init__(
    self,
    logger,
    tool_delay_sec: float = 0.5,
    torque_noise_nm: float = 0.05,
    fail_station: str = '',
    fail_screw_index: int = -1,
  ):
    self._logger = logger
    self._tool_delay_sec = max(float(tool_delay_sec), 0.0)
    self._torque_noise_nm = max(float(torque_noise_nm), 0.0)
    self._fail_station = fail_station
    self._fail_screw_index = int(fail_screw_index)
    self._enabled = False
    self._stopped = False

  def enable(self) -> ToolResult:
    self._logger.info('[MockTool] 电批工具使能')
    time.sleep(self._tool_delay_sec)
    if self._stopped:
      return ToolResult(False, 'mock 电批已停止')
    self._enabled = True
    return ToolResult(True, 'mock 电批已使能', duration_sec=self._tool_delay_sec)

  def drive_screw(
    self,
    torque_nm: float,
    timeout_sec: float,
    station: str = '',
    screw_index: int = 0,
  ) -> ToolResult:
    start = time.monotonic()
    if self._stopped:
      return ToolResult(False, 'mock 电批已停止', target_torque_nm=torque_nm)
    if not self._enabled:
      return ToolResult(False, 'mock 电批未使能', target_torque_nm=torque_nm)

    self._logger.info(
      f'[MockTool] 工位 {station} 第 {screw_index + 1} 颗螺丝打紧, '
      f'target_torque={torque_nm:.2f}Nm')
    time.sleep(min(self._tool_delay_sec, max(timeout_sec, 0.0)))
    duration = time.monotonic() - start
    actual = self._mock_actual_torque(torque_nm, screw_index)
    if self._should_fail(station, screw_index):
      reason = f'mock 电批打螺丝失败: station={station}, screw_index={screw_index}'
      self._logger.warn(reason)
      return ToolResult(False, reason, torque_nm, actual, duration)
    return ToolResult(True, 'mock 打螺丝成功', torque_nm, actual, duration)

  def disable(self) -> ToolResult:
    self._logger.info('[MockTool] 电批工具关闭')
    time.sleep(self._tool_delay_sec)
    self._enabled = False
    return ToolResult(True, 'mock 电批已关闭', duration_sec=self._tool_delay_sec)

  def stop(self) -> None:
    self._stopped = True
    self._enabled = False
    self._logger.warn('mock 电批已停止')

  def _mock_actual_torque(self, torque_nm: float, screw_index: int) -> float:
    pattern = (screw_index % 3) - 1
    return round(float(torque_nm) + pattern * self._torque_noise_nm, 3)

  def _should_fail(self, station: str, screw_index: int) -> bool:
    if self._fail_station and station != self._fail_station:
      return False
    if self._fail_screw_index >= 0:
      return screw_index == self._fail_screw_index
    return bool(self._fail_station and station)
