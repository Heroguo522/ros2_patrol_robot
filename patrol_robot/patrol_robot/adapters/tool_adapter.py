from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class ToolResult:
  success: bool
  message: str = ''
  target_torque_nm: float = 0.0
  actual_torque_nm: float = 0.0
  duration_sec: float = 0.0


class ToolAdapter(ABC):
  """末端工具适配层，第一版用于 mock 电批，后续可替换真实工具。"""

  @abstractmethod
  def enable(self) -> ToolResult:
    pass

  @abstractmethod
  def drive_screw(
    self,
    torque_nm: float,
    timeout_sec: float,
    station: str = '',
    screw_index: int = 0,
  ) -> ToolResult:
    pass

  @abstractmethod
  def disable(self) -> ToolResult:
    pass

  @abstractmethod
  def stop(self) -> None:
    pass
