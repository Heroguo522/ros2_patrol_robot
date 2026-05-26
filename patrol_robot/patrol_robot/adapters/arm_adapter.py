from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class ArmResult:
  success: bool
  message: str = ''
  duration_sec: float = 0.0


class ArmAdapter(ABC):
  """机械臂适配层，屏蔽 mock、厂商 SDK、ROS Action 等底层差异。"""

  @abstractmethod
  def move_to_ready_pose(self, station: str) -> ArmResult:
    pass

  @abstractmethod
  def move_to_screw_pose(self, station: str, screw_index: int) -> ArmResult:
    pass

  @abstractmethod
  def retract(self) -> ArmResult:
    pass

  @abstractmethod
  def stop(self) -> None:
    pass
