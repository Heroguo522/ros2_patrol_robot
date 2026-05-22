from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto

from rclpy.node import Node


class SkillStatus(Enum):
  IDLE = auto()
  RUNNING = auto()
  SUCCEEDED = auto()
  FAILED = auto()
  CANCELED = auto()


@dataclass
class SkillResult:
  status: SkillStatus
  message: str = ''
  fault_code: str = ''
  recoverable: bool = True
  details: dict[str, object] = field(default_factory=dict)

  @property
  def succeeded(self) -> bool:
    return self.status == SkillStatus.SUCCEEDED


class Skill(ABC):
  def __init__(self, node: Node, name: str):
    self._node = node
    self._name = name

  @property
  def name(self) -> str:
    return self._name

  @abstractmethod
  def execute(self, **kwargs) -> SkillResult:
    pass

  def cancel(self) -> None:
    pass
