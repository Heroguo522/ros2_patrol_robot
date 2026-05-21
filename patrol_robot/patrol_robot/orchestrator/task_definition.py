from dataclasses import dataclass, field


@dataclass
class StationDef:
  name: str
  x: float
  y: float
  yaw_deg: float


@dataclass
class StepDef:
  type: str
  params: dict[str, object] = field(default_factory=dict)
  optional: bool = False


@dataclass
class TaskDef:
  name: str
  task_id: str
  steps: list[StepDef]
  description: str = ''
  on_failure: str = 'retry_step'


@dataclass
class TaskLibrary:
  stations: dict[str, StationDef]
  tasks: dict[str, TaskDef]
