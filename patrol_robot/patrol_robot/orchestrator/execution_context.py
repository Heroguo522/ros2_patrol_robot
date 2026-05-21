from dataclasses import dataclass, field


@dataclass
class ExecutionContext:
  task_id: str
  task_name: str
  current_station: str | None = None
  step_index: int = 0
  step_total: int = 0
  current_step_type: str = ''
  last_image_path: str | None = None
  last_anomaly: dict | None = None
  vars: dict[str, object] = field(default_factory=dict)
