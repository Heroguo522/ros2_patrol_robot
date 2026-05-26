from dataclasses import dataclass
from enum import Enum


class FaultCategory(str, Enum):
  NAVIGATION = 'navigation'
  SERVICE = 'service'
  SENSOR = 'sensor'
  PERCEPTION = 'perception'
  REPORTING = 'reporting'
  TASK = 'task'
  SYSTEM = 'system'


class FaultSeverity(str, Enum):
  INFO = 'info'
  WARN = 'warn'
  ERROR = 'error'
  FATAL = 'fatal'


class FaultCode(str, Enum):
  NAV_FAILED = 'NAV_FAILED'
  NAV_TIMEOUT = 'NAV_TIMEOUT'
  TF_UNAVAILABLE = 'TF_UNAVAILABLE'
  TTS_SERVICE_UNAVAILABLE = 'TTS_SERVICE_UNAVAILABLE'
  TTS_TIMEOUT = 'TTS_TIMEOUT'
  TTS_FAILED = 'TTS_FAILED'
  CAMERA_SERVICE_UNAVAILABLE = 'CAMERA_SERVICE_UNAVAILABLE'
  CAMERA_NO_IMAGE = 'CAMERA_NO_IMAGE'
  CAMERA_STALE_IMAGE = 'CAMERA_STALE_IMAGE'
  CAPTURE_FAILED = 'CAPTURE_FAILED'
  REPORT_FAILED = 'REPORT_FAILED'
  STEP_EXCEPTION = 'STEP_EXCEPTION'
  TASK_FAILED = 'TASK_FAILED'


class RecoveryAction(str, Enum):
  CONTINUE = 'continue'
  RETRY_STEP = 'retry_step'
  SKIP_STEP = 'skip_step'
  ABORT_TASK = 'abort_task'


class RecoveryOutcome(str, Enum):
  CONTINUE = 'continue'
  RETRY_STEP = 'retry_step'
  SKIP_STEP = 'skip_step'
  ABORT_TASK = 'abort_task'


@dataclass
class FaultContext:
  task_id: str
  task_name: str
  step_type: str
  step_index: int
  step_total: int
  station: str
  fault_code: str
  message: str
  details: dict[str, object]


@dataclass
class RecoveryDecision:
  action: RecoveryAction
  max_attempts: int
  wait_sec: float
  backoff_factor: float
  blocks_task: bool
  category: FaultCategory
  severity: FaultSeverity


@dataclass
class RecoveryResult:
  outcome: RecoveryOutcome
  attempt: int
  max_attempts: int
  message: str
