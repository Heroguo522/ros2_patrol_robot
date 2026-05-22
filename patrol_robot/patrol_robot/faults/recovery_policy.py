from patrol_robot.faults.fault_types import (
  FaultCategory,
  FaultCode,
  FaultSeverity,
  RecoveryAction,
  RecoveryDecision,
)


class RecoveryPolicy:
  def __init__(self, params: dict[str, object]):
    self._params = params

  @classmethod
  def from_node(cls, node) -> 'RecoveryPolicy':
    return cls({
      'nav_retry_max_attempts': int(node.get_parameter(
        'fault_recovery.nav_retry_max_attempts').value),
      'nav_retry_initial_wait_sec': float(node.get_parameter(
        'fault_recovery.nav_retry_initial_wait_sec').value),
      'nav_retry_backoff_factor': float(node.get_parameter(
        'fault_recovery.nav_retry_backoff_factor').value),
      'nav_clear_costmap_on_retry': bool(node.get_parameter(
        'fault_recovery.nav_clear_costmap_on_retry').value),
      'capture_retry_max_attempts': int(node.get_parameter(
        'fault_recovery.capture_retry_max_attempts').value),
      'capture_retry_wait_sec': float(node.get_parameter(
        'fault_recovery.capture_retry_wait_sec').value),
      'capture_failure_blocks_task_default': bool(node.get_parameter(
        'fault_recovery.capture_failure_blocks_task_default').value),
      'camera_no_image_blocks_task_default': bool(node.get_parameter(
        'fault_recovery.camera_no_image_blocks_task_default').value),
      'tts_failure_blocks_task': bool(node.get_parameter(
        'fault_recovery.tts_failure_blocks_task').value),
    })

  @property
  def params(self) -> dict[str, object]:
    return self._params

  def resolve(self, fault_code: str, step_type: str, required: bool = False) -> RecoveryDecision:
    if fault_code in (
      FaultCode.NAV_FAILED.value,
      FaultCode.NAV_TIMEOUT.value,
      FaultCode.TF_UNAVAILABLE.value,
    ):
      return RecoveryDecision(
        action=RecoveryAction.RETRY_STEP,
        max_attempts=max(int(self._params['nav_retry_max_attempts']), 1),
        wait_sec=max(float(self._params['nav_retry_initial_wait_sec']), 0.0),
        backoff_factor=max(float(self._params['nav_retry_backoff_factor']), 1.0),
        blocks_task=True,
        category=FaultCategory.NAVIGATION,
        severity=FaultSeverity.ERROR,
      )

    if fault_code in (
      FaultCode.TTS_SERVICE_UNAVAILABLE.value,
      FaultCode.TTS_TIMEOUT.value,
      FaultCode.TTS_FAILED.value,
    ):
      blocks = bool(self._params['tts_failure_blocks_task'])
      return RecoveryDecision(
        action=RecoveryAction.ABORT_TASK if blocks else RecoveryAction.CONTINUE,
        max_attempts=1,
        wait_sec=0.0,
        backoff_factor=1.0,
        blocks_task=blocks,
        category=FaultCategory.SERVICE,
        severity=FaultSeverity.WARN,
      )

    if fault_code in (
      FaultCode.CAMERA_SERVICE_UNAVAILABLE.value,
      FaultCode.CAPTURE_FAILED.value,
    ):
      blocks = required or bool(self._params['capture_failure_blocks_task_default'])
      return self._capture_decision(blocks)

    if fault_code in (
      FaultCode.CAMERA_NO_IMAGE.value,
      FaultCode.CAMERA_STALE_IMAGE.value,
    ):
      blocks = required or bool(self._params['camera_no_image_blocks_task_default'])
      return self._capture_decision(blocks)

    if fault_code == FaultCode.REPORT_FAILED.value:
      return RecoveryDecision(
        action=RecoveryAction.CONTINUE,
        max_attempts=1,
        wait_sec=0.0,
        backoff_factor=1.0,
        blocks_task=False,
        category=FaultCategory.REPORTING,
        severity=FaultSeverity.WARN,
      )

    if fault_code == FaultCode.STEP_EXCEPTION.value:
      return RecoveryDecision(
        action=RecoveryAction.ABORT_TASK,
        max_attempts=1,
        wait_sec=0.0,
        backoff_factor=1.0,
        blocks_task=True,
        category=FaultCategory.TASK,
        severity=FaultSeverity.ERROR,
      )

    if step_type == 'navigate':
      return RecoveryDecision(
        action=RecoveryAction.RETRY_STEP,
        max_attempts=max(int(self._params['nav_retry_max_attempts']), 1),
        wait_sec=max(float(self._params['nav_retry_initial_wait_sec']), 0.0),
        backoff_factor=max(float(self._params['nav_retry_backoff_factor']), 1.0),
        blocks_task=True,
        category=FaultCategory.NAVIGATION,
        severity=FaultSeverity.ERROR,
      )

    return RecoveryDecision(
      action=RecoveryAction.ABORT_TASK,
      max_attempts=1,
      wait_sec=0.0,
      backoff_factor=1.0,
      blocks_task=True,
      category=FaultCategory.SYSTEM,
      severity=FaultSeverity.FATAL,
    )

  def _capture_decision(self, blocks_task: bool) -> RecoveryDecision:
    if not blocks_task:
      return RecoveryDecision(
        action=RecoveryAction.CONTINUE,
        max_attempts=1,
        wait_sec=0.0,
        backoff_factor=1.0,
        blocks_task=False,
        category=FaultCategory.SENSOR,
        severity=FaultSeverity.WARN,
      )
    return RecoveryDecision(
      action=RecoveryAction.RETRY_STEP,
      max_attempts=max(int(self._params['capture_retry_max_attempts']), 1),
      wait_sec=max(float(self._params['capture_retry_wait_sec']), 0.0),
      backoff_factor=1.0,
      blocks_task=True,
      category=FaultCategory.SENSOR,
      severity=FaultSeverity.WARN,
    )
