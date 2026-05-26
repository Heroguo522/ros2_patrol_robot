import json
from types import SimpleNamespace

from patrol_robot.faults import fault_manager as fault_manager_module
from patrol_robot.faults.fault_manager import FaultManager
from patrol_robot.faults.fault_types import RecoveryAction, RecoveryOutcome
from patrol_robot.faults.recovery_policy import RecoveryPolicy
from patrol_robot.gateway.schema import build_fault_event
from patrol_robot.skills.base import SkillResult, SkillStatus


def _policy_params() -> dict[str, object]:
  return {
    'nav_retry_max_attempts': 2,
    'nav_retry_initial_wait_sec': 0.0,
    'nav_retry_backoff_factor': 1.0,
    'nav_clear_costmap_on_retry': True,
    'capture_retry_max_attempts': 2,
    'capture_retry_wait_sec': 0.0,
    'capture_failure_blocks_task_default': False,
    'camera_no_image_blocks_task_default': False,
    'tts_failure_blocks_task': False,
  }


def test_recovery_policy_nav_retry_mapping():
  policy = RecoveryPolicy(_policy_params())
  decision = policy.resolve('NAV_TIMEOUT', 'navigate', required=False)
  assert decision.action == RecoveryAction.RETRY_STEP
  assert decision.max_attempts == 2
  assert decision.blocks_task is True


def test_recovery_policy_capture_required_blocks_task():
  policy = RecoveryPolicy(_policy_params())
  decision = policy.resolve('CAMERA_NO_IMAGE', 'capture_image', required=True)
  assert decision.action == RecoveryAction.RETRY_STEP
  assert decision.blocks_task is True


def test_recovery_policy_tts_default_continue():
  policy = RecoveryPolicy(_policy_params())
  decision = policy.resolve('TTS_TIMEOUT', 'speak', required=False)
  assert decision.action == RecoveryAction.CONTINUE
  assert decision.blocks_task is False


def test_fault_manager_abort_when_retries_exhausted(monkeypatch):
  monkeypatch.setattr(
    fault_manager_module,
    'build_fault_event',
    lambda **kwargs: SimpleNamespace(event_type=kwargs['event_type'], stamp=None),
  )

  events = []
  states = []
  fault_codes = []

  class _Logger:
    def info(self, *_args, **_kwargs):
      return None

    def warn(self, *_args, **_kwargs):
      return None

  class _Clock:
    def now(self):
      return SimpleNamespace(to_msg=lambda: object())

  class _Node:
    def get_logger(self):
      return _Logger()

    def get_clock(self):
      return _Clock()

  class _Publisher:
    def publish(self, msg):
      events.append(msg.event_type)

  class _Navigate:
    def cancel(self):
      return None

    def clear_costmaps(self):
      return None

  manager = FaultManager(
    node=_Node(),
    robot_id='robot_001',
    event_publisher=_Publisher(),
    recovery_policy=RecoveryPolicy(_policy_params()),
    navigate_skill=_Navigate(),
    set_state_callback=lambda state: states.append(state),
    set_fault_code_callback=lambda code: fault_codes.append(code),
  )
  ctx = SimpleNamespace(
    task_id='task_1',
    task_name='demo',
    current_step_type='navigate',
    step_index=0,
    step_total=3,
    current_station='station_1',
  )
  step = SimpleNamespace(type='navigate', required=False)
  result = SkillResult(SkillStatus.FAILED, '导航失败', fault_code='NAV_FAILED')
  recovery = manager.handle_skill_failure(
    ctx=ctx,
    step=step,
    result=result,
    retry_callback=lambda: SkillResult(SkillStatus.FAILED, '仍然失败', fault_code='NAV_FAILED'),
  )

  assert recovery.outcome == RecoveryOutcome.ABORT_TASK
  assert states[-1] == 'failed'
  assert 'recovery_failed' in events
  assert 'task_failed' in events
  assert fault_codes[0] == 'NAV_FAILED'


def test_gateway_fault_event_payload_fields():
  payload = json.loads(
    build_fault_event(
      robot_id='robot_001',
      task_id='inspection_A',
      task_name='inspection_route_A',
      event_type='recovery_retrying',
      fault_code='NAV_TIMEOUT',
      fault_category='navigation',
      severity='error',
      recovery_action='retry_step',
      attempt=2,
      max_attempts=3,
      step_type='navigate',
      step_index=0,
      step_total=8,
      station='station_1',
      message='导航超时',
      details_json='{"timeout_sec":120.0}',
    ))
  assert payload['robot_id'] == 'robot_001'
  assert payload['event_type'] == 'recovery_retrying'
  assert payload['fault_code'] == 'NAV_TIMEOUT'
  assert payload['details']['timeout_sec'] == 120.0
