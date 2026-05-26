import os
from pathlib import Path

import yaml

from patrol_robot.orchestrator.task_definition import (
  StationDef,
  StepDef,
  TaskDef,
  TaskLibrary,
)


class TaskLoader:
  SUPPORTED_STEP_TYPES = {
    'navigate',
    'speak',
    'capture_image',
    'wait',
    'detect_anomaly',
    'detect_workpiece',
    'screw_drive',
    'report',
  }

  def __init__(self, logger):
    self._logger = logger

  def load_library(self, base_dir: str) -> TaskLibrary:
    stations = self._load_stations(Path(base_dir) / 'stations.yaml')
    tasks = self._load_tasks(Path(base_dir) / 'tasks')
    return TaskLibrary(stations=stations, tasks=tasks)

  def _load_stations(self, path: Path) -> dict[str, StationDef]:
    raw = self._load_yaml(path)
    data = raw.get('stations')
    if not isinstance(data, dict) or not data:
      raise ValueError(f'{path} 缺少 stations 映射')

    stations: dict[str, StationDef] = {}
    for name, cfg in data.items():
      if not isinstance(cfg, dict):
        raise ValueError(f'station {name} 必须是对象')
      for key in ('x', 'y', 'yaw_deg'):
        if key not in cfg:
          raise ValueError(f'station {name} 缺少字段 {key}')
      stations[name] = StationDef(
        name=name,
        x=float(cfg['x']),
        y=float(cfg['y']),
        yaw_deg=float(cfg['yaw_deg']),
      )
    self._logger.info(f'已加载站点 {len(stations)} 个')
    return stations

  def _load_tasks(self, task_dir: Path) -> dict[str, TaskDef]:
    if not task_dir.exists():
      raise ValueError(f'任务目录不存在: {task_dir}')
    task_files = sorted(task_dir.glob('*.yaml'))
    if not task_files:
      raise ValueError(f'任务目录为空: {task_dir}')

    tasks: dict[str, TaskDef] = {}
    for file_path in task_files:
      raw = self._load_yaml(file_path)
      name = str(raw.get('name', '')).strip()
      if not name:
        raise ValueError(f'{file_path} 缺少 name')
      if name in tasks:
        raise ValueError(f'重复任务名: {name}')

      steps_raw = self._expand_steps(raw, file_path)
      if not isinstance(steps_raw, list) or not steps_raw:
        raise ValueError(f'{file_path} 缺少非空 steps 列表')

      steps: list[StepDef] = []
      for idx, step_raw in enumerate(steps_raw):
        if not isinstance(step_raw, dict):
          raise ValueError(f'{file_path} step[{idx}] 必须是对象')
        step_type = str(step_raw.get('type', '')).strip()
        if step_type not in self.SUPPORTED_STEP_TYPES:
          raise ValueError(
            f'{file_path} step[{idx}] 不支持 type={step_type}')
        params = {
          k: v for k, v in step_raw.items()
          if k not in ('type', 'optional', 'required')
        }
        optional = bool(step_raw.get('optional', False))
        required = bool(step_raw.get('required', False))
        steps.append(
          StepDef(
            type=step_type,
            params=params,
            optional=optional,
            required=required,
          ))

      task_id = str(raw.get('task_id', name)).strip() or name
      on_failure = str(raw.get('on_failure', 'retry_step')).strip()
      tasks[name] = TaskDef(
        name=name,
        task_id=task_id,
        description=str(raw.get('description', '')).strip(),
        on_failure=on_failure,
        steps=steps,
      )
    self._logger.info(f'已加载任务 {len(tasks)} 个')
    return tasks

  def _expand_steps(self, raw: dict, file_path: Path) -> list:
    steps_raw = raw.get('steps')
    if not isinstance(steps_raw, list):
      return steps_raw

    station_groups = raw.get('station_groups', {})
    if station_groups and not isinstance(station_groups, dict):
      raise ValueError(f'{file_path} station_groups 必须是对象')

    expanded = []
    for idx, step_raw in enumerate(steps_raw):
      if not isinstance(step_raw, dict):
        expanded.append(step_raw)
        continue
      if step_raw.get('type') != 'station_group':
        expanded.append(step_raw)
        continue
      expanded.extend(
        self._expand_station_group_step(
          step_raw=step_raw,
          station_groups=station_groups or {},
          file_path=file_path,
          step_index=idx,
        ))
    return expanded

  def _expand_station_group_step(
    self,
    step_raw: dict,
    station_groups: dict,
    file_path: Path,
    step_index: int,
  ) -> list[dict]:
    group_name = str(step_raw.get('group', '')).strip()
    if not group_name:
      raise ValueError(f'{file_path} step[{step_index}] station_group 缺少 group')
    group_cfg = station_groups.get(group_name)
    if not isinstance(group_cfg, dict):
      raise ValueError(f'{file_path} 未找到 station_group: {group_name}')

    cfg = {**group_cfg, **{k: v for k, v in step_raw.items() if k != 'type'}}
    stations = cfg.get('stations')
    if not isinstance(stations, list) or not stations:
      raise ValueError(f'{file_path} station_group {group_name} 缺少 stations')

    expanded: list[dict] = []
    for index, station in enumerate(stations):
      station_name = str(station).strip()
      if not station_name:
        raise ValueError(
          f'{file_path} station_group {group_name} 存在空 station')
      expanded.extend(self._build_station_steps(cfg, station_name, index))
    return expanded

  def _build_station_steps(
    self,
    cfg: dict,
    station: str,
    index: int,
  ) -> list[dict]:
    station_label = str(cfg.get('station_label_template', '{station}')).format(
      station=station, index=index + 1)
    save_tag = str(cfg.get('save_tag_template', '{station}_workpiece')).format(
      station=station, index=index + 1)
    steps: list[dict] = [{'type': 'navigate', 'target': station}]

    speak_template = str(cfg.get('speak_template', '')).strip()
    if speak_template:
      steps.append({
        'type': 'speak',
        'text': speak_template.format(
          station=station,
          station_label=station_label,
          index=index + 1,
        ),
        'optional': True,
      })

    steps.extend([
      {
        'type': 'capture_image',
        'save_tag': save_tag,
      },
      {
        'type': 'detect_workpiece',
        'model': cfg.get('model', 'mock_workpiece_detector'),
        'expected_state': cfg.get('expected_state', 'ready_for_screw'),
      },
      {
        'type': 'screw_drive',
        'target': station,
        'screw_count': cfg.get('screw_count', 4),
        'torque_nm': cfg.get('torque_nm', 1.2),
        'timeout_sec': cfg.get('timeout_sec', 15.0),
      },
      {
        'type': 'report',
        'channel': cfg.get('report_channel', 'mqtt'),
      },
    ])
    return steps

  def _load_yaml(self, path: Path) -> dict:
    if not path.exists():
      raise ValueError(f'配置文件不存在: {path}')
    with path.open('r', encoding='utf-8') as f:
      data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
      raise ValueError(f'YAML 顶层必须为对象: {path}')
    return data
