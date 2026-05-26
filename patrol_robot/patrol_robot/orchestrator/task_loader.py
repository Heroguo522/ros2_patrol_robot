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
    'navigate', 'speak', 'capture_image', 'wait', 'detect_anomaly', 'report'
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

      steps_raw = raw.get('steps')
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

  def _load_yaml(self, path: Path) -> dict:
    if not path.exists():
      raise ValueError(f'配置文件不存在: {path}')
    with path.open('r', encoding='utf-8') as f:
      data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
      raise ValueError(f'YAML 顶层必须为对象: {path}')
    return data
