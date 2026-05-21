# 自主巡逻机器人 · 操作手册

面向第一次跑通本项目的同学：在不细读全部源码的前提下，完成 **仿真 + Nav2 + YAML 任务巡逻**，并知道出问题该查哪里。

已熟悉 ROS 2 可直接看 [快速运行](#快速运行)。任务 DSL、Skill 分层、MQTT 网关的详细设计分别见：

- [`TASK_DSL_ARCHITECTURE.md`](TASK_DSL_ARCHITECTURE.md) — `stations.yaml` / `tasks/*.yaml`
- [`TASK_SKILL_ARCHITECTURE.md`](TASK_SKILL_ARCHITECTURE.md) — TaskManager + Skill
- [`ROBOT_GATEWAY_ARCHITECTURE.md`](ROBOT_GATEWAY_ARCHITECTURE.md) — `robot_gateway_node` 与 MQTTX 演示

---

## 目录

1. [项目能做什么](#1-项目能做什么)
2. [系统架构](#2-系统架构)
3. [环境准备](#3-环境准备)
4. [快速运行](#4-快速运行)
5. [配置与自定义](#5-配置与自定义)
6. [机器人不动？排查清单](#6-机器人不动排查清单)
7. [常用调试命令](#7-常用调试命令)
8. [代码结构](#8-代码结构)
9. [MQTT 云边演示（可选）](#9-mqtt-云边演示可选)
10. [FAQ](#10-faq)

---

## 1. 项目能做什么

一条 launch 拉起整套系统后：

| 能力 | 说明 |
|------|------|
| Gazebo 仿真 | 自定义房间 + 差速底盘 + 激光 / 相机 / IMU |
| Nav2 导航 | AMCL 定位 + 全局规划 + DWB 局部跟踪 |
| YAML 任务巡逻 | `TaskOrchestrator` 按 `tasks/*.yaml` 执行 `navigate` / `speak` / `capture_image` 等步骤 |
| 到点拍照 | `capture_image_node` 订阅 `/camera_sensor/image_raw`，保存 JPG |
| 中文语音 | `audio_player_node` 通过 gTTS 在线合成（需外网） |
| IoT 网关（可选） | `robot_gateway_node` 将 `/robot/status` 上报 MQTT，并接收远程任务 |

默认自动执行 `legacy_room_patrol`；也可通过服务或 MQTT 切换为 `inspection_route_A` 等任务。

---

## 2. 系统架构

```
Gazebo ──/scan, /odom, /camera──▶ AMCL + Nav2 ◀── NavigateSkill ◀── patrol_node
                                              │         TaskManager
                                              │              │
                                              │    TaskOrchestrator + skills/*
                                              ▼              ▼
                                         /cmd_vel      PlayAudio / CaptureImage
                                                         │              │
                                              audio_player_node   capture_image_node

patrol_node ──/robot/status──▶ robot_gateway_node ──MQTT──▶ MQTTX / 云端
```

三层关系：

1. **仿真层**（`my_robot_description`）：物理、传感器、`ros2_control` 差速控制。
2. **导航层**（`robot_navigation2` + Nav2）：`NavigateToPose` Action。
3. **应用层**（`patrol_robot`）：YAML 任务编排 + 语音/拍照/检测/上报 Skill。

---

## 3. 环境准备

### 3.1 推荐环境

- **Ubuntu 22.04** + **ROS 2 Humble**（本项目未在其他发行版上验证）。

### 3.2 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup ros-humble-navigation2 \
  ros-humble-nav2-simple-commander \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher ros-humble-xacro \
  ros-humble-cv-bridge ros-humble-tf-transformations \
  python3-colcon-common-extensions
```

> `ros-humble-gazebo-ros2-control` 缺失时，Gazebo 里机器人**能显示但不会响应** `/cmd_vel`。

### 3.3 Python 依赖

```bash
pip3 install --user gTTS pygame opencv-python tf-transformations paho-mqtt PyYAML
```

> `gTTS` 首次播报需访问 Google；离线环境请在任务 YAML 中对 `speak` 设置 `optional: true`，或改用本地 TTS（改 `audio_player_node.py`）。

### 3.4 工作区与编译

将本仓库各功能包放入 `colcon` 工作区 `src`（例如 `~/my_robot_ws/src`），在工作区根目录：

```bash
source /opt/ros/humble/setup.bash
cd ~/my_robot_ws
colcon build --symlink-install
```

`--symlink-install` 下修改 Python / launch / yaml **无需重新编译**，重启 launch 即可。

---

## 4. 快速运行

每个新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/my_robot_ws/install/setup.bash   # 按你的实际工作区路径修改
```

启动：

```bash
ros2 launch patrol_robot one_in_all.launch.py
```

无 MQTT Broker 时可关闭网关：

```bash
ros2 launch patrol_robot one_in_all.launch.py enable_gateway:=false
```

**正常日志顺序（约 30~60 秒）**：

1. Gazebo、RViz2 弹出；
2. 控制器 `robot_joint_state_broadcaster`、`robot_diff_driver_controller` 变为 `active`；
3. 终端出现 `Nav2 已激活`、`auto_start_task: legacy_room_patrol`；
4. 机器人开始第一个 `navigate` 步骤。

**常见启动问题**：

| 现象 | 处理 |
|------|------|
| `Package 'patrol_robot' not found` | 漏 `source install/setup.bash` 或编译失败 |
| 冷启动很久 | 首次 Gazebo 下载模型，属正常 |
| `speak` 超时 / `Failed to connect` | gTTS 无网；对 `speak` 加 `optional: true` 或关网测试 |
| `colcon` 报 `audio_config.yaml` 不存在 | `build/patrol_robot/config/` 下有断链；删除后 `colcon build --packages-select patrol_robot` |

---

## 5. 配置与自定义

### 5.1 核心配置文件

| 文件 | 作用 |
|------|------|
| `patrol_robot/config/patrol_config.yaml` | `default_task_name`、`auto_start_task`、`initial_pose`、`navigate_retry_wait_sec` |
| `patrol_robot/config/stations.yaml` | 站点坐标（`navigate` 的 `target` 引用） |
| `patrol_robot/config/tasks/*.yaml` | 任务步骤 DSL |
| `patrol_robot/config/capture_config.yaml` | `picture_save_dir`、`image_topic` |
| `patrol_robot/config/gateway_config.yaml` | MQTT Broker、topic 前缀 |
| `robot_navigation2/config/nav2_params.yaml` | 速度、代价地图、规划器参数 |
| `robot_navigation2/maps/room.{yaml,pgm}` | 占栅地图 |

### 5.2 改巡逻路线

编辑 `stations.yaml` 与 `tasks/*.yaml`，保存后**重启 launch**（`--symlink-install` 下无需 `colcon build`）。

最小示例：

```yaml
# stations.yaml
stations:
  station_1: { x: 3.2, y: -1.0, yaw_deg: 0.0 }

# tasks/my_route.yaml
name: my_route
steps:
  - type: navigate
    target: station_1
  - type: speak
    text: "到达一号点"
    optional: true
  - type: capture_image
    save_tag: station_1
```

切换默认任务：在 `patrol_config.yaml` 中设置 `default_task_name: "my_route"`。

步骤类型、`optional`、MQTT `start_task` 等见 [`TASK_DSL_ARCHITECTURE.md`](TASK_DSL_ARCHITECTURE.md)。

### 5.3 改初始位姿

`patrol_config.yaml`：

```yaml
patrol_node:
  ros__parameters:
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0   # 弧度，需与 Gazebo 中机器人位置一致
```

### 5.4 改速度

编辑 `robot_navigation2/config/nav2_params.yaml` 中 `controller_server` → `FollowPath` 的 `max_vel_x`、`max_vel_theta` 等。修改后重启 launch。

### 5.5 改拍照目录

编辑 `capture_config.yaml`（不是 `patrol_config.yaml`）：

```yaml
capture_image_node:
  ros__parameters:
    picture_save_dir: "/tmp/patrol_pic"
    image_topic: "/camera_sensor/image_raw"
```

### 5.6 远程触发任务（ROS 服务）

```bash
ros2 service call /submit_patrol_task patrol_interfaces/srv/SubmitPatrolTask \
  "{task_name: 'inspection_route_A', task_id: 'demo_001'}"

ros2 service call /control_patrol patrol_interfaces/srv/ControlPatrol \
  "{action: 'pause'}"    # pause | resume | cancel
```

---

## 6. 机器人不动？排查清单

按顺序检查，不要跳步。

1. **是否还在启动** — 等到日志出现 `Nav2 已激活`（冷启动 30~60 秒正常）。
2. **AMCL 是否有位姿** — `ros2 topic echo /amcl_pose --once` 无输出时，在 RViz 用 **2D Pose Estimate** 点一下机器人位置。
3. **初始位姿是否在自由区** — 换地图/世界后，调整 `patrol_config.yaml` 的 `initial_pose`。
4. **`/cmd_vel` 是否进 Gazebo** — `ros2 control list_controllers` 中 `robot_diff_driver_controller` 须为 `active`；可手动验证：
   ```bash
   ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
   ```
   若这一步仍不动，问题在仿真层（检查 `gazebo-ros2-control` 安装与 URDF 插件）。
5. **`use_sim_time`** — `ros2 param get /amcl use_sim_time` 应为 `True`（本仓库 launch 默认已设）。
6. **规划失败** — 日志 `Failed to get a path` 多为目标点在障碍上，改站点坐标或地图。

> 本项目在 `TaskManager.run()` 中已先 `setInitialPose` 再 `waitUntilNav2Active()`。若使用旧 fork 顺序相反，会卡在 `Waiting for amcl_pose`。

---

## 7. 常用调试命令

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo map base_link

ros2 lifecycle get /amcl
ros2 control list_controllers

# 语音 / 拍照服务
ros2 service call /play_audio_service patrol_interfaces/srv/PlayAudio \
  "{text_to_speak: '测试语音'}"
ros2 service call /capture_image_service patrol_interfaces/srv/CaptureImage \
  "{filename_prefix: 'manual_test'}"

# 绕过 patrol，直接 Nav2 导航
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 8. 代码结构

```
ros2_patrol_robot/
├── my_robot_description/     # URDF、Gazebo 世界、控制器配置
├── robot_navigation2/        # Nav2 launch、地图、nav2_params.yaml
├── patrol_interfaces/        # PlayAudio、CaptureImage、RobotStatus、任务服务
├── patrol_robot/
│   ├── launch/
│   │   ├── one_in_all.launch.py   # 仿真 + Nav2 + 应用
│   │   └── patrol_launch.py       # patrol / audio / capture / gateway
│   ├── config/                    # patrol、stations、tasks、capture、gateway
│   └── patrol_robot/
│       ├── patrol_node.py         # 编排入口、任务服务
│       ├── task_manager.py        # 状态机、Nav2 初始化、任务循环
│       ├── orchestrator/          # TaskLoader、TaskOrchestrator、SkillRegistry
│       ├── skills/                # navigate / speak / capture / detect / report
│       ├── audio_player_node.py
│       ├── capture_image_node.py
│       └── robot_gateway_node.py  # MQTT 网关
└── robot_application/        # 独立学习示例（不参与 one_in_all）
```

阅读顺序建议：`one_in_all.launch.py` → `task_manager.py` → `orchestrator/` → `skills/`。

---

## 9. MQTT 云边演示（可选）

1. 安装并启动 Mosquitto：`sudo apt install mosquitto && sudo systemctl start mosquitto`
2. MQTTX 连接 `127.0.0.1:1883`，订阅 `robots/robot_001/#`
3. `ros2 launch patrol_robot one_in_all.launch.py`
4. 向 `robots/robot_001/command` 发布 `docs/mqtt_demo/start_inspection_A.json`
5. 观察遥测与 Gazebo 中机器人运动

Broker 配置、JSON 字段、命令表见 [`ROBOT_GATEWAY_ARCHITECTURE.md`](ROBOT_GATEWAY_ARCHITECTURE.md) 与 [`docs/mqtt_demo/README.md`](mqtt_demo/README.md)。

---

## 10. FAQ

**RViz 模型红色或缺失？**  
检查 `robot_state_publisher` 与 `Fixed Frame` 是否为 `map`。

**gTTS `Failed to connect`？**  
网络无法访问 Google。任务里对 `speak` 设 `optional: true`，或换离线 TTS。

**Gazebo 首次极慢？**  
Classic 会拉取模型；可预置 `~/.gazebo/models`。

**`controller_manager: No load_controller service`？**  
按 [3.2](#32-系统依赖) 重装 `ros2-control` 与 `gazebo-ros2-control`。

**到点转圈、晃动？**  
调 `nav2_params.yaml` 中 `FollowPath` 与 `general_goal_checker` 的容差与 `RotateToGoal` 权重。

**无 GPU 能跑吗？**  
可以，Gazebo CPU 渲染帧率较低；可降相机 `update_rate` 减轻负载。
