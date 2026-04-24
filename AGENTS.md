# AGENTS.md

## 项目概述

基于 ROS 2 Humble 的自主巡逻机器人项目。机器人在 Gazebo 仿真环境中运行，使用 Nav2 导航栈进行自主巡逻，到达预设路径点后自动执行拍照和中文语音播报。

## 仓库结构

```
patrol_robot/          # 核心巡逻应用（ament_python）
├── patrol_robot/      #   Python 节点源码（patrol_node, audio_player_node）
├── launch/            #   启动文件（one_in_all.launch.py 等）
├── config/            #   巡逻配置（patrol_config.yaml）
└── test/              #   pytest + ament lint 测试

patrol_interfaces/     # 自定义 ROS 2 服务接口（ament_cmake + rosidl）
└── srv/               #   PlayAudio.srv

my_robot_description/  # 机器人模型与仿真世界（ament_cmake）
├── urdf/              #   URDF/Xacro 模型
├── worlds/            #   Gazebo 世界文件
├── launch/            #   仿真启动文件
└── config/            #   RViz、ros2_control 配置

robot_navigation2/     # Nav2 导航配置（ament_cmake）
├── launch/            #   导航启动文件
├── config/            #   nav2_params.yaml
└── maps/              #   地图文件

robot_application/     # 辅助 Python 工具节点（ament_python）
├── robot_application/ #   初始化位姿、导航、路径点跟随等
└── test/              #   pytest + ament lint 测试
```

## 技术栈

| 类别       | 技术                                                         |
| ---------- | ------------------------------------------------------------ |
| 中间件     | ROS 2 Humble (`rclpy`, `ament_python`, `ament_cmake`)       |
| 导航       | Nav2 (`nav2_simple_commander`, AMCL, NavFn, DWB)            |
| 仿真       | Gazebo, `ros2_control`                                       |
| 应用语言   | Python 3                                                     |
| Python 依赖 | `gTTS`, `pygame`, `opencv-python`, `tf-transformations`, `cv_bridge` |

## 构建与运行

构建工具为 **colcon**，需在 ROS 2 工作区根目录执行：

```bash
colcon build
source install/setup.bash
```

一键启动完整系统（Gazebo + Nav2 + 巡逻应用）：

```bash
ros2 launch patrol_robot one_in_all.launch.py
```

## 测试

Python 包（`patrol_robot`、`robot_application`）包含基于 **pytest** 的测试，以及 ament 内置的 lint 检查（flake8、pep257、copyright）。

运行测试：

```bash
colcon build
colcon test --packages-select patrol_robot robot_application
colcon test-result --verbose
```

CMake 包（`my_robot_description`、`robot_navigation2`、`patrol_interfaces`）声明了 lint 依赖但目前未注册自动化测试。

## 代码风格

- Python 代码遵循 **flake8** 和 **PEP 257** 规范（通过 `ament_flake8` / `ament_pep257` 在测试中检查）。
- ROS 2 包名使用 **snake_case**，节点名同样使用 snake_case（如 `patrol_node`、`audio_player_node`）。
- 日志信息和文档注释使用**中文**。
- 未配置项目级别的格式化工具（无 `.flake8`、`pyproject.toml` 格式化配置、`.pre-commit-config.yaml`）。

## 关键配置文件

- `patrol_robot/config/patrol_config.yaml` — 巡逻路径点，格式为 `"x,y,yaw_in_degrees"`
- `robot_navigation2/config/nav2_params.yaml` — Nav2 导航参数（速度、规划器、控制器等）

## CI/CD

当前仓库无 CI/CD 配置。
