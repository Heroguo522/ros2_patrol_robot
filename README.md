# 基于小鱼的ROS2自主巡逻机器人

这是一个基于 ROS 2 Humble 和 Nav2 导航栈实现的自主巡逻机器人项目。该机器人能够在一键启动仿真环境、导航系统和应用程序后，自主地在预设的地图路径点之间进行巡逻。在到达每个巡逻点后，它会自动执行拍照和中文语音播报任务。

本项目旨在提供一个完整、模块化且易于扩展的 ROS 2 应用范例，涵盖了仿真、导航、节点通信、参数配置、多节点集成启动等核心概念。

> 想从 0 开始跑通这个项目? 推荐先阅读新手友好的操作手册: [`docs/MANUAL.md`](docs/MANUAL.md)。
> 手册中包含完整环境配置、一键运行、常见坑、Gazebo/RViz 中机器人不动的逐步排查方法等内容。

## 1. 项目介绍

本项目包含以下核心功能和组件：

*   **仿真环境**: 使用 Gazebo 搭建了一个包含自定义房间布局的仿真世界。
*   **机器人模型**: 包含一个两轮差速驱动的机器人模型（URDF/XACRO），并集成了 `ros2_control` 进行物理仿真。
*   **自主导航**: 利用 Nav2 导航栈实现机器人的定位（AMCL）、路径规划（NavFn）和局部路径跟踪（DWB）。
*   **核心应用 - 自主巡逻（TaskManager + Skill）**:
    *   `patrol_node`: 巡逻编排节点，内含 `TaskManager`，通过 Skill 调度导航与到点动作。
    *   `audio_player_node`: 语音播放服务节点（`PlayAudio.srv`）。
    *   `capture_image_node`: 拍照服务节点（`CaptureImage.srv`），订阅相机并保存 JPG。
*   **模块化接口**:
    *   `patrol_interfaces`: 定义 `PlayAudio.srv`、`CaptureImage.srv`，实现应用与能力节点解耦。
*   **架构文档**: [`docs/TASK_SKILL_ARCHITECTURE.md`](docs/TASK_SKILL_ARCHITECTURE.md)
*   **一键启动**: 提供了一个顶层的 `launch` 文件，能够一键启动包括 Gazebo 仿真、Nav2 导航以及所有自定义应用节点在内的完整系统。

### 技术栈

*   **ROS 版本**: ROS 2 Humble Hawksbill
*   **仿真器**: Gazebo
*   **导航**: Nav2 Stack
*   **核心语言**: Python
*   **主要依赖库**: `nav2_simple_commander`, `gTTS`, `pygame`, `opencv-python`, `tf_transformations`

## 2. 使用方法

### 2.1 安装依赖

在开始之前，请确保您已经安装了 ROS 2 Humble，并配置好了您的 `colcon` 工作区。

1.  **安装 ROS 2 核心依赖**:
    请确保您已安装 Nav2、`ros2_control`、Gazebo 等核心组件。
    ```bash
    sudo apt update
    sudo apt install ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-cv-bridge
    ```

2.  **安装 Python 依赖库**:
    本项目使用了几个 Python 库来实现特定功能，请使用 pip 进行安装。
    ```bash
    pip install gTTS pygame opencv-python tf-transformations
    ```

3.  **克隆并编译项目代码**:
    将本项目的所有功能包（`my_robot_description`, `robot_navigation2`, `patrol_robot`, `patrol_interfaces`）克隆到您的 `colcon` 工作区 `src` 目录下。

    ```bash
    # 假设你的工作区在 ~/my_robot_ws
    cd ~/my_robot_ws/src
    # git clone <your-repo-url> .  # 此处替换为你的仓库地址
    
    # 返回工作区根目录并编译
    cd ~/my_robot_ws
    colcon build
    ```

### 2.2 运行

本项目设计了高度集成的启动流程，仅需一条命令即可启动完整的巡逻仿真任务。

1.  **source 环境**:
    在每次打开新终端时，都需要 source 您的工作区环境。
    ```bash
    source ~/my_robot_ws/install/setup.bash
    ```

2.  **一键启动**:
    运行 `patrol_robot` 包中提供的顶层 `launch` 文件。
    ```bash
    ros2 launch patrol_robot one_in_all.launch.py
    ```

    这条命令将会：
    *   启动 Gazebo 仿真环境并加载机器人模型。
    *   启动完整的 Nav2 导航栈，包括 AMCL 定位、路径规划器、控制器等。
    *   启动 `patrol_node`、`audio_player_node`、`capture_image_node`。

3.  **观察机器人**:
    启动后，机器人会自动进行初始化，然后开始依次导航到 `patrol_config.yaml` 中的路径点。到达每个点后会有语音播报，照片保存在 `capture_config.yaml` 的 `picture_save_dir`（默认 `~/patrol_images`）。

> 如果启动后机器人不动, 请参阅 [`docs/MANUAL.md` 第 6 节](docs/MANUAL.md#6-gazebo--rviz-中机器人不动逐步排查) 的排查指南。

### 2.3 自定义巡逻路径和速度

*   **修改巡逻点**:
    编辑 `patrol_robot/config/patrol_config.yaml` 文件，修改 `patrol_points` 列表即可自定义巡逻路径。格式为 `"x,y,yaw_in_degrees"`。

*   **调整机器人速度**:
    编辑 `robot_navigation2/config/nav2_params.yaml` 文件。您可以修改 `controller_server` 下 `FollowPath` 部分的 `max_speed_xy` 和 `velocity_smoother` 下的 `max_velocity` 参数来调整机器人的最大行驶速度。修改后需要重启 `launch` 文件才能生效。

## 3. 作者

nanimi, 一个不知名的ROS2学习者