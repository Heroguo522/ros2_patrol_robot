# 自主巡逻机器人 · 新手操作手册

本手册面向**第一次接触 ROS 2 的同学**，目标是让你在不深入研究每一行代码的前提下，把整套仿真+导航+巡逻系统在自己的电脑上跑起来，并且看懂机器人**为什么做、做了什么、下一步去哪儿**。

> 如果你已经熟悉 ROS 2，仅想快速上手，可直接跳到 [§4 一键运行](#4-一键运行)。

---

## 目录

1. [项目能做什么](#1-项目能做什么)
2. [系统架构与数据流](#2-系统架构与数据流)
3. [环境准备](#3-环境准备)
4. [一键运行](#4-一键运行)
5. [初学者最常踩的 3 个坑](#5-初学者最常踩的-3-个坑)
6. [Gazebo / RViz 中机器人不动？逐步排查](#6-gazebo--rviz-中机器人不动逐步排查)
7. [自定义：路径点、初始位姿、速度、地图](#7-自定义路径点初始位姿速度地图)
8. [常用调试命令速查](#8-常用调试命令速查)
9. [代码结构导览（哪个文件做什么）](#9-代码结构导览哪个文件做什么)
10. [FAQ](#10-faq)

---

## 1. 项目能做什么

启动一条命令后，你会看到：

- **Gazebo** 中弹出一个房间和一台两轮差速机器人；
- **RViz2** 中加载好地图、机器人模型、激光雷达扫描和 Nav2 的代价地图；
- 机器人**自动逐个**前往 `patrol_robot/config/patrol_config.yaml` 中预设的巡逻点；
- 到达每个点后**用 gTTS 中文语音播报**「已到达目标点，准备拍照」「拍照完成」「三秒后前往下一个目标点」等；
- 同时调用相机话题保存一张**当前视野的 JPG 图片**到本地。

整个流程是 ROS 2 中**仿真 + 定位 + 导航 + 应用层**最经典的小项目骨架，特别适合用来学习 Nav2、`ros2_control`、xacro、launch、自定义 srv 等知识点。

---

## 2. 系统架构与数据流

```
┌─────────────────────┐    /robot_description     ┌────────────────────────┐
│ robot_state_pub     │──────────────────────────▶│ Gazebo (仿真物理)      │
│  (xacro→URDF)       │                            │  - ros2_control 插件   │
└─────────────────────┘                            │  - 雷达/相机/IMU 插件  │
          │                                        └────────────┬───────────┘
          │ TF (base_link → wheels …)                           │
          ▼                                                     ▼
┌─────────────────────┐    /scan /odom /clock      ┌────────────────────────┐
│ AMCL (定位)         │◀──────────────────────────│ /cmd_vel  →  diff_drive │
└──────────┬──────────┘                            │             controller │
           │ map → odom TF                          └────────────┬───────────┘
           ▼                                                     │
┌─────────────────────┐    NavigateToPose Action   ┌─────────────▼──────────┐
│ Nav2 (planner +     │◀──────────────────────────│ patrol_node            │
│ controller + BT)    │──────────────────────────▶│  - BasicNavigator      │
└─────────────────────┘    /cmd_vel               │  - 读巡逻点配置        │
                                                  │  - 调用语音服务        │
                                                  └─────────────┬──────────┘
                                                                │ srv:PlayAudio
                                                                ▼
                                                  ┌────────────────────────┐
                                                  │ audio_player_node      │
                                                  │  gTTS + pygame         │
                                                  └────────────────────────┘
```

记住三层关系即可：
- **Gazebo** 提供物理世界与传感器仿真，发布 `/scan`、`/odom`、`/camera_sensor/image_raw` 等话题；
- **Nav2** 提供「定位 + 路径规划 + 避障」，对外暴露 `NavigateToPose` 这个 Action；
- **patrol_robot** 是你的应用层：从 YAML 读巡逻点 → 一个一个调用 Nav2 → 到点后请求语音服务 + 拍照。

---

## 3. 环境准备

### 3.1 操作系统与 ROS 2

- 推荐 **Ubuntu 22.04** + **ROS 2 Humble**。Humble 是 LTS 版本，与本项目使用的 `nav2`、`gazebo_ros_pkgs`、`ros2_control` API 完全匹配。
- 如果你用 ROS 2 Iron / Jazzy，部分 launch 接口和 `nav2_simple_commander` 行为可能略有差异，本项目未做兼容测试。

### 3.2 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-nav2-simple-commander \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  ros-humble-tf-transformations \
  python3-colcon-common-extensions
```

> 上面 `ros-humble-gazebo-ros2-control` 不能漏，否则 Gazebo 启动后**机器人能显示但不会动**——因为没有把 `/cmd_vel` 接到 Gazebo 的轮子上。

### 3.3 安装 Python 依赖

```bash
pip3 install --user gTTS pygame opencv-python tf-transformations
```

> `gTTS` 在第一次播报时会**联网**调用 Google 翻译的 TTS 接口；如果你的网络无法访问 Google，可考虑改用 `pyttsx3`/`edge-tts`，相关代码在 `patrol_robot/patrol_robot/audio_player_node.py`。

### 3.4 创建工作区并克隆仓库

```bash
mkdir -p ~/my_robot_ws/src
cd ~/my_robot_ws/src
git clone <你的仓库地址> .
cd ~/my_robot_ws
```

### 3.5 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/my_robot_ws
colcon build --symlink-install
```

`--symlink-install` 的好处是改 Python / launch / yaml 文件不用重新 `colcon build`，只需重启 launch。

---

## 4. 一键运行

每开一个新终端都需要两步 source：

```bash
source /opt/ros/humble/setup.bash
source ~/my_robot_ws/install/setup.bash
```

然后启动：

```bash
ros2 launch patrol_robot one_in_all.launch.py
```

正常情况下会依次发生：

1. Gazebo 弹出房间 + 机器人；
2. ROS 2 控制器 `robot_joint_state_broadcaster` 与 `robot_diff_driver_controller` 被加载并 `active`；
3. Nav2 全栈节点启动（map_server、amcl、planner_server、controller_server、bt_navigator、behavior_server、…）；
4. RViz2 弹出，显示地图、激光、机器人模型；
5. 终端里能看到 `已连接到语音播放服务`、`Nav2 is ready for use!`、`巡逻任务开始!`；
6. 机器人开始往第一个巡逻点 `(3.2, -1.0)` 移动。

---

## 5. 初学者最常踩的 3 个坑

### 5.1 没等 Gazebo / Nav2 启动完就以为「卡死了」

整个一键启动会同时拉起几十个节点，**第一次冷启动 30~60 秒是正常的**。在你看到 `Nav2 is ready for use!` 之前，机器人不会动。

### 5.2 source 顺序错误 / 漏 source

新开终端**必须**两条都 source：
```bash
source /opt/ros/humble/setup.bash
source ~/my_robot_ws/install/setup.bash
```
否则会出现 `Package 'patrol_robot' not found` 或者 `nav2_simple_commander` 找不到。

### 5.3 用 `sudo` 启动 / 在 root 下运行

`gTTS` 会写临时文件、`pygame` 需要访问音频设备、Gazebo 需要 GUI 显示——这些都和**当前用户**绑定。请用普通用户运行，**不要** `sudo ros2 launch …`。

---

## 6. Gazebo / RViz 中机器人不动？逐步排查

> 这是这个项目最常见的问题，本节给一个**自顶向下**的检查流程。本仓库已经修复了一处导致初始位姿没设的代码 bug（详见 §6.1），如果你拉了最新代码后仍然不动，请按下面的顺序继续排查。

### 6.1 ⚠ 已知 bug：调用顺序错误（已修复）

旧版 `patrol_robot/patrol_robot/patrol_node.py` 的 `patrol_loop()` 写法是：

```python
self.waitUntilNav2Active()   # 先等 Nav2 激活
self.init_robot_pose()       # 再设初始位姿
```

但 `BasicNavigator.waitUntilNav2Active()` 在使用 AMCL 时会**反复阻塞**直到收到一次 `/amcl_pose`，而 AMCL 又必须先收到一次 `/initialpose` 才会发布 `/amcl_pose`。**于是程序会永远卡在 `Setting initial pose` / `Waiting for amcl_pose to be received` 的循环里**，机器人根本不会动。

正确顺序应当先调用 `setInitialPose(...)`，再 `waitUntilNav2Active()`：

```python
self.init_robot_pose()       # 先发布 /initialpose
self.waitUntilNav2Active()   # 再等待 amcl/bt_navigator 进入 active
```

本项目最新代码已经按官方示例修正这个顺序。

### 6.2 终端日志要看哪里

在启动 launch 的终端里搜索这几个关键字：

| 关键字 | 含义 |
| --- | --- |
| `Configuring` / `Activating` | Nav2 lifecycle 正常推进，等就行 |
| `Nav2 is ready for use!` | Nav2 全栈已激活 |
| `Setting initial pose` | `BasicNavigator` 正在向 AMCL 发初始位姿 |
| `Waiting for amcl_pose to be received` 反复刷屏 | **AMCL 没在听 / 没收到** → 看 §6.3 |
| `'NavigateToPose' action server not available` | bt_navigator 没起来 → 检查 nav2 是否报错 |
| `Failed to get a path` | 全局规划失败，目标点可能在障碍上 → 改 `patrol_config.yaml` |

### 6.3 检查 AMCL 是否拿到了初始位姿

```bash
# 看 AMCL 输出
ros2 topic echo /amcl_pose --once
# 看 map → odom 的 TF 是否建立
ros2 run tf2_ros tf2_echo map odom
```
如果两个都没有输出，说明初始位姿压根没设进 AMCL。**临时人工补一下**就能让机器人立刻动起来：

打开 RViz2，点击工具栏 **`2D Pose Estimate`**，在地图上机器人当前位置点一下，拖出朝向，松手。这时 AMCL 会立刻发布 `/amcl_pose`，被卡住的 patrol_node 也会继续往下走。

### 6.4 检查 `/cmd_vel` 是否真的接到 Gazebo 的轮子

```bash
ros2 topic info /cmd_vel
ros2 control list_controllers
```
期望看到：
- `/cmd_vel` 有一个 publisher（控制器）和一个 subscriber（gazebo_ros2_control）；
- `robot_diff_driver_controller [diff_drive_controller/DiffDriveController] active`；
- `robot_joint_state_broadcaster [joint_state_broadcaster/JointStateBroadcaster] active`。

任何一个不是 `active`，都可能是 `ros-humble-gazebo-ros2-control` 没装，或者 controller_manager 启动失败。重新安装：
```bash
sudo apt install ros-humble-gazebo-ros2-control ros-humble-ros2-controllers
```

可以**手动绕开 Nav2** 验证机器人本体能不能动：
```bash
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```
如果这一步机器人也不动，那问题就是**仿真层**而非导航层，请先解决这一步。

### 6.5 检查时间同步（`use_sim_time`）

Gazebo 启动后所有「认 Gazebo 时间」的节点都需要 `use_sim_time: true`。本项目的 launch 已经默认 `true`，但如果你**单独**起某个节点忘加这个参数，就会出现 TF 时间戳和雷达时间戳错配，Nav2 报 `extrapolation into the future`。

```bash
ros2 param get /amcl use_sim_time   # 应当是 True
```

### 6.6 检查初始位姿是否落在地图自由区

`patrol_config.yaml` 默认 `initial_pose: (0,0,0)`。如果你换了世界 / 换了地图，原点可能落在墙里，AMCL 永远收敛不了，全局规划也总是失败。修改：

```yaml
patrol_node:
  ros__parameters:
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
```

让它和你在 Gazebo 中机器人的真实初始位置对齐。

---

## 7. 自定义：路径点、初始位姿、速度、地图

### 7.1 改巡逻点

编辑 `patrol_robot/config/patrol_config.yaml`：
```yaml
patrol_points:
  - "3.2, -1.0, 0.0"   # x, y, yaw(度)
  - "4.7, -4.7, 90.0"
  - "0.0,  0.0, 180.0"
```
保存后**重新启动 launch** 即可生效（不需要 `colcon build`，因为 `--symlink-install`）。

### 7.2 改初始位姿

同一个文件中：
```yaml
initial_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0   # 弧度
```

### 7.3 改速度

编辑 `robot_navigation2/config/nav2_params.yaml`，主要看两处：
- `controller_server.FollowPath.max_vel_x` / `max_speed_xy`（DWB 局部规划器最大线速度）
- `controller_server.FollowPath.max_vel_theta`（最大角速度）

注意 `robot_diff_driver_controller` 的 `wheel_radius` 与 URDF 一致（0.032 m），如果增大速度建议同时调高 `acc_lim_x`。

### 7.4 改地图 / 世界

- Gazebo 世界：`my_robot_description/world/custom_room.world`
- 与之配套的占栅地图：`robot_navigation2/maps/room.pgm` + `room.yaml`

注意两者要**对齐**——`room.yaml` 中的 `origin` 是地图左下角在世界坐标系下的坐标。建议建图流程：
```bash
# 1. 启动仿真
ros2 launch my_robot_description gazebo_sim.launch.py
# 2. 启动 SLAM (可选: slam_toolbox)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
# 3. 在另一终端用 teleop 走一圈
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 4. 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/my_robot_ws/src/robot_navigation2/maps/room
```

### 7.5 改图片保存目录

最新代码暴露了 `picture_save_dir` 参数，默认 `~/patrol_robot_pictures/`。要改的话编辑 `patrol_config.yaml` 添加：
```yaml
patrol_node:
  ros__parameters:
    picture_save_dir: /tmp/patrol_pic
```

---

## 8. 常用调试命令速查

```bash
# 看活着的节点
ros2 node list

# 看话题、看话题里的数据
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz   /odom

# 看 TF 树
ros2 run tf2_tools view_frames     # 生成 frames.pdf
ros2 run tf2_ros  tf2_echo map base_link

# 看 Nav2 lifecycle 状态
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator

# 列控制器
ros2 control list_controllers
ros2 control list_hardware_interfaces

# 直接给机器人发速度（跳过 Nav2）
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'

# 调用语音服务测试
ros2 service call /play_audio_service patrol_interfaces/srv/PlayAudio "{text_to_speak: '你好，我是巡逻机器人'}"

# 给一个一次性导航目标（绕过 patrol_node）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 9. 代码结构导览（哪个文件做什么）

```
patrol_robot/                       # 应用层
├── launch/
│   ├── one_in_all.launch.py        # 顶层一键启动: 仿真 + 导航 + 应用
│   └── patrol_launch.py            # 仅启动 patrol_node + audio_player_node
├── config/patrol_config.yaml       # 巡逻点 / 初始位姿 / 图片保存目录
└── patrol_robot/
    ├── patrol_node.py              # 核心: BasicNavigator + 拍照 + 调语音服务
    └── audio_player_node.py        # gTTS + pygame 中文播报服务

patrol_interfaces/                  # 自定义 srv 接口
└── srv/PlayAudio.srv               # text_to_speak → success, message

my_robot_description/               # 机器人 + 仿真世界
├── urdf/
│   ├── robot.urdf.xacro            # 总装文件
│   ├── parts/                      # 底盘、轮子、相机、激光、IMU
│   └── plugins/
│       ├── gazebo_sensor_plugin.xacro       # /scan /imu /camera_sensor/image_raw
│       └── robbot_ros2_control.xacro        # gazebo_ros2_control 插件
├── config/robot_ros2_controller.yaml         # 关节状态 + 差速控制器
├── world/custom_room.world                  # Gazebo 世界
└── launch/gazebo_sim.launch.py              # 启 Gazebo + spawn_entity + 加载控制器

robot_navigation2/                  # Nav2 配置
├── launch/navigation2.launch.py    # 包装 nav2_bringup + RViz
├── config/nav2_params.yaml         # AMCL / DWB / costmap / planner 参数
└── maps/room.{yaml,pgm}            # 占栅地图

robot_application/                  # 学习用的小工具节点（与一键启动无关）
└── robot_application/
    ├── init_robot_pose.py          # 单独发一次初始位姿
    ├── nav_to_pose.py              # 单点导航示例
    ├── waypoint_follow.py          # FollowWaypoints 示例
    └── get_robot_pose.py           # TF 查询当前位姿示例
```

阅读建议：
1. 先看 `one_in_all.launch.py` —— 它把三个子 launch 串起来；
2. 再看 `gazebo_sim.launch.py` 与 URDF —— 了解仿真侧；
3. 再看 `navigation2.launch.py` 和 `nav2_params.yaml` —— 了解 Nav2 是如何被「兜底」启动的；
4. 最后看 `patrol_node.py` —— 这是应用入口，模仿它你就能写自己的 Nav2 应用。

---

## 10. FAQ

**Q1：RViz 里机器人模型显示成红色 / 没颜色 / 没模型？**
A：通常是 `robot_state_publisher` 没拿到 `robot_description`，或者 RViz 的 `Fixed Frame` 不是 `map`。前者重启 launch 即可；后者把 RViz 左上 `Fixed Frame` 改成 `map`。

**Q2：报 `gtts.tts.gTTSError: Failed to connect`？**
A：gTTS 需要联网，且会被部分网络环境屏蔽。临时关掉播报：把 `patrol_node.py` 里 `self.speach_text(...)` 注释掉；或者把 `audio_player_node.py` 换成离线 TTS。

**Q3：Gazebo 第一次启动特别慢甚至卡死？**
A：Gazebo Classic 会下载模型缓存。可在启动前预下载 / 使用本地模型库；或在 `~/.gazebo/models` 中提前放好模型。

**Q4：报 `controller_manager: No msg "load_controller" service`？**
A：缺 `ros-humble-ros2-control` / `gazebo_ros2_control`，按 §3.2 重装。

**Q5：每个目标点之间机器人晃来晃去转圈？**
A：DWB 的 `RotateToGoal.scale` 偏大，或 `xy_goal_tolerance` 太小，编辑 `nav2_params.yaml` 的 `controller_server.FollowPath` 与 `general_goal_checker` 调一下。

**Q6：能在没 GPU 的机器上跑吗？**
A：可以。Gazebo Classic CPU 渲染没问题，只是帧率较低。如果觉得太慢可以注释掉 `gazebo_sensor_plugin.xacro` 里相机部分或调低 `update_rate`。
