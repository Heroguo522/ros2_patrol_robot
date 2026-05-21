# MQTT 演示样例（MQTTX / MQTT Explorer）

将下列 JSON **整段复制**到 MQTTX 的 Publish 面板，Topic 填：

`robots/robot_001/command`

订阅建议：`robots/robot_001/#`

| 文件 | 用途 |
|------|------|
| `start_inspection_A.json` | 启动 DSL 任务 `inspection_route_A` |
| `start_legacy_patrol.json` | 启动兼容任务 `legacy_room_patrol` |
| `pause.json` | 暂停 |
| `resume.json` | 恢复 |
| `cancel.json` | 取消 |

安装后路径：`$(ros2 pkg prefix patrol_robot)/share/patrol_robot/mqtt_demo/`
