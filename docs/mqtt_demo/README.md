# MQTT 演示样例（MQTTX / MQTT Explorer）

将下列 JSON **整段复制**到 MQTTX 的 Publish 面板，Topic 填：

`robots/robot_001/command`

订阅建议：`robots/robot_001/#`

| 文件 | 用途 |
|------|------|
| `start_inspection_A.json` | 远程启动 3 点巡逻 |
| `pause.json` | 暂停 |
| `resume.json` | 恢复 |
| `cancel.json` | 取消 |

安装后路径：`$(ros2 pkg prefix patrol_robot)/share/patrol_robot/mqtt_demo/`
