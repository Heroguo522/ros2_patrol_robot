import math

import tf_transformations
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def pose_from_xyyaw(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
  pose = PoseStamped()
  pose.header.frame_id = 'map'
  pose.header.stamp = node.get_clock().now().to_msg()
  pose.pose.position.x = x
  pose.pose.position.y = y
  pose.pose.position.z = 0.0

  quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
  pose.pose.orientation.x = quat[0]
  pose.pose.orientation.y = quat[1]
  pose.pose.orientation.z = quat[2]
  pose.pose.orientation.w = quat[3]
  return pose


def parse_patrol_points(node: Node, logger) -> list[PoseStamped]:
  logger.info('正在从参数服务器获取巡逻点...')
  point_strings = node.get_parameter('patrol_points').get_parameter_value().string_array_value

  if not point_strings:
    logger.error("参数 'patrol_points' 未设置或为空")
    return []

  poses = []
  for point_str in point_strings:
    try:
      parts = point_str.split(',')
      x = float(parts[0])
      y = float(parts[1])
      yaw_degrees = float(parts[2])
      yaw_radians = math.radians(yaw_degrees)
      poses.append(pose_from_xyyaw(node, x, y, yaw_radians))
      logger.info(f'  - 巡逻点: x={x}, y={y}, yaw={yaw_degrees}°')
    except (ValueError, IndexError) as e:
      logger.warn(
        f"无法解析巡逻点 '{point_str}'，格式应为 'x,y,yaw_degrees': {e}")

  return poses
