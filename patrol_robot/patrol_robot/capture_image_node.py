#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from patrol_interfaces.srv import CaptureImage
from rclpy.node import Node
from sensor_msgs.msg import Image


class CaptureImageNode(Node):
  DEFAULT_PREFIX = 'patrol_image'

  def __init__(self):
    super().__init__('capture_image_node')

    self.declare_parameter('picture_save_dir', os.path.expanduser('~/patrol_images'))
    self.declare_parameter('image_topic', '/camera_sensor/image_raw')
    self.declare_parameter('fault_recovery.image_stale_timeout_sec', 5.0)

    self._save_dir = self.get_parameter('picture_save_dir').value
    image_topic = self.get_parameter('image_topic').value

    self._latest_image = None
    self._last_frame_time = 0.0
    self._image_lock = threading.Lock()
    self._bridge = CvBridge()

    self._image_sub = self.create_subscription(
      Image, image_topic, self._image_callback, 10)
    self._srv = self.create_service(
      CaptureImage, 'capture_image_service', self._handle_capture_request)

    self.get_logger().info(
      f'拍照服务已就绪。保存目录: {self._save_dir}, 话题: {image_topic}')

  def _image_callback(self, msg: Image):
    with self._image_lock:
      self._latest_image = msg
      self._last_frame_time = time.monotonic()

  def _handle_capture_request(self, request, response):
    prefix = request.filename_prefix.strip() or self.DEFAULT_PREFIX

    try:
      os.makedirs(self._save_dir, exist_ok=True)
    except OSError as e:
      response.success = False
      response.message = f"创建保存目录失败: {e}"
      response.saved_path = ''
      self.get_logger().error(response.message)
      return response

    with self._image_lock:
      if self._latest_image is None:
        response.success = False
        response.message = 'CAMERA_NO_IMAGE'
        response.saved_path = ''
        self.get_logger().warn(response.message)
        return response
      stale_timeout = float(self.get_parameter('fault_recovery.image_stale_timeout_sec').value)
      if stale_timeout > 0 and (time.monotonic() - self._last_frame_time) > stale_timeout:
        response.success = False
        response.message = 'CAMERA_STALE_IMAGE'
        response.saved_path = ''
        self.get_logger().warn(response.message)
        return response

      try:
        cv_image = self._bridge.imgmsg_to_cv2(self._latest_image, 'bgr8')
        timestamp = time.strftime('%Y%m%d-%H%M%S')
        filename = f'{prefix}_{timestamp}.jpg'
        full_path = os.path.join(self._save_dir, filename)
        cv2.imwrite(full_path, cv_image)

        response.success = True
        response.message = '图像保存成功'
        response.saved_path = full_path
        self.get_logger().info(f'图像已保存: {full_path}')
      except Exception as e:
        response.success = False
        response.message = f'保存图像失败: {e}'
        response.saved_path = ''
        self.get_logger().error(response.message)

    return response


def main(args=None):
  rclpy.init(args=args)
  node = CaptureImageNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()
