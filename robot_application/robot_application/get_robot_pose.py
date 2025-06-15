import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion # ROS 1->2 compatible for this
import math

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        # 参数声明
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('source_frame', 'odom')

        self.target_frame_ = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame_ = self.get_parameter('source_frame').get_parameter_value().string_value

        # TF2 相关初始化
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        # 创建一个定时器，每秒调用一次回调函数
        self.timer_ = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            f"TF Listener initialized. Listening for transform from "
            f"'{self.source_frame_}' to '{self.target_frame_}'."
        )

    def timer_callback(self):

        # 获取当前时间
        now = rclpy.time.Time()
        # 查找变换
        # lookup_transform() 的参数顺序是: target_frame, source_frame, time
        # 它会返回从 source_frame 到 target_frame 的变换
        trans = self.tf_buffer_.lookup_transform(
            self.target_frame_,
            self.source_frame_,
            now,
            timeout=Duration(seconds=0.1) # 可选的超时，避免长时间阻塞
        )

        # 提取平移 (XYZ)
        translation = trans.transform.translation
        xyz = [translation.x, translation.y, translation.z]

        # 提取旋转 (四元数)
        rotation_q = trans.transform.rotation
        quaternion = [
            rotation_q.x,
            rotation_q.y,
            rotation_q.z,
            rotation_q.w
        ]

        # 将四元数转换为欧拉角 (RPY - Roll, Pitch, Yaw) 单位是弧度
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        self.get_logger().info(
            f"Transform from '{self.source_frame_}' to '{self.target_frame_}':\n"
            f"Translation (XYZ) [m]:   [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]\n"
            f"Rotation (RPY) [rad]:  [{roll:.3f}, {pitch:.3f}, {yaw:.3f}]\n"
            f"Rotation (RPY) [deg]:  [{math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TF Listener node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()