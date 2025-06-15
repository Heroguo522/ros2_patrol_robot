import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import tf_transformations
import math
import time
import threading
import os 

# tf查询位姿
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration

# 用于ROS图像消息
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 语音服务接口
from patrol_interfaces.srv import PlayAudio


class PatrolNode(BasicNavigator):
    """
    一个控制机器人自主巡逻的节点
    它从配置文件中读取巡逻点, 并依次导航到每个点
    到达每个点后, 它会记录一张图像并（通过服务）请求播放一段语音
    """
    def __init__(self):
        # 调用父类构造函数, 节点名为 'patrol_node'
        super().__init__('patrol_node')

        # --- tf 初始化 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # --- 成员变量初始化 ---
        self.latest_image = None
        self.image_lock = threading.Lock() # 线程锁, 用于安全地访问 latest_image
        self.bridge = CvBridge()
        
        # --- ROS2 通信设置 ---
        # 1. 图像订阅者
        self.image_subscriber = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)
        
        # 2. 语音服务客户端
        self.speech_client = self.create_client(PlayAudio, 'play_audio_service')
        while not self.speech_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音服务 [play_audio_service] 不可用, 正在等待...')
        self.get_logger().info("已连接到语音播放服务")

        # --- 参数声明 ---
        # 声明用于机器人初始化的位姿参数
        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.yaw', 0.0)

        # 声明巡逻点列表参数 (字符串数组格式)
        self.declare_parameter('patrol_points', rclpy.Parameter.Type.STRING_ARRAY)
        
        self.get_logger().info("巡逻节点已成功初始化.")

    def image_callback(self, msg: Image):
        """
        将最新的图像消息放到 self.latest_image 中
        使用线程锁确保数据同步安全
        """
        with self.image_lock:
            self.get_logger().debug('接收到新的图像帧')
            self.latest_image = msg

    def record_image(self):
        """
        记录当前最新的图像到指定的项目目录中
        """
        # --- 1. 定义并检查目标目录 ---
        save_dir = '/home/nanimi/my_robot_ws/src/patrol_robot/picture'
        
        # 检查目录是否存在, 如果不存在则创建它
        # exist_ok=True 表示如果目录已存在, 不要抛出错误
        try:
            os.makedirs(save_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().error(f"创建图片保存目录 '{save_dir}' 失败: {e}")
            return # 如果目录创建失败, 则无法继续, 直接返回

        # --- 2. 获取锁并处理图像 ---
        with self.image_lock:
            if self.latest_image is None:
                self.get_logger().warn('无法记录图像：尚未接收到任何图像帧')
                return

            try:
                self.get_logger().info('正在将ROS图像消息转换为OpenCV格式...')
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
                
                # --- 3. 生成完整的文件路径 ---
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename_only = f"patrol_image_{timestamp}.jpg"
                
                # 使用 os.path.join 来正确地拼接目录和文件名, 这能跨平台工作
                full_path = os.path.join(save_dir, filename_only)
                
                # --- 4. 保存图像到完整路径 ---
                cv2.imwrite(full_path, cv_image)
                
                self.get_logger().info(f'图像已成功保存到: {full_path}')
                
            except Exception as e:
                self.get_logger().error(f'记录图像时发生严重错误: {e}')


    # 在 PatrolNode 类中
    def speach_text(self, text: str):
        """
        调用语音服务来播放指定的文本
        这是一个同步调用, 会等待服务完成
        """
        if not self.speech_client.service_is_ready():
            self.get_logger().warn("语音服务不可用, 无法播放")
            return

        self.get_logger().info(f"请求播放语音: '{text}'")
        request = PlayAudio.Request()
        request.text_to_speak = text
        
        # 异步发送请求
        future = self.speech_client.call_async(request)
        
        # 使用 spin_until_future_complete 等待服务调用完成
        # 这会阻塞当前函数, 直到收到响应或超时
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) # 10秒超时
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"语音服务报告: {response.message}")
                else:
                    self.get_logger().warn(f"语音服务报告播放失败: {response.message}")
            except Exception as e:
                self.get_logger().error(f'调用语音服务时发生异常: {e}')
        else:
            self.get_logger().error('调用语音服务超时')


    def get_pose_by_xyyaw(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        通过 x, y, yaw (欧拉角, 弧度) 合成一个 PoseStamped 消息
        frame_id 硬编码为 'map', 因为导航目标通常在 map 坐标系中
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0 # 2D导航, z为0

        # 从欧拉角 (yaw) 计算四元数
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose


    def init_robot_pose(self):
        """
        从ROS参数服务器获取初始位姿, 并使用 BasicNavigator 的 setInitialPose 进行设置
        """
        self.get_logger().info("正在初始化机器人位姿...")
        init_x = self.get_parameter('initial_pose.x').get_parameter_value().double_value
        init_y = self.get_parameter('initial_pose.y').get_parameter_value().double_value
        init_yaw = self.get_parameter('initial_pose.yaw').get_parameter_value().double_value
        
        initial_pose = self.get_pose_by_xyyaw(init_x, init_y, init_yaw)
        self.setInitialPose(initial_pose)
        self.get_logger().info(f"机器人初始位姿已设置为: x={init_x}, y={init_y}, yaw={init_yaw}")


    def get_target_points(self) -> list:
        """
        从ROS参数服务器获取目标点集合
        参数格式是一个字符串数组, 每个字符串格式为 "x,y,yaw"
        """
        self.get_logger().info("正在从参数服务器获取巡逻点...")
        point_strings = self.get_parameter('patrol_points').get_parameter_value().string_array_value
        
        if not point_strings:
            self.get_logger().error("参数 'patrol_points' 未设置或为空！无法执行巡逻")
            return []

        target_poses = []
        for point_str in point_strings:
            try:
                parts = point_str.split(',')
                x = float(parts[0])
                y = float(parts[1])
                # 将角度从度转换为弧度
                yaw_degrees = float(parts[2])
                yaw_radians = math.radians(yaw_degrees)
                
                pose = self.get_pose_by_xyyaw(x, y, yaw_radians)
                target_poses.append(pose)
                self.get_logger().info(f"  - 已添加巡逻点: x={x}, y={y}, yaw={yaw_degrees}°")
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f"无法解析巡逻点字符串 '{point_str}'格式应为 'x,y,yaw_degrees'错误: {e}")
        
        return target_poses

      
    def nav_to_pose(self, target_pose: PoseStamped):
        """
        使用 BasicNavigator 的 goToPose 方法导航到指定的位姿
        这是一个阻塞式调用, 会等待导航任务完成, 并提供详细的日志反馈
        """
        self.get_logger().info(f"开始导航到目标点: (x={target_pose.pose.position.x:.2f}, y={target_pose.pose.position.y:.2f})")
        self.goToPose(target_pose)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            current_pose = self.get_current_pose() 

            if feedback and current_pose:
                remaining_seconds = feedback.estimated_time_remaining.sec
                pos_x = current_pose.pose.position.x
                pos_y = current_pose.pose.position.y
                distance_remaining = feedback.distance_remaining

                self.get_logger().info(
                    f'导航中... '
                    f'当前位置: ({pos_x:.2f}, {pos_y:.2f}) | '
                    f'距离目标: {distance_remaining:.2f} 米 | '
                    f'预计剩余时间: {remaining_seconds} 秒'
                )
            
            time.sleep(1)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航成功!')
            return True  
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航任务被取消!')
            return False 
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航任务失败!')
            return False 
        else:
            self.get_logger().error('无效的导航任务结果!')
            return False
        

    def get_current_pose(self) -> PoseStamped | None:
        """
        通过查询TF树来获取机器人当前的位姿
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            # 将获取到的变换转换成 PoseStamped 消息
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except Exception as e:
            self.get_logger().warn(f"无法获取当前机器人位姿: {e}")
            return None
        

    def patrol_loop(self):
        """
        主巡逻逻辑
        """
        # 1. 等待 Nav2 系统激活
        self.waitUntilNav2Active()
        self.get_logger().info("Nav2 已激活, 巡逻任务开始!")

        # 2. 初始化机器人位姿
        self.init_robot_pose()

        # 3. 获取巡逻点
        patrol_points = self.get_target_points()
        if not patrol_points:
            self.get_logger().error("没有可用的巡逻点, 节点将退出")
            return
        
        # 4. 开始无限循环巡逻
        point_index = 0
        self.speach_text("巡逻任务开始") 

        while rclpy.ok():
            target_pose = patrol_points[point_index]
            self.get_logger().info(f"--- 前往巡逻点 {point_index + 1}/{len(patrol_points)} ---")

            # 导航到目标点
            nav_success = self.nav_to_pose(target_pose)

            if nav_success:
                # --- 到达后执行的动作序列 ---
                self.get_logger().info("导航成功, 执行到达后动作")

                self.speach_text("已到达目标点, 准备拍照")
                
                time.sleep(2)  # 等待机器人稳定
                self.record_image()
                time.sleep(1)
                
                self.speach_text("拍照完成")

            else:
                # 导航失败处理
                # 保留这里的语音播报
                self.speach_text("导航失败, 请检查环境或地图将在一分钟后重试")
                self.get_logger().warn("导航失败, 等待60秒后重试当前点...")
                time.sleep(60)
                continue # 重新尝试当前点

            # 移动到下一个点, 如果到了末尾则从头开始
            point_index = (point_index + 1) % len(patrol_points)
            
            # 如果不是只有一个巡逻点, 则在去下一个点前播报
            if len(patrol_points) > 1:
                self.speach_text("三秒后前往下一个目标点")
                time.sleep(3)
            else:
                self.get_logger().info("已完成所有巡逻点, 任务结束")
                break # 如果只有一个点, 可以结束循环, 或者让它继续重复


def main(args=None):
    rclpy.init(args=args)
    
    patrol_node = PatrolNode()
    
    try:
        # 在一个单独的线程中运行巡逻逻辑, 以避免阻塞spin
        # 但对于这个简单场景, 直接调用也可以, 因为nav_to_pose内部有等待循环
        patrol_node.patrol_loop()
    except KeyboardInterrupt:
        patrol_node.get_logger().info('用户中断, 正在关闭节点...')
    finally:
        patrol_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()