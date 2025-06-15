#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from patrol_interfaces.srv import PlayAudio  # <--- 使用我们刚创建的接口

from gtts import gTTS
import pygame
import os
import tempfile

class AudioPlayerNode(Node):
    def __init__(self):
        super().__init__('audio_player_node')
        self.srv = self.create_service(
            PlayAudio, 
            'play_audio_service',  # 服务名称
            self.handle_play_audio_request
        )
        
        # 初始化 pygame mixer
        pygame.mixer.init()
        self.get_logger().info("语音播放服务已就绪。")

    def handle_play_audio_request(self, request, response):
        text_to_speak = request.text_to_speak
        self.get_logger().info(f"收到语音播放请求: '{text_to_speak}'")

        try:
            # 1. 使用 gTTS 将文本转换为语音并保存到临时文件
            tts = gTTS(text=text_to_speak, lang='zh-cn') # 'zh-cn' 表示中文
            
            # 使用临时文件来保存mp3，程序退出后会自动清理
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=True) as fp:
                temp_filename = fp.name
                tts.save(temp_filename)
                
                # 2. 使用 pygame 播放这个音频文件
                pygame.mixer.music.load(temp_filename)
                pygame.mixer.music.play()
                
                # 3. 等待播放完成
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)

            # 4. 设置成功响应
            response.success = True
            response.message = "语音播放成功"
            self.get_logger().info("语音播放成功。")

        except Exception as e:
            # 5. 如果出错，设置失败响应
            response.success = False
            response.message = f"播放失败: {str(e)}"
            self.get_logger().error(f"语音播放时发生错误: {e}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    audio_player_node = AudioPlayerNode()
    rclpy.spin(audio_player_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()