#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealsenseImagePublisher(Node):
    def __init__(self):
        super().__init__('realsense_image_publisher')
        
        # 创建发布者
        self.color_publisher = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        
        # 初始化 CV Bridge
        self.bridge = CvBridge()
        
        # 配置 RealSense 管道
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 获取设备产品线以设置分辨率
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        
        # 检查是否支持 RGB 相机
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            self.get_logger().error("The device does not have an RGB camera!")
            rclpy.shutdown()
            return
        
        # 启用彩色和深度流
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # 启动管道
        self.pipeline.start(config)
        
        self.get_logger().info("RealSense camera started, publishing images...")
        
        # 创建定时器以定期发布图像
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
    
    def timer_callback(self):
        try:
            # 等待一组帧
            frames = self.pipeline.wait_for_frames()
            
            # 获取彩色帧
            color_frame = frames.get_color_frame()
            if not color_frame:
                self.get_logger().warn("No color frame received")
                return
            
            # 获取深度帧
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                self.get_logger().warn("No depth frame received")
                return
            
            # 将图像转换为 numpy 数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 转换颜色图像为 ROS 消息并发布
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = self.get_clock().now().to_msg()
            color_msg.header.frame_id = "camera_color_optical_frame"
            self.color_publisher.publish(color_msg)
            
            # 转换深度图像为 ROS 消息并发布
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            self.depth_publisher.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error capturing or publishing images: {str(e)}")
    
    def destroy_node(self):
        self.get_logger().info("Shutting down RealSense pipeline...")
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = RealsenseImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()