#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealsensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        # 声明参数
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('enable_depth', False)
        
        # 获取参数
        self.camera_name = self.get_parameter('camera_name').value
        color_width = self.get_parameter('color_width').value
        color_height = self.get_parameter('color_height').value
        color_fps = self.get_parameter('color_fps').value
        self.enable_depth = self.get_parameter('enable_depth').value
        
        # 创建发布者
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        
        if self.enable_depth:
            self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
            self.depth_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        
        self.bridge = CvBridge()
        
        # 配置RealSense管道
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 启用彩色流
        config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, color_fps)
        
        if self.enable_depth:
            config.enable_stream(rs.stream.depth, color_width, color_height, rs.format.z16, color_fps)
        
        # 启动管道
        try:
            pipeline_profile = self.pipeline.start(config)
            self.get_logger().info("RealSense pipeline started successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to start pipeline: {str(e)}")
            raise
        
        # 获取彩色流配置
        color_profile = pipeline_profile.get_stream(rs.stream.color)
        self.color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        
        # 初始化CameraInfo消息
        self.init_camera_info()
        
        if self.enable_depth:
            depth_profile = pipeline_profile.get_stream(rs.stream.depth)
            self.depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
            self.init_depth_info()
        
        self.get_logger().info("RealSense camera started, publishing images...")
        self.timer = self.create_timer(1.0/color_fps, self.timer_callback)
    
    def init_camera_info(self):
        """初始化彩色相机的CameraInfo消息"""
        self.color_info_msg = CameraInfo()
        self.color_info_msg.header.frame_id = f"{self.camera_name}_color_optical_frame"
        self.color_info_msg.height = self.color_intrinsics.height
        self.color_info_msg.width = self.color_intrinsics.width
        self.color_info_msg.distortion_model = "plumb_bob"
        
        # 设置内参矩阵 (3x3 row-major)
        self.color_info_msg.k = [
            float(self.color_intrinsics.fx), 0.0, float(self.color_intrinsics.ppx),
            0.0, float(self.color_intrinsics.fy), float(self.color_intrinsics.ppy),
            0.0, 0.0, 1.0
        ]
        
        # 设置畸变系数 (k1, k2, t1, t2, k3)
        self.color_info_msg.d = [float(coeff) for coeff in self.color_intrinsics.coeffs[:5]]
        
        # 设置投影矩阵 (3x4 row-major)
        self.color_info_msg.p = [
            float(self.color_intrinsics.fx), 0.0, float(self.color_intrinsics.ppx), 0.0,
            0.0, float(self.color_intrinsics.fy), float(self.color_intrinsics.ppy), 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
    
    def init_depth_info(self):
        """初始化深度相机的CameraInfo消息"""
        self.depth_info_msg = CameraInfo()
        self.depth_info_msg.header.frame_id = f"{self.camera_name}_depth_optical_frame"
        self.depth_info_msg.height = self.depth_intrinsics.height
        self.depth_info_msg.width = self.depth_intrinsics.width
        self.depth_info_msg.distortion_model = "plumb_bob"
        
        self.depth_info_msg.k = [
            float(self.depth_intrinsics.fx), 0.0, float(self.depth_intrinsics.ppx),
            0.0, float(self.depth_intrinsics.fy), float(self.depth_intrinsics.ppy),
            0.0, 0.0, 1.0
        ]
        
        self.depth_info_msg.d = [float(coeff) for coeff in self.depth_intrinsics.coeffs[:5]]
        
        self.depth_info_msg.p = [
            float(self.depth_intrinsics.fx), 0.0, float(self.depth_intrinsics.ppx), 0.0,
            0.0, float(self.depth_intrinsics.fy), float(self.depth_intrinsics.ppy), 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
    
    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames()
            timestamp = self.get_clock().now().to_msg()
            
            # 处理彩色图像
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_msg.header.stamp = timestamp
                color_msg.header.frame_id = self.color_info_msg.header.frame_id
                self.color_pub.publish(color_msg)
                
                self.color_info_msg.header.stamp = timestamp
                self.camera_info_pub.publish(self.color_info_msg)
            
            # 处理深度图像
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
                    depth_msg.header.stamp = timestamp
                    depth_msg.header.frame_id = self.depth_info_msg.header.frame_id
                    self.depth_pub.publish(depth_msg)
                    
                    self.depth_info_msg.header.stamp = timestamp
                    self.depth_info_pub.publish(self.depth_info_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {str(e)}", throttle_duration_sec=5)
    
    def destroy_node(self):
        self.get_logger().info("Stopping RealSense pipeline...")
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RealsensePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()