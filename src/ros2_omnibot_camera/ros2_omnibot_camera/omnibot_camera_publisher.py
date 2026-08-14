#!/usr/bin/env python3

import rclpy
import cv2
import time

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class camera_publisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        self.camera_device_num = 0
        self.camera_width = 480
        self.camera_height = 360

        self.camera = cv2.VideoCapture(self.camera_device_num)
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.camera.set(cv2.CAP_PROP_FOCUS, 0)
        
        self.bridge_object = CvBridge()
        
        self.queue_size = 10
        self.communication_period = 0.01

        self.camera_topic_name = '/camera/omnibot_camera_raw'
        self.camera_publisher = self.create_publisher(Image, self.camera_topic_name, self.queue_size)
        self.camera_timer = self.create_timer(self.communication_period, self.cameraCallbackFunction)

        self.previous_time = 0
        self.current_time = 0
        self.frame_per_second = 0

        self.declare_parameter('show_fps', False)
    
    def cameraCallbackFunction(self):
        success, frame_RGB = self.camera.read()
        
        if not success:
            self.get_logger().info('Failed initializing camera')
        
        frame_RGB = cv2.rotate(frame_RGB, cv2.ROTATE_180)

        image_message = self.bridge_object.cv2_to_imgmsg(frame_RGB, encoding='bgr8')
        self.camera_publisher.publish(image_message)

        self.current_time = time.time()
        self.frame_per_second = int(1 / (self.current_time - self.previous_time))
        self.previous_time = self.current_time

        show_fps = self.get_parameter('show_fps').value
        
        if show_fps:
            self.get_logger().info(f"FPS : {self.frame_per_second}")
        
        cv2.imshow('[Publisher] - Omnibot Camera Stream', frame_RGB)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    publisher_node = camera_publisher()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()