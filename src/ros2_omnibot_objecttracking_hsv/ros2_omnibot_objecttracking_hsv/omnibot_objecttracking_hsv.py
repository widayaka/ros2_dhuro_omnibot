#!/usr/bin/env python3

import rclpy
import cv2
import time
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image

class object_tracking_HSV(Node):
    def __init__(self):
        super().__init__('objecttracking_hsv_node')
        self.camera_device_num = 0
        self.camera_width = 640
        self.camera_height = 480
        self.camera_stream = cv2.VideoCapture(self.camera_device_num)
        self.camera_stream.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.camera_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.topic_name = 'ros2_topic_camera'
        self.queue_size = 20
        self.publisher = self.create_publisher(Int16MultiArray, self.topic_name, self.queue_size)
        self.communication_period = 0.02
        self.timer_callback = self.create_timer(self.communication_period, self.timerCallbackFunction)
        self.x_error = 0
        self.y_error = 0
        self.d_error = 0
        self.previous_time = 0
        self.trackbar_name = "Omnibot HSV Trackbar"
        cv2.namedWindow(self.trackbar_name)
        self.createTrackbarFunction()

    def createTrackbarFunction(self):
        cv2.namedWindow(self.trackbar_name)
        cv2.createTrackbar("H Min", self.trackbar_name, 0, 179, lambda x: None)
        cv2.createTrackbar("H Max", self.trackbar_name, 179, 179, lambda x: None)
        cv2.createTrackbar("S Min", self.trackbar_name, 0, 255, lambda x: None)
        cv2.createTrackbar("S Max", self.trackbar_name, 255, 255, lambda x: None)
        cv2.createTrackbar("V Min", self.trackbar_name, 0, 255, lambda x: None)
        cv2.createTrackbar("V Max", self.trackbar_name, 255, 255, lambda x: None)

    def getTrackbarValueFunction(self):
        h_min = cv2.getTrackbarPos("H Min", self.trackbar_name)
        h_max = cv2.getTrackbarPos("H Max", self.trackbar_name)
        s_min = cv2.getTrackbarPos("S Min", self.trackbar_name)
        s_max = cv2.getTrackbarPos("S Max", self.trackbar_name)
        v_min = cv2.getTrackbarPos("V Min", self.trackbar_name)
        v_max = cv2.getTrackbarPos("V Max", self.trackbar_name)
        return (h_min, s_min, v_min), (h_max, s_max, v_max)
    
    def timerCallbackFunction(self):
        success, frame_RGB = self.camera_stream.read()
        frame_RGB_copied = frame_RGB.copy()
        self.frame_width = frame_RGB.shape[1]
        self.frame_height = frame_RGB.shape[0]
        self.center_frame_x = (int)(self.frame_width/2)
        self.center_frame_y = (int)(self.frame_height/2)
        self.center_frame = (self.center_frame_x, self.center_frame_y)

        frame_HSV = cv2.cvtColor(frame_RGB, cv2.COLOR_BGR2HSV)
        HSV_lower_value, HSV_higher_value = self.getTrackbarValueFunction()

        frame_binary = cv2.inRange(frame_HSV, np.array(HSV_lower_value), np.array(HSV_higher_value))
        kernel = np.ones((5, 5), np.uint8)
        frame_binary = cv2.morphologyEx(frame_binary, cv2.MORPH_OPEN, kernel=kernel)
        frame_binary = cv2.morphologyEx(frame_binary, cv2.MORPH_CLOSE, kernel=kernel)
        frame_RGB_copied = cv2.bitwise_and(frame_RGB_copied, frame_RGB_copied, mask=frame_binary)

        contours, hierarchy = cv2.findContours(frame_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total_contours = len(contours)

        self.object_name = 'Ball'
        self.object_flag = 0
        self.object_flag_name = 'Object Detected: '
        self.object_flag_true = 'Detected'
        self.object_flag_false = 'Not Detected'

        self.error_x_str = 'Error X = '
        self.error_y_str = 'Error Y = '

        cv2.putText(frame_RGB, self.object_flag_name, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame_RGB, self.error_x_str, (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame_RGB, self.error_y_str, (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        if (total_contours > 0):
            for contour in contours:
                contour_area = cv2.contourArea(contour)
                if (contour_area > 1000):
                    self.object_flag = 1
                    detected_object = cv2.drawContours(frame_RGB_copied, contours, -1, (0,255,0), 1)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame_RGB, (x, y), (x+w, y+h), (0, 255, 0), 1)

                    cv2.putText(frame_RGB, self.object_flag_true, (140, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    cv2.putText(frame_RGB, self.object_name, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

                    x_center_object = (int)(x+w/2)
                    y_center_object = (int)(y+h/2)
                    center_object = (x_center_object, y_center_object)

                    cv2.circle(frame_RGB, center_object, 2, (0, 0, 255), -1, cv2.LINE_AA)
                    cv2.circle(frame_RGB, self.center_frame, 2, (0, 0, 255), -1, cv2.LINE_AA)
                    cv2.line(frame_RGB, self.center_frame, center_object, (255, 255, 255), 1, cv2.LINE_AA)
                    
                    self.x_error = x_center_object - self.center_frame_x
                    self.y_error = y_center_object - self.center_frame_y

                    if (self.x_error > 255):
                        self.x_error = 255
                    
                    if (self.x_error < -255):
                        self.x_error = -255

                    if (self.y_error > 255):
                        self.y_error = 255
                    
                    if (self.y_error < -255):
                        self.y_error = -255
                    
                    cv2.putText(frame_RGB, str(self.x_error), (90, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    cv2.putText(frame_RGB, str(self.y_error), (90, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        else:
            self.object_flag = 0
            cv2.putText(frame_RGB, self.object_flag_false, (140, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(frame_RGB, self.object_flag_false, (90, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(frame_RGB, self.object_flag_false, (90, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        self.current_time = time.time()
        fps = 1 / (self.current_time - self.previous_time)
        self.previous_time = self.current_time

        cv2.putText(frame_RGB, f'FPS: {int(fps)}', (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow('Robot Camera - Original', frame_RGB)
        cv2.imshow('Robot Camera - HSV', frame_HSV)
        cv2.imshow('Robot Camera - BW', frame_binary)
        cv2.imshow('Robot Camera - Detected Object', frame_RGB_copied)

        cv2.waitKey(1)

        message = Int16MultiArray()
        message.data = [self.x_error, self.y_error]
        self.publisher.publish(message)
        self.get_logger().info(f'Camera Publisher Node - Publishing Error X = {self.x_error}, Y = {self.y_error}')

def main(args=None):
    rclpy.init(args=args)
    publisher_node = object_tracking_HSV()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()