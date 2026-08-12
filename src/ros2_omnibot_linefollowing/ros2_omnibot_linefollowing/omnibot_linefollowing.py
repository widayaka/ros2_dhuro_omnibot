#!/usr/bin/env python3

import rclpy
import cv2
import time
import serial
import matplotlib.pyplot as plt
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class linefollowing(Node):
    def __init__(self):
        super().__init__('linefollowing_node')

        self.bridge_object = CvBridge()
        
        self.sub_camera_topic_name = 'ros2_topic_camera'
        self.queue_size = 20
        self.subscriber_camera = self.create_subscription(Image, 
                                                          self.sub_camera_topic_name, 
                                                          self.subscriberCallbackFunction, 
                                                          self.queue_size)

        # self.COMPort = '/dev/ttyAMA10'
        self.COMPort = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
        self.Baudrate = 115200

        self.detector = LineDetectorModule(threshold=120)
        self.pid_control = PIDControllerModule(1.0, 0.0, 0.0)
        self.motor = SerialCommunicationModule(self.COMPort, self.Baudrate)

        self.lin_speed = 5
        self.ang_speed = 0
        self.wheel_speed = 120

        self.start_time = time.time()

        self.time_log = []
        self.setpoint_log = []
        self.error_log = []
        self.pid_log = []
        self.left_speed_log = []
        self.right_speed_log = []

    def plot_pid_response(self):
        plt.figure(figsize=(10, 6))

        # plot error
        plt.subplot(2, 2, 1)
        plt.plot(self.time_log, self.error_log, label='Error')
        plt.plot(self.time_log, self.setpoint_log, '--', label='Setpoint')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.title('PID Response')
        plt.legend()
        plt.grid()

        # plot PID Output
        plt.subplot(2, 2, 2)
        plt.plot(self.time_log, self.pid_log, label='PID Output', color='orange')
        plt.xlabel('Time (s)')
        plt.ylabel('Control Signal')
        plt.legend()
        plt.grid()

        # plot left motor speed
        plt.subplot(2, 2, 3)
        plt.plot(self.time_log, self.left_speed_log, label='Left Speed')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed Left')
        plt.legend()
        plt.grid()

        # plot right motor speed
        plt.subplot(2, 2, 4)
        plt.plot(self.time_log, self.right_speed_log, label='Right Speed')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed Right')
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show(block=True)
    
    def subscriberCallbackFunction(self, msg):
        frame_RGB = self.bridge_object.imgmsg_to_cv2(msg)
        frame_RGB_copy = frame_RGB.copy()

        error, frame_contour_detection, frame_gray, frame_binary = self.detector.process(frame_RGB_copy)
        control_signal = self.pid_control.compute(error)
        self.ang_speed = control_signal
        left_speed = self.wheel_speed + control_signal
        right_speed = self.wheel_speed - control_signal

        # self.motor.sendSpeed(left_speed, right_speed)
        lin_speed = self.lin_speed
        ang_speed = self.ang_speed
        self.motor.sendLinAngSpeed(error, self.lin_speed, self.ang_speed)

        current_time = time.time() - self.start_time

        self.time_log.append(current_time)
        self.setpoint_log.append(0)
        self.error_log.append(error)
        self.pid_log.append(control_signal)
        self.left_speed_log.append(left_speed)
        self.right_speed_log.append(right_speed)

        self.get_logger().info(f"Error = {error}, PID = {control_signal}, Lin Speed = {self.lin_speed}, Ang Speed = {self.ang_speed}")

        # cv2.imshow('[Subscriber] - Omnibot Camera', frame_RGB)
        # cv2.imshow('[Subscriber] - Omnibot Gray', frame_gray)
        # cv2.imshow('[Subscriber] - Omnibot Binary', frame_binary)
        # cv2.imshow('[Subscriber] - Omnibot Detection', frame_contour_detection)

        cv2.waitKey(1)

class LineDetectorModule:
    def __init__(self, width=640, height=480, threshold=100):
        self.width = width
        self.height = height
        self.threshold = threshold
        self.center_x = width // 2
        self.center_y = height // 2

    def process(self, frame):
        input_frame = frame
        input_frame_copy = input_frame.copy()

        frame_height, frame_width, _ = input_frame.shape

        roi_top = int(frame_height * 0.25)
        roi_bottom = int(frame_height * 0.75)
        roi_left = int(frame_width * 0.25)
        roi_right = int(frame_width * 0.75)

        roi_frame = input_frame_copy[roi_top:roi_bottom, roi_left:roi_right]
        roi_frame_copy = roi_frame.copy()
                                
        roi_frame_height = roi_frame.shape[0]
        roi_frame_width = roi_frame.shape[1]

        roi_center_x = roi_frame_width // 2
        roi_center_y = roi_frame_height // 2

        # cv2.rectangle(input_frame, (roi_left, roi_top), (roi_right, roi_bottom), (0,255,0), 1, cv2.LINE_AA)
        # cv2.circle(input_frame, (roi_center_x, roi_center_y), 2, (0,255,0), 2, cv2.LINE_AA)

        gray_frame = cv2.cvtColor(roi_frame_copy, cv2.COLOR_BGR2GRAY)
        blur_frame = cv2.GaussianBlur(gray_frame, (7,7), 0)
        _, binary_frame = cv2.threshold(blur_frame, self.threshold, 255, cv2.THRESH_BINARY_INV)
        
        cx, cy = None, None

        scan_y = min(roi_frame_height - 1, roi_frame_height - 10)
        row = binary_frame[scan_y]
        white_pixels = np.where(row == 255)[0]

        cv2.line(roi_frame, (0, scan_y), (roi_frame_width, scan_y), (0,0,255), 1)

        if len(white_pixels) > 0:
            cx = int(np.mean(white_pixels))
            cy = scan_y

            cv2.circle(roi_frame, (cx,cy), 2, (0,255,0), 1, cv2.LINE_AA)
            cv2.line(roi_frame, (cx, cy), (roi_center_x, roi_center_y), (255,255,255), 1, cv2.LINE_AA)

        else:
            edge_detection = cv2.Canny(binary_frame, 100, 255)
            contours, _ = cv2.findContours(edge_detection, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 100:
                    moment = cv2.moments(largest)
                    if moment['m00'] != 0:
                        cx = int(moment['m10'] / moment['m00'])
                        cy = int(moment['m01'] / moment['m00'])
                        cv2.drawContours(roi_frame, [largest], -1, (0,0,255), 2, cv2.LINE_AA)
            
        cv2.circle(roi_frame, (roi_center_x, roi_center_y), 2, (255,255,0), 1, cv2.LINE_AA)
        cv2.imshow('ROI Binary', binary_frame)
        cv2.imshow('ROI Camera', roi_frame)

        if cx is not None and cy is not None:
            cv2.line(roi_frame, (roi_center_x, roi_center_y), (cx, cy), (255,255,255), 1, cv2.LINE_AA)
            error = roi_center_x - cx

        else:
            error = 0

        return error, input_frame, gray_frame, binary_frame
        
class PIDControllerModule:
    def __init__(self, kp, ki, kd, output_limit=255):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.output_limit = output_limit

    def compute(self, error):
        P = self.kp * error
        self.integral += error
        I = self.ki * self.integral
        D = self.kd * (error - self.last_error)
        self.last_error = error

        output = P + I + D
        return max(min(output, self.output_limit), -self.output_limit)
    
class MotorInterface:
    def send(self, left_spped, right_speed):
        raise NotImplementedError
    
class SerialCommunicationModule:
    def __init__(self, COMPort, baudrate=115200):
        self.serial = serial.Serial(COMPort, baudrate, timeout=0.1)
        print('Serial Open', COMPort)

    def sendSpeed(self, left_speed, right_speed):
        left_speed = int(max(min(left_speed, 255), -255))
        right_speed = int(max(min(right_speed, 255), -255))
        command = f"*10,{left_speed},{right_speed}#"
        self.serial.write(command.encode())
        self.serial.flush()
        print(command, flush=True)

    def sendLinAngSpeed(self, error, lin_speed, ang_speed):
        lin_speed = int(max(min(lin_speed, 255), -255))
        ang_speed = int(max(min(ang_speed, 360), -360))
        command = f"*12,{error},{lin_speed},{ang_speed}#"
        print(command)
        self.serial.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = linefollowing()
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()
        subscriber_node.plot_pid_response()

if __name__ == '__main__':
    main()