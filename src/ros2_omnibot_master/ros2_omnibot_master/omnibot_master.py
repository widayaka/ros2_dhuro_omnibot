#!/usr/bin/env python3

import rclpy
import cv2
import time
import math
import serial
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster

class master_node(Node):
    def __init__(self):
        super().__init__('omnibot_master_node')

        self.odom_topic_name = '/odom'
        self.queue_size = 10
        self.odom_publisher = self.create_publisher(
            Odometry, 
            self.odom_topic_name,
            self.queue_size
        )

        self.twist_topic_name = "/cmd_vel"
        self.twist_subscriber = self.create_subscription(Twist, self.twist_topic_name, self.subscriberCallbackFunction, self.queue_size)


        self.tf_broadcaster = TransformBroadcaster(self)

        self.COMPort = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
        self.Baudrate = 115200
        self.serial_from_UART = serial.Serial(self.COMPort, self.Baudrate, timeout=0.01)

        self.buffer = ""

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        
        self.communication_period = 0.02
        self.timer = self.create_timer(self.communication_period, self.timerCallbackFunction)

        self.get_logger().info("Master Node Started...")

    def timerCallbackFunction(self):
        if self.serial_from_UART.in_waiting:
            self.buffer += self.serial_from_UART.read(
                self.serial_from_UART.in_waiting
            ).decode(errors='ignore')

            while '*' in self.buffer and '#' in self.buffer:
                start = self.buffer.find('*')
                stop = self.buffer.find('#', start)

                if stop == -1:
                    break

                packet = self.buffer[start+1:stop]
                self.buffer = self.buffer[stop+1:]

                try:
                    pos_x, pos_y, yaw, vel_x, vel_y, vel_w = map(
                        float,
                        packet.split(',')
                    )

                    self.get_logger().info(f"pos x:{pos_x}, pos y:{pos_y}, yaw : {yaw}, vel x:{vel_x}, vel y:{vel_y}, vel w:{vel_w} {self.velocity_x}")
                    self.publish_odometry(pos_x, pos_y, yaw, vel_x, vel_y, vel_w)

                except ValueError:
                    self.get_logger().warn(f"Packet Error : {packet}")

    def yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        return qx, qy, qz, qw
    
    def publish_odometry(self, pos_x, pos_y, yaw, vel_x, vel_y, vel_w):
        current_time = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vel_w

        self.odom_publisher.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = current_time
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"

        tf.transform.translation.x = pos_x
        tf.transform.translation.y = pos_y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

    def subscriberCallbackFunction(self, msg):
        self.velocity_x = msg.linear.x
        self.velocity_y = msg.linear.y
        self.velocity_z = msg.angular.z

        command = f"*{10},{self.velocity_x},{self.velocity_y},{self.velocity_z}#"
        self.serial_from_UART.write(command.encode())
        self.serial_from_UART.flush()

        self.get_logger().info(f'{command} - velx = {self.velocity_x} || vely = {self.velocity_y} || velz = {self.velocity_z}')

def main(args=None):
    rclpy.init(args=args)
    omnibot_master_node = master_node()
    rclpy.spin(omnibot_master_node)
    omnibot_master_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()