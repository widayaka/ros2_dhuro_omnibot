#!/usr/bin/env python3

import rclpy
import cv2
import time
import serial
import matplotlib.pyplot as plt
import numpy as np
import os
import threading

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

class object_tracking_aruco(Node):
    def __init__(self):
        super().__init__('objecttracking_aruco_node')
        self.bridge_object = CvBridge()

        self.COMPort = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
        self.Baudrate = 115200

        # self.serial_to_robot = serial.Serial(self.COMPort, self.Baudrate, timeout=0.1)

        self.serial_to_robot = None
        self.connect_serial()

        self.queue_size = 20
        self.communication_period = 0.02
        self.camera_processing_rate = 100.0

        camera_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self.latest_frame = None 
        self.frame_lock = threading.Lock()
        self.new_frame_available = False

        self.camera_sub_topic_name = '/omnibot_camera'
        self.camera_subscriber = self.create_subscription(Image, self.camera_sub_topic_name, self.cameraSubCallbackFunction, camera_qos)

        self.aruco_pub_topic_name = '/aruco_pose_estimation'
        self.aruco_publisher = self.create_publisher(Float32MultiArray, self.aruco_pub_topic_name, self.queue_size)

        self.processing_timer = self.create_timer(1.0 / self.camera_processing_rate, self.processLatestFrame)
        self.timer_callback = self.create_timer(self.communication_period, self.publisherCallbackFunction)
        self.serial_timer = self.create_timer(self.communication_period, self.serialCallbackFunction)

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        self.npz_matrix_path = '/home/widayaka/ROS2_Projects/ros2_dhuro_omnibot/src/ros2_omnibot_objecttracking_aruco/ros2_omnibot_objecttracking_aruco/MultiMatrix.npz'
        
        try:
            with np.load(self.npz_matrix_path) as data:
                self.camera_matrix = data['camMatrix']
                self.dist_coeffs = data['distCoef']
        
        except Exception as e:
            print(f'calibration failed: {e}')
            raise

        self.marker_size = 0.10
        self.target_ids = [0, 1, 2, 3, 4, 5]

        self.aruco_id = 0
        self.marker_x = 0.0
        self.marker_y = 0.0
        self.marker_z = 0.0
        self.marker_yaw = 0.0

        self.prev_id = 0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_yaw = 0.0

        self.x_offset = 0.08
        self.y_offset = 0.05
        self.z_offset = 0.10

        self.aruco_detection_flag = False

        self.x_error = 0.0
        self.y_error = 0.0
        self.z_error = 0.0
        self.yaw_error = 0.0

        self.x_setpoint = 0.0
        self.y_setpoint = 0.0
        self.z_setpoint = 0.25
        self.yaw_setpoint = 0.0

        self.center_frame_x = 0
        self.center_frame_y = 0
        self.center_aruco_x = 0
        self.center_aruco_y = 0

        self.x_ref_z = 0.30
        self.x_z_compensation = 0.20

        self.hold_timeout = 0.25
        
        self.time_now = 0
        self.previous_time = 0
        self.current_time = 0
        self.last_seen_time = 0
        self.frame_per_second = 0.0
        self.previous_frame_time = None
        self.fps_alpha = 0.1

        self.commandToSend = '*10,0,0,0,0,0,0#'
        
        # Konfigurasi parameter PID -> kpx, kix, kdx, kpz, kiz, kdz, kpyaw, kiyaw, kdyaw
        self.PID = PIDControllerModule(0.05, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.05)
        
        self.start_time = time.time()
        self.time_log = []

        self.x_setpoint_log = []
        self.x_error_log = []

        self.z_setpoint_log = []
        self.z_error_log = []
        
        self.yaw_setpoint_log = []
        self.yaw_error_log = []

        self.aruco_flag_name = 'ArUco Marker ID:'
        self.aruco_status_name = 'ArUco Status:'
        self.aruco_status_detected = 'Detected'
        self.aruco_status_undetected = 'Not Detected'

        self.error_x_name = 'ArUco Error X:'
        self.error_y_name = 'ArUco Error Y:'
        self.error_z_name = 'ArUco Error Z:'
        self.error_yaw_name = 'ArUco Error Yaw:'

        self.processing_times = {}
    
    def cameraSubCallbackFunction(self, msg):
        try:
            frame = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        with self.frame_lock:
            self.latest_frame = frame
            self.new_frame_available = True

    def processLatestFrame(self):
        process_start = time.perf_counter()

        step_start = time.perf_counter()
        with self.frame_lock:
            if (self.latest_frame is None or not self.new_frame_available):
                return
            frame = self.latest_frame
            self.new_frame_available = False
        step_start = self.measure_time("Frame", step_start)

        step_two = time.perf_counter()
        current_time = time.monotonic()
        if self.previous_frame_time is not None:
            dt = current_time - self.previous_frame_time
            
            if dt > 0.0:
                instantaneous_fps = 1.0 / dt
                
                if self.frame_per_second == 0.0:
                    self.frame_per_second = instantaneous_fps

                else:
                    self.frame_per_second = (self.fps_alpha * instantaneous_fps + (1.0 - self.fps_alpha) * self.frame_per_second)
        
        self.previous_frame_time = current_time
        step_two = self.measure_time("FPS Calc", step_two)

        step_three = time.perf_counter()
        self.frame_width = frame.shape[1]
        self.frame_height = frame.shape[0]
        
        self.center_frame_x = (int)(self.frame_width/2)
        self.center_frame_y = (int)(self.frame_height/2)
        
        self.center_frame = (self.center_frame_x, self.center_frame_y)

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        step_three = self.measure_time("Gray", step_three)
        
        step_four = time.perf_counter()
        self.aruco_detection_flag = False
        corners, ids, _ = self.aruco_detector.detectMarkers(frame_gray)
        step_four = self.measure_time("Aruco Detection", step_four)
        
        step_five = time.perf_counter()
        if ids is not None:
            
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            selected_index = None 
            selected_id = None
            step_five = self.measure_time("Pose Estimation", step_five)
            
            step_six = time.perf_counter()
            for i, marker_id, in enumerate(ids.flatten()):
                marker_id = int(marker_id)
                if marker_id in self.target_ids:
                    selected_index = i
                    selected_id = marker_id
            step_six = self.measure_time("Marker Selection", step_six)

            step_seven = time.perf_counter()
            if selected_index is not None:
                
                i = selected_index
                marker_id = selected_id

                points = corners[i][0]
                self.center_aruco_x = int(np.mean(points[:,0]))
                self.center_aruco_y = int(np.mean(points[:,1]))
                x, y, z, = tvecs[i][0]
                R, _ = cv2.Rodrigues(rvecs[i])
                yaw_raw = np.degrees(np.arctan2(-R[2, 0], R[0, 0]))

                x -= self.x_offset
                y -= self.y_offset
                z -= self.z_offset

                x = x - ((z - self.x_ref_z) * self.x_z_compensation)
                alpha = 0.6
                step_seven = self.measure_time("Pose Calculation", step_seven)
                
                step_eight = time.perf_counter()
                self.marker_x = alpha * x + (1 - alpha) * self.prev_x
                self.marker_y = alpha * y + (1 - alpha) * self.prev_y
                self.marker_z = alpha * z + (1 - alpha) * self.prev_z
                self.marker_yaw = alpha * yaw_raw + (1 - alpha) * self.prev_yaw

                self.aruco_id = marker_id
                self.prev_id = marker_id
                self.prev_x = self.marker_x
                self.prev_y = self.marker_y
                self.prev_z = self.marker_z
                self.prev_yaw = self.marker_yaw
                self.last_seen_time = time.monotonic()

                self.x_error = self.center_frame_x - self.center_aruco_x
                self.y_error = self.center_frame_y - self.center_aruco_y
                self.z_error = self.z_setpoint - self.marker_z
                self.yaw_error = self.yaw_setpoint - yaw_raw

                # self.z_error = round(self.z_error, 2)
                # self.yaw_error = round(self.yaw_error, 2)

                self.aruco_detection_flag = True
                step_eight = self.measure_time("Error Calculation", step_eight)
                    # current_time = time.time() - self.start_time
                    # self.time_log.append(current_time)
                    # self.x_setpoint_log.append(0)
                    # self.x_error_log.append(self.x_error)
                    # self.z_setpoint_log.append(0)
                    # self.z_error_log.append(self.z_error)

                step_nine = time.perf_counter()
                    # cv2.drawFrameAxes(frame_RGB_copy, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                cv2.circle(frame,(self.center_aruco_x, self.center_aruco_y), 5, (0, 0, 255), -1)
                cv2.circle(frame,(self.center_frame_x, self.center_frame_y), 5, (0, 0, 255), -1)                    
                cv2.line(frame, (self.center_frame_x, self.center_frame_y), (self.center_aruco_x, self.center_aruco_y), (255, 255, 255), 1, cv2.LINE_AA)
                    # cv2.putText(frame_RGB_copy, str(marker_id), (150, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    # cv2.putText(frame_RGB_copy, self.aruco_status_detected, (150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    # cv2.putText(frame_RGB_copy, str(self.x_error), (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    # cv2.putText(frame_RGB_copy, str(self.y_error), (150, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    # cv2.putText(frame_RGB_copy, str(self.z_error), (150, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    # cv2.putText(frame_RGB_copy, str(self.yaw_error), (150, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                step_nine = self.measure_time("Drawing", step_nine)

        if not self.aruco_detection_flag:
            # cv2.putText(frame_RGB_copy, str("no IDs"), (150, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame_RGB_copy, self.aruco_status_undetected, (150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame_RGB_copy, self.aruco_status_undetected, (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame_RGB_copy, self.aruco_status_undetected, (150, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame_RGB_copy, self.aruco_status_undetected, (150, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            # cv2.putText(frame_RGB_copy, self.aruco_status_undetected, (150, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            elapsed = (time.monotonic() - self.last_seen_time)

            if elapsed < self.hold_timeout:
                self.aruco_id = self.prev_id
                self.marker_x = self.prev_x
                self.marker_y = self.prev_y
                self.marker_z = self.prev_z
                self.marker_yaw = self.prev_yaw

            else:
                self.aruco_id = 0
                self.marker_x = 0.0
                self.marker_y = 0.0
                self.marker_z = 0.0
                self.marker_yaw = 0.0

                self.x_error = 0.0
                self.y_error = 0.0
                self.z_error = 0.0
                self.yaw_error = 0.0

        control_signalx, control_signalz, control_signalyaw = self.PID.calculation(self.x_error, self.z_error, self.yaw_error)
        self.commandToSend = f'*10,{self.x_error},{self.y_error},{self.z_error},{control_signalx},{control_signalz},{control_signalyaw}#'

        # cv2.putText(frame_RGB_copy, self.aruco_flag_name, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        # cv2.putText(frame_RGB_copy, self.aruco_status_name, (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        # cv2.putText(frame_RGB_copy, self.error_x_name, (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        # cv2.putText(frame_RGB_copy, self.error_y_name, (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        # cv2.putText(frame_RGB_copy, self.error_z_name, (5, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        # cv2.putText(frame_RGB_copy, self.error_yaw_name, (5, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        cv2.putText(frame, str('FPS:'), (550,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        
        
        # self.get_logger().info(f'ArUco Marker : ID_{self.aruco_id}, X = {self.marker_x:.3f}, Y = {self.marker_y:.3f}, Z = {self.marker_z:.3f}, Yaw = {self.marker_yaw:.3f}, command = {self.commandToSend}')

        process_time = (time.perf_counter() - process_start) * 1000.0
        self.processing_times["Total"] = process_time

        self.get_logger().info(
            f"Frame: {self.processing_times.get('Frame', 0.0):.3f} ms | "
            f"FPS Calc: {self.processing_times.get('FPS Calc', 0.0):.3f} ms | "
            f"Gray: {self.processing_times.get('Gray', 0.0):.3f} ms | "
            f"Aruco Detection: {self.processing_times.get('Aruco Detection', 0.0):.3f} ms | "
            f"Marker Selection: {self.processing_times.get('Marker Selection', 0.0):.3f} ms | "
            f"Pose Calculation: {self.processing_times.get('Pose Calculation', 0.0):.3f} ms | "
            f"Error Calculation: {self.processing_times.get('Error Calculation', 0.0):.3f} ms | "
            f"Drawing: {self.processing_times.get('Drawing', 0.0):.3f} ms | "
            f"Total: {process_time:.3f} ms | "
        )

        processing_fps = 1000.0 / process_time if process_time > 0 else 0.0
        cv2.putText(frame, f"{processing_fps:.1f}", (590,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.imshow("ArUco Marker Detection", frame)
        cv2.waitKey(1)

    def measure_time(self, name, start_time):
        elapsed = (time.perf_counter() - start_time) * 1000.0
        self.processing_times[name] = elapsed
        return time.perf_counter()

    def publisherCallbackFunction(self):
        message = Float32MultiArray()
        message.data = [int(self.aruco_id), self.marker_x, self.marker_y, self.marker_z, self.marker_yaw]
        self.aruco_publisher.publish(message)

    def serialCallbackFunction(self):
        if self.serial_to_robot is None:
            self.connect_serial()
            return
    
        if not self.serial_to_robot.is_open:
            self.connect_serial()
            return
        
        try:
            self.serial_to_robot.write(self.commandToSend.encode('ascii'))
        
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Serial Error! {e}")

            try:
                self.serial_to_robot.close()
            
            except Exception:
                pass

            self.serial_to_robot = None

    def connect_serial(self):
        try:
            self.serial_to_robot = serial.Serial(self.COMPort, self.Baudrate, timeout=0.1, write_timeout=0.1)
            self.get_logger().info(f"Serial connected: {self.COMPort}")

        except serial.SerialException as e:
            self.serial_to_robot = None
            self.get_logger().error(f"Failed to connect serial: {e}")

    def plotPIDResponse(self):
        plt.figure(figsize=(10, 6))

        # plot error
        # plt.subplot(2, 2, 1)
        # plt.plot(self.time_log, self.x_error_log, label='Error X-Axis')
        # plt.plot(self.time_log, self.x_setpoint_log, '--', label='Setpoint X-Axis')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Error X-Axis (m)')
        # plt.title('X-Axis PID Response')
        # plt.legend()
        # plt.grid()

        # plt.subplot(2, 2, 2)
        plt.plot(self.time_log, self.z_error_log, label='Error Z-Axis')
        plt.plot(self.time_log, self.z_setpoint_log, '--', label='Setpoint Z-Axis')
        plt.xlabel('Time (s)')
        plt.ylabel('Error Z-Axis (m)')
        plt.title('Z-Axis PID Response')
        plt.legend()
        plt.grid()

        # plt.subplot(2, 2, 3)
        # plt.plot(self.time_log, self.yaw_error_log, label='Error Yaw-Axis')
        # plt.plot(self.time_log, self.yaw_setpoint_log, '--', label='Setpoint Yaw-Axis')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Error Yaw-Axis (deg)')
        # plt.title('Yaw-Axis PID Response')
        # plt.legend()
        # plt.grid()

        plt.tight_layout()
        # plt.savefig("/home/widayaka/ROS2_Projects/ros2_dhuro_omnibot/src/ros2_omnibot_objecttracking_aruco/figure/PIDx_Response.png")
        plt.show(block=True)

class PIDControllerModule():
    def __init__(self, kpx, kix, kdx, kpz, kiz, kdz, kpyaw, kiyaw, kdyaw, output_limit=255):
        self.kpx = kpx
        self.kix = kix
        self.kdx = kdx

        self.kpz = kpz
        self.kiz = kiz
        self.kdz = kdz

        self.kpyaw = kpyaw
        self.kiyaw = kiyaw
        self.kdyaw = kdyaw

        self.integralx = 0.0
        self.integralz = 0.0
        self.integralyaw = 0.0

        self.last_errorx = 0.0
        self.last_errorz = 0.0
        self.last_erroryaw = 0.0

        self.output_limit = output_limit

    def calculation(self, error_x, error_z, error_yaw):
        Px = self.kpx * error_x
        self.integralx += error_x
        Ix = self.kix * self.integralx
        Dx = self.kdx * (error_x - self.last_errorx)
        self.last_errorx = error_x
        PIDx = Px + Ix + Dx

        Pz = self.kpz * error_z
        self.integralz += error_z
        Iz = self.kiz * self.integralz
        Dz = self.kdz * (error_z - self.last_errorz)
        self.last_errorz = error_z
        PIDz = Pz + Iz + Dz

        Pyaw = self.kpyaw * error_yaw
        self.integralyaw += error_yaw
        Iyaw = self.kiyaw * self.integralyaw
        Dyaw = self.kdyaw * (error_yaw - self.last_erroryaw)
        self.last_erroryaw = error_yaw
        PIDyaw = Pyaw + Iyaw + Dyaw

        return max(min(PIDx, self.output_limit), -self.output_limit), max(min(PIDz, self.output_limit), -self.output_limit), max(min(PIDyaw, self.output_limit), -self.output_limit)

def main(args=None):
    rclpy.init(args=args)
    publisher_node = object_tracking_aruco()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()