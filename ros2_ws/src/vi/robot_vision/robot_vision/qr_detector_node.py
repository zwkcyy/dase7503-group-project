#!/usr/bin/env python3

import csv
import os
import re
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class QRDetectorNode(Node):

    def __init__(self):
        super().__init__('qr_detector')

        # =====================================================
        # Parameters
        # =====================================================
        self.declare_parameter(
            'image_topic',
            '/camera/image_raw'
        )

        self.declare_parameter(
            'process_hz',
            10.0
        )

        # Same QR must be detected twice before confirmation
        self.declare_parameter(
            'confirm_hits',
            2
        )

        self.declare_parameter(
            'confirm_window',
            0.8
        )

        self.declare_parameter(
            'model_dir',
            '~/.local/share/robot_vision/wechat_qrcode'
        )

        self.declare_parameter(
            'log_file',
            '~/.ros/robot_vision/qr_scan_log.csv'
        )

        self.image_topic = (
            self.get_parameter('image_topic')
            .get_parameter_value()
            .string_value
        )

        self.process_hz = (
            self.get_parameter('process_hz')
            .get_parameter_value()
            .double_value
        )

        self.confirm_hits = (
            self.get_parameter('confirm_hits')
            .get_parameter_value()
            .integer_value
        )

        self.confirm_window = (
            self.get_parameter('confirm_window')
            .get_parameter_value()
            .double_value
        )

        self.model_dir = os.path.expanduser(
            self.get_parameter('model_dir')
            .get_parameter_value()
            .string_value
        )

        self.log_file = os.path.expanduser(
            self.get_parameter('log_file')
            .get_parameter_value()
            .string_value
        )

        if self.process_hz <= 0.0:
            self.process_hz = 10.0

        self.process_period = 1.0 / self.process_hz

        # =====================================================
        # Valid QR format
        # =====================================================
        self.valid_pattern = re.compile(
            r'^(START|END|RACK[A-D]_[A-Za-z0-9]+)$'
        )

        # =====================================================
        # Vision
        # =====================================================
        self.bridge = CvBridge()

        detect_proto = os.path.join(
            self.model_dir,
            'detect.prototxt'
        )

        detect_model = os.path.join(
            self.model_dir,
            'detect.caffemodel'
        )

        sr_proto = os.path.join(
            self.model_dir,
            'sr.prototxt'
        )

        sr_model = os.path.join(
            self.model_dir,
            'sr.caffemodel'
        )

        # Primary detector
        self.wechat_detector = (
            cv2.wechat_qrcode_WeChatQRCode(
                detect_proto,
                detect_model,
                sr_proto,
                sr_model
            )
        )

        # Fallback detector
        self.opencv_detector = cv2.QRCodeDetector()

        # =====================================================
        # Scanner state
        # =====================================================

        # IMPORTANT:
        # Default = IDLE
        # No QR inference until /qr/start_scan is called
        self.scanning_enabled = False

        self.last_process_time = 0.0

        self.hit_history = deque()

        self.last_confirmed = None

        # =====================================================
        # ROS publishers
        # =====================================================
        self.detected_pub = self.create_publisher(
            Bool,
            '/qr/detected',
            10
        )

        self.data_pub = self.create_publisher(
            String,
            '/qr/data',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/qr/debug_image',
            10
        )

        # =====================================================
        # ROS service
        # =====================================================
        self.start_scan_service = self.create_service(
            Trigger,
            '/qr/start_scan',
            self.start_scan_callback
        )

        # =====================================================
        # Camera subscriber
        # =====================================================
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.prepare_log()

        self.get_logger().info('')
        self.get_logger().info(
            '=========================================='
        )
        self.get_logger().info(
            'DASE7503 QR Scanner V4'
        )
        self.get_logger().info(
            'Mode     : On-demand scanning'
        )
        self.get_logger().info(
            'Primary  : WeChatQRCode CNN + SR'
        )
        self.get_logger().info(
            'Fallback : OpenCV QRCodeDetector'
        )
        self.get_logger().info(
            'State    : IDLE'
        )
        self.get_logger().info(
            'Service  : /qr/start_scan'
        )
        self.get_logger().info(
            'Result   : /qr/data'
        )
        self.get_logger().info(
            '=========================================='
        )

    # =========================================================
    # Start scan service
    # =========================================================
    def start_scan_callback(self, request, response):

        # Avoid repeated start calls while already scanning
        if self.scanning_enabled:

            response.success = False
            response.message = (
                'QR scanner is already scanning.'
            )

            self.get_logger().warning(
                'Start request ignored: already scanning.'
            )

            return response

        # Reset scan session
        self.hit_history.clear()
        self.last_process_time = 0.0

        # Enable QR inference
        self.scanning_enabled = True

        # Reset detected state
        detected_msg = Bool()
        detected_msg.data = False
        self.detected_pub.publish(detected_msg)

        response.success = True
        response.message = 'QR scanning started.'

        self.get_logger().info('')
        self.get_logger().info(
            '=========================================='
        )
        self.get_logger().info(
            'SCAN STARTED'
        )
        self.get_logger().info(
            'Waiting for valid QR code...'
        )
        self.get_logger().info(
            '=========================================='
        )

        return response

    # =========================================================
    # CSV
    # =========================================================
    def prepare_log(self):

        folder = os.path.dirname(
            self.log_file
        )

        if folder:
            os.makedirs(
                folder,
                exist_ok=True
            )

        if not os.path.exists(
            self.log_file
        ):

            with open(
                self.log_file,
                'w',
                newline='',
                encoding='utf-8'
            ) as f:

                writer = csv.writer(f)

                writer.writerow([
                    'timestamp',
                    'qr_data',
                    'detector'
                ])

    def save_result(
        self,
        data,
        detector
    ):

        timestamp = datetime.now().isoformat(
            timespec='milliseconds'
        )

        with open(
            self.log_file,
            'a',
            newline='',
            encoding='utf-8'
        ) as f:

            writer = csv.writer(f)

            writer.writerow([
                timestamp,
                data,
                detector
            ])

    # =========================================================
    # WeChat QR detection
    # =========================================================
    def detect_with_wechat(
        self,
        frame
    ):

        results = []

        try:

            decoded_info, points = (
                self.wechat_detector.detectAndDecode(
                    frame
                )
            )

            if decoded_info is None:
                return results

            for i, data in enumerate(
                decoded_info
            ):

                if data is None:
                    continue

                data = str(data).strip()

                if not data:
                    continue

                pts = None

                if points is not None:

                    try:

                        pts = np.asarray(
                            points[i],
                            dtype=np.float32
                        ).reshape(-1, 2)

                    except Exception:
                        pts = None

                results.append(
                    (
                        data,
                        pts,
                        'WeChatQRCode'
                    )
                )

        except Exception as e:

            self.get_logger().warning(
                f'WeChatQRCode error: {e}'
            )

        return results

    # =========================================================
    # OpenCV fallback
    # =========================================================
    def detect_with_opencv(
        self,
        frame
    ):

        results = []

        try:

            data, points, _ = (
                self.opencv_detector.detectAndDecode(
                    frame
                )
            )

            if data and points is not None:

                pts = np.asarray(
                    points,
                    dtype=np.float32
                ).reshape(-1, 2)

                results.append(
                    (
                        data.strip(),
                        pts,
                        'QRCodeDetector'
                    )
                )

        except Exception as e:

            self.get_logger().warning(
                f'OpenCV fallback error: {e}'
            )

        return results

    # =========================================================
    # Combined detector
    # =========================================================
    def detect_qr(
        self,
        frame
    ):

        # Main detector
        results = self.detect_with_wechat(
            frame
        )

        # Fallback only if main detector failed
        if not results:

            results = self.detect_with_opencv(
                frame
            )

        valid_results = []

        for data, points, detector in results:

            # Ignore QR codes not belonging
            # to the coursework format
            if (
                self.valid_pattern.fullmatch(data)
                is None
            ):
                continue

            area = 0.0

            if (
                points is not None
                and len(points) >= 4
            ):

                try:

                    area = abs(
                        cv2.contourArea(
                            points
                        )
                    )

                except Exception:
                    area = 0.0

            valid_results.append(
                (
                    data,
                    points,
                    detector,
                    area
                )
            )

        if not valid_results:
            return None

        # If multiple QR codes are visible,
        # select the largest one.
        valid_results.sort(
            key=lambda item: item[3],
            reverse=True
        )

        return valid_results[0]

    # =========================================================
    # QR confirmation
    # =========================================================
    def confirm_qr(
        self,
        data,
        detector
    ):

        self.last_confirmed = data

        # -----------------------------------------------
        # Publish EXACTLY ONCE
        # -----------------------------------------------
        result_msg = String()
        result_msg.data = data

        self.data_pub.publish(
            result_msg
        )

        # Save result
        self.save_result(
            data,
            detector
        )

        # -----------------------------------------------
        # Stop scanner immediately
        # -----------------------------------------------
        self.scanning_enabled = False

        self.hit_history.clear()

        self.get_logger().info('')
        self.get_logger().info(
            '=========================================='
        )
        self.get_logger().info(
            f'CONFIRMED QR : {data}'
        )
        self.get_logger().info(
            f'DETECTOR     : {detector}'
        )
        self.get_logger().info(
            '/qr/data published once.'
        )
        self.get_logger().info(
            'SCAN FINISHED'
        )
        self.get_logger().info(
            'State         : IDLE'
        )
        self.get_logger().info(
            '=========================================='
        )

    # =========================================================
    # Camera callback
    # =========================================================
    def image_callback(
        self,
        msg
    ):

        # =====================================================
        # IDLE
        #
        # Camera messages still arrive,
        # but NO QR inference is performed.
        # =====================================================
        if not self.scanning_enabled:
            return

        now = time.monotonic()

        # Limit inference rate
        if (
            now - self.last_process_time
            < self.process_period
        ):
            return

        self.last_process_time = now

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'cv_bridge error: {e}'
            )

            return

        debug = frame.copy()

        result = self.detect_qr(
            frame
        )

        # =====================================================
        # QR detected
        # =====================================================
        if result is not None:

            (
                data,
                points,
                detector,
                area
            ) = result

            detected_msg = Bool()
            detected_msg.data = True

            self.detected_pub.publish(
                detected_msg
            )

            # Save hit
            self.hit_history.append(
                (
                    now,
                    data,
                    detector
                )
            )

            # Remove old hits
            while self.hit_history:

                age = (
                    now
                    - self.hit_history[0][0]
                )

                if age > self.confirm_window:
                    self.hit_history.popleft()
                else:
                    break

            # Count matching hits
            hits = sum(
                1
                for _, code, _
                in self.hit_history
                if code == data
            )

            # Draw QR boundary
            if (
                points is not None
                and len(points) >= 4
            ):

                polygon = (
                    points.astype(np.int32)
                    .reshape((-1, 1, 2))
                )

                cv2.polylines(
                    debug,
                    [polygon],
                    True,
                    (0, 255, 0),
                    3
                )

            cv2.putText(
                debug,
                f'QR: {data}',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2
            )

            cv2.putText(
                debug,
                f'Detector: {detector}',
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            cv2.putText(
                debug,
                f'Confirm: {hits}/{self.confirm_hits}',
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            # -----------------------------------------------
            # Confirm QR
            # -----------------------------------------------
            if hits >= self.confirm_hits:

                self.confirm_qr(
                    data,
                    detector
                )

        # =====================================================
        # No QR detected
        # =====================================================
        else:

            detected_msg = Bool()
            detected_msg.data = False

            self.detected_pub.publish(
                detected_msg
            )

            cv2.putText(
                debug,
                'Scanning for QR...',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        # =====================================================
        # Debug image
        # Only published while SCANNING
        # =====================================================
        try:

            debug_msg = (
                self.bridge.cv2_to_imgmsg(
                    debug,
                    encoding='bgr8'
                )
            )

            debug_msg.header = msg.header

            self.debug_pub.publish(
                debug_msg
            )

        except Exception as e:

            self.get_logger().error(
                f'Debug image error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = QRDetectorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
