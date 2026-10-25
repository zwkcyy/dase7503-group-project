#!/usr/bin/env python3

import csv
import math
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
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

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
        self.declare_parameter('evidence_dir', '')
        self.declare_parameter('evidence_source', 'unspecified')
        self.declare_parameter('opencv_threads', 1)
        opencv_threads = self.get_parameter('opencv_threads').value
        if isinstance(opencv_threads, bool) or not isinstance(opencv_threads, int) or not 1 <= opencv_threads <= 4:
            raise ValueError('opencv_threads must be an integer from 1 to 4')
        # Avoid oversubscribing the shared Pi/WSL CPU alongside Nav2 and the EKF.
        cv2.setNumThreads(opencv_threads)

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

        if not math.isfinite(self.process_hz) or self.process_hz <= 0.0:
            raise ValueError('process_hz must be finite and greater than zero')
        if self.confirm_hits < 1:
            raise ValueError('confirm_hits must be at least 1')
        if not math.isfinite(self.confirm_window) or self.confirm_window <= 0.0:
            raise ValueError('confirm_window must be finite and greater than zero')
        if (self.confirm_hits - 1) / self.process_hz > self.confirm_window:
            raise ValueError('confirm_window is too short for confirm_hits and process_hz')

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

        # Primary detector. Keep the OpenCV fallback usable when the optional
        # WeChat models or opencv-contrib module are not installed.
        model_paths = (
            detect_proto,
            detect_model,
            sr_proto,
            sr_model,
        )
        wechat_factory = getattr(
            cv2,
            'wechat_qrcode_WeChatQRCode',
            None
        )
        self.wechat_detector = None

        if all(os.path.isfile(path) for path in model_paths) and wechat_factory:
            try:
                self.wechat_detector = wechat_factory(*model_paths)
            except Exception as e:
                self.get_logger().warning(
                    f'WeChatQRCode unavailable; using OpenCV fallback: {e}'
                )
        else:
            missing = [path for path in model_paths if not os.path.isfile(path)]
            if missing:
                self.get_logger().warning(
                    'WeChatQRCode models are missing; using OpenCV fallback. '
                    'Run scripts/download_qr_models.sh to enable the primary detector.'
                )
            elif not wechat_factory:
                self.get_logger().warning(
                    'This OpenCV build has no WeChatQRCode module; using OpenCV fallback.'
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

        self.last_process_time = -math.inf
        self.scan_started_ns = 0
        self.last_frame_stamp_ns = None

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
        self.stop_scan_service = self.create_service(
            Trigger,
            '/qr/stop_scan',
            self.stop_scan_callback
        )

        # =====================================================
        # Camera subscriber
        # =====================================================
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        try:
            self.prepare_log()
        except OSError as e:
            self.get_logger().warning(f'QR log unavailable; scanning will still work: {e}')

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
            'Primary  : ' + ('WeChatQRCode CNN + SR' if self.wechat_detector
                            is not None else 'OpenCV QRCodeDetector')
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

            response.success = True
            response.message = (
                'QR scanner is already scanning.'
            )

            self.get_logger().debug(
                'Start request acknowledged: already scanning.'
            )

            return response

        # Reset scan session
        self.hit_history.clear()
        self.last_process_time = -math.inf
        self.scan_started_ns = self.get_clock().now().nanoseconds
        self.last_frame_stamp_ns = None
        self.last_confirmed = None

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

    def stop_scan_callback(self, request, response):

        # Idempotent: STOP succeeds even if a result already returned us to IDLE.
        self.scanning_enabled = False
        self.hit_history.clear()

        detected_msg = Bool()
        detected_msg.data = False
        self.detected_pub.publish(detected_msg)

        response.success = True
        response.message = 'QR scanning stopped.'
        self.get_logger().info('QR scanning stopped.')
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

        if self.wechat_detector is None:
            return results

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
                        if not np.isfinite(pts).all():
                            pts = None

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
    def detect_with_opencv(self, frame):
        results = []
        # Multi-code detection avoids losing a coursework marker when another
        # (unrelated) QR is also visible. Older OpenCV builds retain the single
        # detector path below.
        multi = getattr(self.opencv_detector, 'detectAndDecodeMulti', None)
        if multi is not None:
            try:
                found, decoded, points, _ = multi(frame)
                if found and points is not None:
                    for data, polygon in zip(decoded, points):
                        if data and self.valid_pattern.fullmatch(data.strip()):
                            pts = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
                            if np.isfinite(pts).all():
                                results.append((data.strip(), pts, 'QRCodeDetector'))
            except Exception as e:
                self.get_logger().debug(f'OpenCV multi-QR detection failed: {e}')
        if results:
            return results
        try:
            data, points, _ = self.opencv_detector.detectAndDecode(frame)
            if data:
                pts = None
                if points is not None:
                    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
                    if not np.isfinite(pts).all():
                        pts = None
                results.append((data.strip(), pts, 'QRCodeDetector'))
        except Exception as e:
            self.get_logger().warning(f'OpenCV fallback error: {e}')
        return results

    # =========================================================
    # Combined detector
    # =========================================================
    def detect_qr(
        self,
        frame
    ):

        # Fall back when WeChat detects no coursework marker. A random QR code
        # must not suppress a valid START/END/rack result from the fallback.
        results = [
            result
            for result in self.detect_with_wechat(frame)
            if self.valid_pattern.fullmatch(result[0]) is not None
        ]

        if not results:
            results = [
                result
                for result in self.detect_with_opencv(frame)
                if self.valid_pattern.fullmatch(result[0]) is not None
            ]

        # Bound the retry cost and restore polygons to the original image frame.
        if not results and 0 < max(frame.shape[:2]) < 240:
            scale = min(4.0, 480.0 / max(frame.shape[:2]))
            enlarged = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            for data, points, detector in self.detect_with_opencv(enlarged):
                if self.valid_pattern.fullmatch(data):
                    results.append((data, points / scale if points is not None else None,
                                    detector + '+resize'))

        valid_results = []

        for data, points, detector in results:

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
        # Stop scanner immediately
        # -----------------------------------------------
        self.scanning_enabled = False

        self.hit_history.clear()

        # Publish before disk I/O so local logging cannot delay the mission.
        result_msg = String()
        result_msg.data = data
        self.data_pub.publish(result_msg)

        try:
            self.save_result(data, detector)
        except OSError as e:
            self.get_logger().error(f'Could not save QR result: {e}')

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
    def image_callback(self, msg):
        if not self.scanning_enabled:
            return
        now = time.monotonic()
        if now - self.last_process_time < self.process_period:
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns:
            # Ignore queued frames from before this scan and repeated frames.
            # Zero stamps remain supported for image sources without timestamps.
            if stamp_ns < self.scan_started_ns:
                return
            if self.last_frame_stamp_ns is not None and stamp_ns <= self.last_frame_stamp_ns:
                return
            self.last_frame_stamp_ns = stamp_ns
        self.last_process_time = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        result = self.detect_qr(frame)
        detected_msg = Bool()
        detected_msg.data = result is not None
        self.detected_pub.publish(detected_msg)
        hit_time = time.monotonic()
        while self.hit_history and hit_time - self.hit_history[0][0] > self.confirm_window:
            self.hit_history.popleft()
        hits = 0
        if result is not None:
            data, points, detector, _ = result
            self.hit_history.append((hit_time, data, detector))
            hits = sum(code == data for _, code, _ in self.hit_history)
            if hits >= self.confirm_hits:
                self.confirm_qr(data, detector)
                directory = self.get_parameter('evidence_dir').value
                if directory:
                    try:
                        from robot_vision.evidence import save_confirmation
                        save_confirmation(directory, data, detector, frame, stamp_ns,
                                          self.get_parameter('evidence_source').value)
                    except Exception as exc:
                        self.get_logger().error(f'Could not save QR evidence: {exc}')

        # Save Pi CPU and image copies when no viewer is connected. Debug drawing
        # must never prevent a confirmed result from reaching the mission.
        if self.debug_pub.get_subscription_count() == 0:
            return
        try:
            debug = frame.copy()
            if result is not None:
                data, points, detector, _ = result
                if points is not None and len(points) >= 4:
                    polygon = points.astype(np.int32).reshape((-1, 1, 2))
                    cv2.polylines(debug, [polygon], True, (0, 255, 0), 3)
                lines = [f'QR: {data}', f'Detector: {detector}',
                         f'Confirm: {hits}/{self.confirm_hits}']
            else:
                lines = ['Scanning for QR...']
            for i, line in enumerate(lines):
                cv2.putText(debug, line, (20, 40 + 35 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug image error: {e}')


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = QRDetectorNode()

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:

        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
