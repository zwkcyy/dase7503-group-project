"""Publish fresh ROS frames from a local image/video or a generated QR marker."""

import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from .planar_base import require_isolated_domain


class ImageReplay(Node):
    def __init__(self):
        super().__init__('image_replay')
        require_isolated_domain()
        self.declare_parameter('input_file', '')
        self.declare_parameter('payload', 'START')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('loop', True)
        fps = float(self.get_parameter('fps').value)
        if not math.isfinite(fps) or not 0 < fps <= 30:
            raise ValueError('fps must be in (0, 30]')
        self.bridge, self.capture, self.frame, self.payload = CvBridge(), None, None, None
        filename = self.get_parameter('input_file').value
        self.generated = not filename
        if filename:
            if not Path(filename).expanduser().is_file():
                raise ValueError('input_file must be an existing local image or video')
            filename = str(Path(filename).expanduser())
            self.frame = cv2.imread(filename)
            if self.frame is None:
                self.capture = cv2.VideoCapture(filename)
                if not self.capture.isOpened():
                    raise ValueError('Could not open input_file')
        self.publisher = self.create_publisher(Image, '/camera/image_raw',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(1/fps, self.tick)
        self.get_logger().info('IMAGE REPLAY: timestamps are replay times, not camera capture times')

    def tick(self):
        if self.generated:
            payload = self.get_parameter('payload').value
            if payload != self.payload:
                if not isinstance(payload, str) or not 1 <= len(payload) <= 256:
                    self.get_logger().error('payload needs 1-256 characters', throttle_duration_sec=2.)
                    return
                qr = cv2.QRCodeEncoder_create().encode(payload)
                qr = np.pad(qr, 4, constant_values=255)
                self.frame = cv2.cvtColor(cv2.resize(qr, (480, 480), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
                self.payload = payload
        if self.capture:
            ok, frame = self.capture.read()
            if not ok and self.get_parameter('loop').value:
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.capture.read()
            if not ok:
                self.timer.cancel()
                self.get_logger().info('Replay ended')
                return
            self.frame = frame
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageReplay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            if node.capture:
                node.capture.release()
            node.destroy_node()
        rclpy.try_shutdown()
