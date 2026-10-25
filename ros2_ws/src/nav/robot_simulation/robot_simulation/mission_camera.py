"""Feed task-dependent QR images to the real scanner, never publish QR results."""
import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot_bringup.configuration import mission_parameters
from .planar_base import require_isolated_domain
from .qr_images import qr_image


class MissionCamera(Node):
    def __init__(self):
        super().__init__('mission_camera')
        require_isolated_domain()
        self.declare_parameter('mission_file', '')
        self.declare_parameter('scenario', 'success')
        self.scenario = self.get_parameter('scenario').value
        if self.scenario not in ('success', 'wrong_qr', 'missing_qr'):
            raise ValueError('Image scenario must be success, wrong_qr or missing_qr')
        params = mission_parameters(self.get_parameter('mission_file').value)
        self.codes = params['rack_qr_codes']
        self.frames = {text: cv2.cvtColor(qr_image(text, 320), cv2.COLOR_BGR2GRAY)
                       for text in ['START', 'END', 'RACKA_WRONG', *self.codes]}
        self.blank = np.full((320, 320), 255, np.uint8)
        self.state, self.index = 'WAIT_START', 0
        self.last_payload = object()
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.create_subscription(String, '/mission/status', self.status, 10)
        self.create_timer(.1, self.tick)
        self.get_logger().warning('SYNTHETIC CAMERA: task-driven QR images; no physical camera visibility model')

    def status(self, msg):
        data = json.loads(msg.data)
        self.state, self.index = data['state'], data['rack_index']

    def tick(self):
        payload = None
        if self.state == 'WAIT_START':
            payload = 'START'
        elif self.state == 'WAIT_RACK_QR' and 0 <= self.index < len(self.codes):
            payload = self.codes[self.index]
        elif self.state == 'VERIFY_DESTINATION':
            payload = 'END'
        if self.state != 'WAIT_START':
            if self.scenario == 'wrong_qr' and payload:
                payload = 'RACKA_WRONG'
            elif self.scenario == 'missing_qr':
                payload = None
        if payload != self.last_payload:
            self.get_logger().info(f'Synthetic image payload: {payload!r}; mission={self.state}')
            self.last_payload = payload
        msg = self.bridge.cv2_to_imgmsg(self.frames.get(payload, self.blank), encoding='mono8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
