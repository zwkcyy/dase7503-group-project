"""Feed task-dependent QR images to the real scanner, never publish QR results."""
import json
import math
import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
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
        self.params = params
        self.pose = None
        self.pose_at = 0.
        self.create_subscription(PoseStamped, '/simulation/pose', self.position, 10)
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

    def position(self, msg):
        self.pose, self.pose_at = msg.pose, time.monotonic()

    def visible(self):
        # Position-gated synthetic fixture; not an optical/lighting simulation.
        if not self.params.get('use_docking'):
            return True
        if self.pose is None or time.monotonic()-self.pose_at > .5:
            return False
        if self.state == 'WAIT_START':
            return 2.42 < self.pose.position.x < 3.02 and .6 < self.pose.position.y < 1.4
        if not 0 <= self.index < len(self.codes):
            return False
        prefix = 'rack' if self.state == 'WAIT_RACK_QR' else 'placement'
        x, y, theta = [self.params[prefix+'_'+key][self.index] for key in ('x_m', 'y_m', 'yaw_rad')]
        if prefix == 'rack':
            x -= self.params['docking_distance_m']*math.cos(theta)
            y -= self.params['docking_distance_m']*math.sin(theta)
        q = self.pose.orientation
        heading = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        return (math.hypot(x-self.pose.position.x, y-self.pose.position.y) < .10
                and abs(math.atan2(math.sin(theta-heading), math.cos(theta-heading))) < .15)

    def tick(self):
        payload = None
        if self.state == 'WAIT_START':
            payload = 'START'
        elif self.state == 'WAIT_RACK_QR' and 0 <= self.index < len(self.codes):
            payload = self.codes[self.index]
        elif self.state == 'VERIFY_DESTINATION':
            payload = 'END'
        if payload and not self.visible():
            payload = None
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
