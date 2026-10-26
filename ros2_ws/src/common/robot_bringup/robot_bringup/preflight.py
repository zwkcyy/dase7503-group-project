"""Observe readiness without sending motion, lift or scanner commands."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger, SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from robot_geometry.profile import load_profile
from .configuration import mission_parameters
from .health import freshness


class Preflight(Node):
    def __init__(self, mode='full', max_age=2.0, image_topic='/camera/image_raw', mission=None):
        super().__init__('nav_vi_preflight')
        self.mode, self.max_age, self.image_topic = mode, max_age, image_topic
        self.mission = mission or {}
        self.samples = {}
        self.topic_issues = {}
        self.service_clients = {}
        self.lifecycle = {}
        self.last_poll = 0.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        topics = {}
        if mode in ('vision', 'full'):
            topics[image_topic] = Image
            for key in ('qr_start_service', 'qr_stop_service'):
                name = self.mission.get(key, '/qr/' + ('start_scan' if key == 'qr_start_service' else 'stop_scan'))
                self.service_clients[name] = self.create_client(Trigger, name)
        self.nav = None
        if mode in ('navigation', 'full'):
            topics.update({'/scan_filtered': LaserScan, '/odom/wheel': Odometry,
                           '/odometry/filtered': Odometry, '/imu/data_raw': Imu})
            self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.service_clients['/nav/set_loaded'] = self.create_client(SetBool, '/nav/set_loaded')
            for name in ('local_costmap', 'global_costmap'):
                service = f'/{name}/{name}/set_parameters'
                self.service_clients[service] = self.create_client(SetParameters, service)
            for name in ('amcl', 'controller_server', 'planner_server', 'bt_navigator'):
                self.lifecycle[name] = {'client': self.create_client(GetState, f'/{name}/get_state'),
                                        'future': None, 'sent': 0., 'checked': 0., 'state': None}
        for name, message in topics.items():
            self.samples[name] = (None, 0.)
            self.create_subscription(message, name, lambda msg, key=name: self.receive(key, msg), qos_profile_sensor_data)

    def receive(self, key, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.samples[key] = (time.monotonic(), stamp)
        reason = ''
        if not msg.header.frame_id:
            reason = 'missing frame_id'
        elif isinstance(msg, Image) and (msg.width <= 0 or msg.height <= 0 or not msg.encoding
                                        or msg.step <= 0 or len(msg.data) < msg.step * msg.height):
            reason = 'empty image'
        elif isinstance(msg, LaserScan) and (not msg.ranges or not any(math.isfinite(v) for v in msg.ranges)):
            reason = 'no finite laser returns'
        elif isinstance(msg, Odometry) and msg.child_frame_id != 'base_link':
            reason = 'odometry child_frame_id must be base_link'
        self.topic_issues[key] = reason

    def poll_lifecycle(self, now):
        for item in self.lifecycle.values():
            future = item['future']
            if future and future.done():
                try:
                    item['state'] = future.result().current_state.id
                    item['checked'] = now
                except Exception:
                    item['state'] = None
                item['future'] = None
            elif future and now - item['sent'] > 2.:
                item['client'].remove_pending_request(future)
                future.cancel()
                item['future'] = None
                item['state'] = None
            if item['future'] is None and now - item['sent'] > .5 and item['client'].service_is_ready():
                item['future'] = item['client'].call_async(GetState.Request())
                item['sent'] = now

    def snapshot(self):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds / 1e9
        self.poll_lifecycle(now)
        issues = {}
        for key, (received, stamp) in self.samples.items():
            reason = freshness(received, stamp, now, ros_now, self.max_age) or self.topic_issues.get(key, '')
            if reason:
                issues[key] = reason
        for name, client in self.service_clients.items():
            if not client.service_is_ready():
                issues[name] = 'service unavailable'
        for name, item in self.lifecycle.items():
            if item['state'] != 3 or now - item['checked'] > 3.:
                issues[name] = 'lifecycle not confirmed ACTIVE'
        if self.nav:
            if not self.nav.server_is_ready():
                issues['/navigate_to_pose'] = 'action unavailable'
            for parent, child in (('map', 'odom'), ('odom', 'base_link'),
                                  ('base_link', 'laser'), ('base_link', 'imu_link')):
                try:
                    transform = self.buffer.lookup_transform(parent, child, Time())
                    if parent in ('map', 'odom'):
                        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
                        # AMCL deliberately timestamps its transform into the future.
                        if stamp <= 0 or not -2. <= ros_now-stamp <= self.max_age:
                            issues[f'TF {parent}->{child}'] = 'stale transform or clock mismatch'
                except TransformException:
                    issues[f'TF {parent}->{child}'] = 'transform unavailable'
        if self.mode == 'full':
            try:
                self.buffer.lookup_transform('base_link', 'camera_optical_frame', Time())
            except TransformException:
                issues['TF base_link->camera_optical_frame'] = 'transform unavailable'
            command = self.mission.get('lift_command_topic', '/lift_cmd')
            state = self.mission.get('lift_state_topic', '/lift_state')
            receivers = self.get_subscriptions_info_by_topic(command)
            publishers = self.get_publishers_info_by_topic(state)
            if not any(info.topic_type == 'std_msgs/msg/Int8' for info in receivers):
                issues[command] = 'no Int8 lift command receiver'
            if not any(info.topic_type == 'std_msgs/msg/Int8' and
                       info.qos_profile.reliability == ReliabilityPolicy.RELIABLE for info in publishers):
                issues[state] = 'no reliable Int8 lift feedback publisher (no command sent by this check)'
        if self.mode in ('vision', 'full'):
            result_topic = self.mission.get('qr_topic', '/qr/data')
            publishers = self.get_publishers_info_by_topic(result_topic)
            if (len(publishers) != 1 or publishers[0].topic_type != 'std_msgs/msg/String'
                    or publishers[0].qos_profile.reliability != ReliabilityPolicy.RELIABLE):
                issues[result_topic] = 'expected exactly one QR result publisher; check duplicate/mock scanners'
        return {'ready': not issues, 'mode': self.mode, 'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                'issues': issues, 'checked_topics': list(self.samples), 'read_only': True}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('full', 'navigation', 'vision'), default='full')
    parser.add_argument('--timeout', type=float, default=40.)
    parser.add_argument('--max-age', type=float, default=2.)
    parser.add_argument('--stable-for', type=float, default=1.)
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--geometry-file')
    parser.add_argument('--mission-file')
    parser.add_argument('--allow-unconfirmed', action='store_true', help='Explicit simulation use only')
    parser.add_argument('--empty-only', action='store_true', help='Unloaded commissioning; never validates a delivery mission')
    parser.add_argument('--output')
    opts = parser.parse_args(args)
    if any(not math.isfinite(v) or v <= 0 for v in (opts.timeout, opts.max_age, opts.stable_for)):
        parser.error('timeouts and stability duration must be finite and positive')
    report = {'ready': False, 'issues': {}}
    node = None
    code = 2
    try:
        if opts.geometry_file:
            load_profile(opts.geometry_file, opts.allow_unconfirmed, opts.empty_only)
        if opts.empty_only and (opts.mode != 'navigation' or opts.mission_file):
            raise ValueError('--empty-only requires --mode navigation and no mission file')
        params = mission_parameters(opts.mission_file) if opts.mission_file else None
        rclpy.init(args=[])
        node = Preflight(opts.mode, opts.max_age, opts.image_topic, params)
        deadline, stable_at, last_text, last_print = time.monotonic()+opts.timeout, None, '', 0.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            report = node.snapshot()
            now = time.monotonic()
            stable_at = (stable_at or now) if report['ready'] else None
            text = json.dumps(report, ensure_ascii=False)
            if text != last_text and now-last_print > 1.:
                print(text, flush=True)
                last_text, last_print = text, now
            if stable_at and now-stable_at >= opts.stable_for:
                code = 0
                break
        report['passed'] = code == 0
        if code:
            report['ready'] = False
            report['issues'].setdefault('readiness', 'readiness did not remain stable before timeout')
    except (KeyboardInterrupt, ExternalShutdownException):
        report = {'ready': False, 'passed': False, 'issues': {'interrupted': 'check cancelled'}}
        code = 130
    except Exception as exc:
        report = {'ready': False, 'passed': False, 'issues': {'configuration': str(exc)}}
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(report, ensure_ascii=False), flush=True)
    if opts.output:
        path = Path(opts.output).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    raise SystemExit(code)
