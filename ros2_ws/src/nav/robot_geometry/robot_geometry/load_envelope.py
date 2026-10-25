"""Change both Nav2 footprints before acknowledging a fresh lift completion."""

import copy
import json
import math
import time

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException

from .profile import load_profile, polygon, inside_filter, transform_point


class LoadEnvelope(Node):
    def __init__(self, **node_options):
        super().__init__('load_envelope', **node_options)
        self.declare_parameter('geometry_file', '')
        self.declare_parameter('allow_unconfirmed_geometry', False)
        self.declare_parameter('initial_loaded', True)
        self.profile = load_profile(self.get_parameter('geometry_file').value,
                                    self.get_parameter('allow_unconfirmed_geometry').value)
        self.loaded = self.get_parameter('initial_loaded').value
        self.busy = False
        self.failed = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.group = ReentrantCallbackGroup()
        self.costmap_clients = [self.create_client(SetParameters, f'/{name}/{name}/set_parameters',
                                          callback_group=self.group)
                        for name in ('local_costmap', 'global_costmap')]
        self.service = self.create_service(SetBool, '/nav/set_loaded', self.set_loaded,
                                           callback_group=self.group)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, '/nav/loaded', qos)
        self.state_pub.publish(Bool(data=self.loaded))
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', sensor_qos)
        self.create_subscription(LaserScan, '/scan', self.filter_scan, sensor_qos)
        self.get_logger().info(f'Load envelope ready; initial_loaded={self.loaded}')

    async def set_loaded(self, request, response):
        if self.busy or self.failed:
            response.message = 'Envelope update busy or previously failed; restart after inspection'
            return response
        if any(not client.service_is_ready() for client in self.costmap_clients):
            response.message = 'Both Nav2 costmap parameter services must be ready'
            return response
        self.busy = True
        futures = []
        deadline = time.monotonic() + 1.5
        waiter = Future()
        timer = None
        try:
            param = Parameter('footprint', value=json.dumps(polygon(self.profile, request.data)))
            for client in self.costmap_clients:
                futures.append(client.call_async(SetParameters.Request(parameters=[param.to_parameter_msg()])))

            def poll():
                if not waiter.done() and (all(f.done() for f in futures) or time.monotonic() >= deadline):
                    waiter.set_result(None)

            timer = self.create_timer(.02, poll, callback_group=self.group)
            await waiter
            if any(not f.done() for f in futures):
                raise RuntimeError('Timed out updating costmap footprints; state is uncertain')
            for future in futures:
                results = future.result().results
                if len(results) != 1 or not results[0].successful:
                    raise RuntimeError('A costmap rejected the new footprint')
            # Filter changes only after both costmaps acknowledge the new geometry.
            self.loaded = bool(request.data)
            self.state_pub.publish(Bool(data=self.loaded))
            response.success = True
            response.message = f'Both costmaps and self-filter loaded={self.loaded}'
            self.get_logger().info(response.message)
        except Exception as exc:
            self.failed = True
            response.message = str(exc)
            self.get_logger().error(response.message)
        finally:
            if timer is not None:
                self.destroy_timer(timer)
            for client, future in zip(self.costmap_clients, futures):
                if not future.done():
                    client.remove_pending_request(future)
                    future.cancel()
            self.busy = False
        return response

    def filter_scan(self, msg):
        try:
            transform = self.tf.lookup_transform('base_link', msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp)).transform
            result = copy.deepcopy(msg)
            for index, distance in enumerate(msg.ranges):
                if not math.isfinite(distance):
                    continue
                angle = msg.angle_min + index * msg.angle_increment
                point = transform_point(transform, distance*math.cos(angle), distance*math.sin(angle))
                if inside_filter(self.profile, self.loaded, *point):
                    # NaN removes a self return without falsely clearing free space behind it.
                    result.ranges[index] = float('nan')
            self.scan_pub.publish(result)
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(f'Scan skipped: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LoadEnvelope()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
