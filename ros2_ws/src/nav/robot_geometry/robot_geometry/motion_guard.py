"""Final command gate: stale sensors/TF, mission holds and latched operator stop.

This complements Nav2 obstacle checking; it is not a safety-rated controller.
The EC watchdog must independently stop the hardware if this process dies.
"""
import json
import math
import time
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException


MOVING = {'NAV_TO_RACK', 'NAV_TO_DESTINATION', 'DOCK_RACK', 'DOCK_DESTINATION',
          'EXIT_RACK', 'EXIT_DESTINATION'}
HOLDING = {'WAIT_START', 'WAIT_RACK_QR', 'LIFT_RACK', 'VERIFY_DESTINATION',
           'LOWER_RACK', 'COMPLETE', 'ERROR'}


def sample_fresh(received, stamp, now, ros_now, limit=.5):
    return (received is not None and stamp > 0 and math.isfinite(stamp)
            and 0 <= now-received <= limit and -.1 <= ros_now-stamp <= limit)


class MotionGuard(Node):
    def __init__(self):
        super().__init__('motion_guard')
        self.command = None
        self.command_at = 0.
        self.raw_command_at = 0.
        self.samples = {}
        self.phase, self.phase_at = None, 0.
        self.latched = False
        self.reason = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/nav/motion_safety', 10)
        self.create_subscription(Twist, '/cmd_vel_requested', self.on_command, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda _: setattr(self, 'raw_command_at', time.monotonic()), 10)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom/wheel', lambda m: self.on_odom(m, 'wheel'), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/status', self.on_phase, 10)
        self.create_service(Trigger, '/nav/emergency_stop', self.stop)
        self.create_timer(.025, self.tick)

    def on_command(self, msg):
        self.command, self.command_at = msg, time.monotonic()

    def remember(self, key, msg, valid):
        self.samples[key] = (time.monotonic(), msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, valid)

    def on_scan(self, msg):
        self.remember('laser', msg, bool(msg.ranges) and any(math.isfinite(x) and
                      msg.range_min <= x <= msg.range_max for x in msg.ranges))

    def on_odom(self, msg, key='odometry'):
        t = msg.twist.twist
        self.remember(key, msg, msg.child_frame_id == 'base_link' and
                      all(math.isfinite(x) for x in (t.linear.x, t.linear.y, t.angular.z)))

    def on_imu(self, msg):
        self.remember('imu', msg, bool(msg.header.frame_id) and math.isfinite(msg.angular_velocity.z))

    def on_phase(self, msg):
        try:
            phase = json.loads(msg.data)['state']
            if not isinstance(phase, str) or phase not in MOVING | HOLDING:
                raise ValueError('Invalid mission phase')
            self.phase = phase
            self.phase_at = time.monotonic()
            if self.phase == 'ERROR':
                self.latched = True
        except (ValueError, KeyError, TypeError):
            self.latched = True

    def stop(self, request, response):
        self.latched = True
        self.pub.publish(Twist())
        response.success, response.message = True, 'Motion stop latched; restart only after inspection'
        return response

    def blocked_reason(self):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds/1e9
        if self.latched:
            return 'latched_stop'
        if self.phase is not None and (self.phase not in MOVING or now-self.phase_at > .75):
            return 'mission_hold_or_stale_status'
        if self.command is None or now-self.command_at > .3:
            return 'stale_command'
        if now-self.raw_command_at > .3:
            return 'stale_controller_command'
        for key in ('laser', 'odometry', 'wheel', 'imu'):
            received, stamp, valid = self.samples.get(key, (None, 0., False))
            if not valid or not sample_fresh(received, stamp, now, ros_now):
                return 'stale_or_invalid_'+key
        for parent, child in (('map', 'odom'), ('odom', 'base_link')):
            try:
                tf = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
                stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
                # AMCL intentionally future-dates map->odom by transform_tolerance.
                if stamp <= 0 or not -.6 <= ros_now-stamp <= .5:
                    return 'stale_transform'
            except TransformException:
                return 'missing_transform'
        t = self.command
        if not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.angular.z)):
            return 'nonfinite_command'
        return ''

    def tick(self):
        reason = self.blocked_reason()
        if reason != self.reason:
            self.reason = reason
            self.status_pub.publish(String(data=json.dumps({'blocked': bool(reason), 'reason': reason})))
        # Keep sending zeros while held; do not replay the last nonzero command.
        self.pub.publish(Twist() if reason else self.command)

    def destroy_node(self):
        self.listener.unregister()
        return super().destroy_node()


def main(args=None):
    # Keep the ROS context alive long enough to send final zero commands.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = False
    def stop_requested(signum, frame):
        nonlocal stopping
        stopping = True
    previous = {sig: signal.signal(sig, stop_requested) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = MotionGuard()
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.latched = True
            for _ in range(5):
                node.pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
