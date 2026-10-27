"""Slow straight docking using TF localisation and fresh laser observations.

This internal action runs only after Nav2 has finished its transit goal. It
never reads simulator ground truth and never turns while between rack legs.
"""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from tf2_ros import Buffer, TransformListener, TransformException
from robot_geometry.profile import load_profile


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        self.declare_parameter('geometry_file', '')
        self.profile = load_profile(self.get_parameter('geometry_file').value, True)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.loaded = False
        self.scan = None
        self.scan_at = 0.
        self.active = None
        self.reserved = False
        self.group = ReentrantCallbackGroup()
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Bool, '/nav/loaded', lambda m: setattr(self, 'loaded', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.server = ActionServer(self, NavigateToPose, 'dock_to_pose', self.execute,
                                  goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                  callback_group=self.group)
        self.create_timer(.05, self.tick, callback_group=self.group)

    def accept(self, request):
        p = request.pose
        q = p.pose.orientation
        values = [p.pose.position.x, p.pose.position.y, q.x, q.y, q.z, q.w]
        if (self.reserved or p.header.frame_id != 'map' or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in values[2:])-1.) > .01 or abs(q.x)+abs(q.y) > .001):
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    async def execute(self, handle):
        future = Future()
        self.active = (handle, future, time.monotonic())
        self.settled = None
        await future
        return NavigateToPose.Result()

    def finish(self, state):
        self.pub.publish(Twist())
        handle, future, _ = self.active
        getattr(handle, state)()
        self.active = None
        self.reserved = False
        future.set_result(None)

    def tick(self):
        if not self.active:
            return
        handle, _, started = self.active
        now = time.monotonic()
        if handle.is_cancel_requested:
            self.finish('canceled')
            return
        if now-started > 20.:
            self.get_logger().error('Docking deadline exceeded')
            self.finish('abort')
            return
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            age = (self.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9
            scan_age = (self.get_clock().now().nanoseconds/1e9-self.scan.header.stamp.sec-
                        self.scan.header.stamp.nanosec/1e9) if self.scan else float('inf')
            if self.scan is None or now-self.scan_at > .4 or not -.1 <= scan_age <= .4 or age > .4 or age < -.6:
                self.pub.publish(Twist())
                self.settled = None
                return
            p, q = tf.transform.translation, tf.transform.rotation
            goal = handle.request.pose.pose
            target_yaw, angle = yaw(goal.orientation), yaw(q)
            dx, dy = goal.position.x-p.x, goal.position.y-p.y
            forward = math.cos(target_yaw)*dx+math.sin(target_yaw)*dy
            lateral = -math.sin(target_yaw)*dx+math.cos(target_yaw)*dy
            error = wrap(target_yaw-angle)
            if math.hypot(dx, dy) < .008 and abs(error) < .018:
                self.pub.publish(Twist())
                if self.settled is None:
                    self.settled = now
                if now-self.settled > .35:
                    self.finish('succeed')
                return
            self.settled = None
            cmd = Twist()
            # Align outside the rack, then hold heading during insertion/exit.
            # A large error close to the target is unsafe to correct in place.
            if abs(error) > .12 and abs(forward) < .24:
                self.get_logger().error('Docking heading error inside the rack corridor')
                self.finish('abort')
                return
            cmd.angular.z = max(-.18, min(.18, 1.5*error))
            if abs(error) < .04:
                speed = max(-.13, min(.13, 1.6*forward))
                side = max(-.04, min(.04, 1.6*lateral))
                # Avoid entering the narrow opening before lateral alignment.
                if forward > .24 and abs(lateral) > .012:
                    speed = 0.
                cmd.linear.x = math.cos(error)*speed-math.sin(error)*side
                cmd.linear.y = math.sin(error)*speed+math.cos(error)*side
            if self.obstructed(cmd):
                self.pub.publish(Twist())
                return
            self.pub.publish(cmd)
        except TransformException:
            self.pub.publish(Twist())

    def obstructed(self, cmd):
        """Stop if a fresh laser point enters the braking envelope (0.35 s)."""
        tf = self.buffer.lookup_transform('base_link', self.scan.header.frame_id, rclpy.time.Time())
        t, theta = tf.transform.translation, yaw(tf.transform.rotation)
        key = 'loaded_bounds_mm' if self.loaded else 'empty_bounds_mm'
        x0, y0, x1, y1 = [v/1000 for v in self.profile[key]]
        for i, distance in enumerate(self.scan.ranges):
            if not math.isfinite(distance) or not self.scan.range_min < distance < self.scan.range_max:
                continue
            angle = self.scan.angle_min+i*self.scan.angle_increment+theta
            px, py = t.x+distance*math.cos(angle), t.y+distance*math.sin(angle)
            for horizon in (.0, .175, .35):
                a = -cmd.angular.z*horizon
                x, y = px-cmd.linear.x*horizon, py-cmd.linear.y*horizon
                u, v = math.cos(a)*x-math.sin(a)*y, math.sin(a)*x+math.cos(a)*y
                if x0-.008 < u < x1+.008 and y0-.008 < v < y1+.008:
                    return True
        return False

    def destroy_node(self):
        self.server.destroy()
        self.listener.unregister()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
