"""Simulated wheel/IMU/laser inputs; EKF remains the only odom TF publisher."""

import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robot_geometry.profile import load_profile, numbers
from .world import advance, collides, segments, ray_distance


def require_isolated_domain():
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        raise RuntimeError('Simulation requires a nonzero ROS_DOMAIN_ID (use 75)')


class PlanarBase(Node):
    def __init__(self):
        super().__init__('planar_base')
        require_isolated_domain()
        self.declare_parameter('world_file', '')
        self.declare_parameter('geometry_file', '')
        with open(self.get_parameter('world_file').value) as stream:
            self.world = yaml.safe_load(stream)
        self.pose = numbers(self.world['initial_pose'], 3, 'initial_pose')
        self.size = numbers(self.world['size'], 2, 'size')
        if min(self.size) <= 0:
            raise ValueError('World size must be positive')
        self.obstacles = [numbers(box, 4, 'obstacle') for box in self.world['obstacles']]
        if any(b[0] >= b[2] or b[1] >= b[3] for b in self.obstacles):
            raise ValueError('Obstacle bounds must increase')
        self.profile = load_profile(self.get_parameter('geometry_file').value, True)
        if any(abs(x) > 1e-8 for x in self.profile['laser_rpy_rad'][:2]):
            raise ValueError('Planar simulator needs a horizontal laser')
        self.loaded = False
        self.edges = segments(self.size, self.obstacles)
        self.velocity = [0., 0., 0.]
        self.command_at = 0.
        self.last_tick = time.monotonic()
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/simulation/pose', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(Bool, '/nav/loaded', lambda msg: setattr(self, 'loaded', msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.025, self.step)
        self.create_timer(.1, self.scan)
        self.get_logger().info('PLANAR SIMULATION: ideal velocities, no wheel slip or lift physics')

    def command(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(x) for x in values):
            self.velocity = [0., 0., 0.]
            return
        self.velocity = [max(-lim, min(lim, value)) for value, lim in zip(values, (.4, .3, 1.5))]
        self.command_at = time.monotonic()

    def step(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, .05)
        self.last_tick = now
        velocity = self.velocity if now-self.command_at < .5 else [0., 0., 0.]
        candidate = advance(self.pose, velocity, dt)
        bounds = [v/1000 for v in self.profile['loaded_bounds_mm' if self.loaded else 'empty_bounds_mm']]
        if collides(candidate, bounds, self.size, self.obstacles):
            velocity = [0., 0., 0.]
            self.get_logger().warning('Simulated collision blocked motion', throttle_duration_sec=2.)
        else:
            self.pose = candidate
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        # EKF uses twist only; do not label map truth as wheel odometry pose.
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = velocity
        for i in range(6):
            odom.twist.covariance[i*7] = .0001 if i in (0, 1, 5) else 1.
        self.odom_pub.publish(odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
        imu.orientation_covariance[0] = -1.
        imu.linear_acceleration_covariance[0] = -1.
        imu.angular_velocity.z = velocity[2]
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = .01
        imu.angular_velocity_covariance[8] = .0001
        self.imu_pub.publish(imu)
        truth = PoseStamped()
        truth.header.stamp, truth.header.frame_id = stamp, 'map'
        truth.pose.position.x, truth.pose.position.y = self.pose[:2]
        truth.pose.orientation.z, truth.pose.orientation.w = math.sin(self.pose[2]/2), math.cos(self.pose[2]/2)
        self.pose_pub.publish(truth)

    def scan(self):
        x, y, yaw = self.pose
        sx, sy, _ = [v/1000 for v in self.profile['laser_xyz_mm']]
        lx, ly = x+math.cos(yaw)*sx-math.sin(yaw)*sy, y+math.sin(yaw)*sx+math.cos(yaw)*sy
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.get_clock().now().to_msg(), 'laser'
        scan.angle_min, scan.angle_increment = -math.pi, math.tau/360
        scan.angle_max = scan.angle_min + 359*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .03, 5., .1
        scan.ranges = [float(ray_distance(lx, ly, yaw+self.profile['laser_rpy_rad'][2]
                      + scan.angle_min+i*scan.angle_increment, self.edges)) for i in range(360)]
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlanarBase()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
