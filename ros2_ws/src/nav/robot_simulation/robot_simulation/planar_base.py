"""Simulated wheel/IMU/laser inputs; EKF remains the only odom TF publisher."""

import math
import os
import time
import json
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Int8, String
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robot_geometry.profile import load_profile, numbers
from .world import advance, collides, segments, ray_distance
from .rack_world import RackWorld


def require_isolated_domain():
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        raise RuntimeError('Simulation requires a nonzero ROS_DOMAIN_ID (use 75)')


class PlanarBase(Node):
    def __init__(self):
        super().__init__('planar_base')
        require_isolated_domain()
        self.declare_parameter('world_file', '')
        self.declare_parameter('geometry_file', '')
        self.declare_parameter('freeze_scan', False)
        self.declare_parameter('freeze_wheel', False)
        self.frozen_scan_stamp = None
        self.frozen_wheel_stamp = None
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
        self.rack_world = (RackWorld(self.world, [v/1000 for v in self.profile['empty_bounds_mm']])
                           if 'racks' in self.world else None)
        self.state, self.rack_index = 'WAIT_START', 0
        self.pending_lift = None
        self.motion_since = time.monotonic()
        self.metrics = {'model': 'planar_body_legs_and_carried_deck', 'samples': 0,
                        'actual_collisions': 0, 'blocked_collision_commands': 0,
                        'minimum_clearance_m': None, 'minimum_swept_clearance_m': None,
                        'lift_rejections': [], 'blocked_samples': []}
        self.report_path = Path(self.world['evidence_dir']) if self.world.get('evidence_dir') else None
        self.trace = None
        if self.report_path:
            self.report_path.mkdir(parents=True, exist_ok=True)
            self.trace = (self.report_path/'collision_trace.jsonl').open('x', encoding='utf-8')
        if self.rack_world:
            if self.rack_world.clearance(self.pose)[0] <= 0:
                raise ValueError('Initial robot pose collides')
            self.lift_pub = self.create_publisher(Int8, '/lift_state', 10)
            self.events = self.create_publisher(String, '/simulation/events',
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.racks_pub = self.create_publisher(String, '/simulation/racks',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Int8, '/lift_cmd', self.lift, 10)
            self.create_subscription(String, '/mission/status', self.status, 10)
            self.create_timer(.2, self.report)
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
        swept = self.rack_world.swept_clearance(self.pose, candidate) if self.rack_world else None
        collision = swept[0] <= 0 if swept else collides(candidate, bounds, self.size, self.obstacles)
        if collision:
            velocity = [0., 0., 0.]
            if self.rack_world:
                self.metrics['blocked_collision_commands'] += 1
                if len(self.metrics['blocked_samples']) < 30:
                    self.metrics['blocked_samples'].append({'pose': list(self.pose), 'candidate': candidate,
                                                            'clearance': swept[0], 'pair': swept[1], 'state': self.state})
            self.get_logger().warning('Simulated collision blocked motion', throttle_duration_sec=2.)
        else:
            self.pose = candidate
        if any(abs(v) > .005 for v in velocity):
            self.motion_since = now
        if self.rack_world:
            clearance, kind = self.rack_world.clearance(self.pose)
            self.metrics['samples'] += 1
            self.metrics['actual_collisions'] += int(clearance <= 0)
            for key, value in [('minimum_clearance_m', clearance), ('minimum_swept_clearance_m', swept[0])]:
                previous = self.metrics[key]
                self.metrics[key] = value if previous is None else min(value, previous)
            if self.trace:
                self.trace.write(json.dumps({'t': now, 'pose': self.pose, 'velocity': velocity,
                    'state': self.state, 'carried': self.rack_world.carried,
                    'attachment': self.rack_world.attachment, 'clearance': clearance,
                    'pair': kind, 'blocked': collision})+'\n')
            self.process_lift(now)
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        # EKF uses twist only; do not label map truth as wheel odometry pose.
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = velocity
        for i in range(6):
            odom.twist.covariance[i*7] = .0001 if i in (0, 1, 5) else 1.
        if self.get_parameter('freeze_wheel').value:
            if self.frozen_wheel_stamp is None:
                self.frozen_wheel_stamp = stamp
            odom.header.stamp = self.frozen_wheel_stamp
        else:
            self.frozen_wheel_stamp = None
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
        if self.rack_world:
            self.edges = segments(self.size, self.obstacles)
            for rack in self.rack_world.racks:
                if rack['label'] != self.rack_world.carried:
                    for points in self.rack_world.legs(rack, self.pose):
                        self.edges.extend((points[i], points[(i+1) % 4]) for i in range(4))
        x, y, yaw = self.pose
        sx, sy, _ = [v/1000 for v in self.profile['laser_xyz_mm']]
        lx, ly = x+math.cos(yaw)*sx-math.sin(yaw)*sy, y+math.sin(yaw)*sx+math.cos(yaw)*sy
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.get_clock().now().to_msg(), 'laser'
        if self.get_parameter('freeze_scan').value:
            if self.frozen_scan_stamp is None:
                self.frozen_scan_stamp = scan.header.stamp
            scan.header.stamp = self.frozen_scan_stamp
        else:
            self.frozen_scan_stamp = None
        scan.angle_min, scan.angle_increment = -math.pi, math.tau/360
        scan.angle_max = scan.angle_min + 359*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .03, 5., .1
        scan.ranges = [float(ray_distance(lx, ly, yaw+self.profile['laser_rpy_rad'][2]
                      + scan.angle_min+i*scan.angle_increment, self.edges)) for i in range(360)]
        self.scan_pub.publish(scan)

    def status(self, msg):
        value = json.loads(msg.data)
        self.state, self.rack_index = value['state'], value['rack_index']

    def event(self, event, **data):
        self.events.publish(String(data=json.dumps({'event': event, **data})))

    def lift(self, msg):
        self.event('lift_command', command=msg.data)
        if self.pending_lift is not None or msg.data not in (0, 1):
            return
        # DDS does not order different topics. Validate the mission phase after
        # the simulated actuator delay, allowing the status message to arrive.
        self.pending_lift = (time.monotonic()+.3, msg.data, self.rack_index)

    def process_lift(self, now):
        if not self.pending_lift or now < self.pending_lift[0]:
            return
        if now-self.motion_since < .25:
            return  # No acknowledgement until the body has actually stopped.
        _, command, index = self.pending_lift
        self.pending_lift = None
        expected = 'LIFT_RACK' if command else 'LOWER_RACK'
        if self.state != expected or index >= len(self.rack_world.racks):
            self.metrics['lift_rejections'].append('Lift command outside its mission phase')
            return
        label = self.rack_world.racks[index]['label']
        try:
            (self.rack_world.lift if command else self.rack_world.lower)(self.pose, label)
            self.rack_world.moves[-1]['t'] = now
            self.lift_pub.publish(Int8(data=command))
            self.event('rack_attached' if command else 'rack_placed', label=label, pose=self.pose)
        except ValueError as exc:
            self.metrics['lift_rejections'].append(str(exc))
            self.get_logger().error(str(exc))
            self.event('lift_rejected', reason=str(exc))
        self.report()

    def report(self):
        snapshot = {'racks': self.rack_world.snapshot(self.pose), 'carried': self.rack_world.carried,
                    'metrics': self.metrics, 'moves': self.rack_world.moves, 'mission_state': self.state}
        if rclpy.ok():
            self.racks_pub.publish(String(data=json.dumps(snapshot)))
        if self.report_path:
            temporary = self.report_path/'collision_summary.tmp'
            temporary.write_text(json.dumps(snapshot, indent=2), encoding='utf-8')
            temporary.replace(self.report_path/'collision_summary.json')
            self.trace.flush()

    def destroy_node(self):
        if self.rack_world:
            self.report()
        if self.trace:
            self.trace.close()
        super().destroy_node()


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
