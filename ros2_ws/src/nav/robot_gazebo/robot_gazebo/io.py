"""Adapt Gazebo sensors/actuators and audit physics without driving body poses.

Only wheel commands and the prismatic lift target are sent to Gazebo. Model
truth is published for evidence, never as localisation or encoder input.
"""
import json
import math
from collections import deque
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Int8, String
from std_srvs.srv import Trigger
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class GazeboIO(Node):
    def __init__(self):
        super().__init__('gazebo_io')
        self.declare_parameter('output_dir', '')
        self.directory = Path(self.get_parameter('output_dir').value)
        self.settings = yaml.safe_load((self.directory/'physics.yaml').read_text())
        self.scene = yaml.safe_load((self.directory/'scene.yaml').read_text())
        self.lock = threading.Lock()
        self.latest_poses, self.pose_received = {}, 0.
        self.contacts, self.contact_samples = {}, 0
        self.truth_rows = deque(maxlen=5000)
        self.last_truth_time, self.dropped_truth_rows = None, 0
        self.gz = GzNode()
        self.contact_topics = ['/gazebo/contacts/'+name for name in
            ['body', 'tray', 'front_left_wheel_link', 'front_right_wheel_link',
             'rear_left_wheel_link', 'rear_right_wheel_link', 'rack_A', 'rack_B', 'rack_C', 'rack_D']]
        self.state, self.index, self.delivered = 'WAIT_START', 0, 0
        self.task_elapsed = 0.
        self.pending, self.carried, self.target, self.lift_position = None, None, 0., None
        self.joint_received, self.command_received, self.last_sim = 0., 0., None
        self.velocity, self.command = [0., 0., 0.], Twist()
        self.bad_contacts, self.allowed_contacts, self.moves = [], 0, []
        self.unexpected_contact_samples = 0
        self.emergency = False
        self.trace = (self.directory/'gazebo_trace.jsonl').open('x', encoding='utf-8')
        self.events_file = (self.directory/'gazebo_events.jsonl').open('x', encoding='utf-8')
        self.latest_snapshot = {}
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.event_pub = self.create_publisher(String, '/simulation/events', latched)
        self.pose_pub = self.create_publisher(PoseStamped, '/simulation/pose', 10)
        self.lift_pub = self.create_publisher(Int8, '/lift_state', 10)
        self.target_pub = self.create_publisher(Float64, '/gazebo/lift_target', 10)
        self.velocity_pub = self.create_publisher(Twist, '/gazebo/cmd_vel', 10)
        self.stop_client = self.create_client(Trigger, '/nav/emergency_stop')
        self.stop_requested = False
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(Odometry, '/gazebo/wheel_odom', self.on_odom, 10)
        self.create_subscription(Imu, '/gazebo/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/gazebo/joint_states', self.on_joints, 10)
        self.create_subscription(Int8, '/lift_cmd', self.on_lift, 10)
        self.create_subscription(String, '/mission/status', self.on_status, 10)
        # A lost /clock bridge must not freeze the actuator watchdog.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.025, self.tick, clock=self.wall_clock)
        self.create_timer(.5, self.report, clock=self.wall_clock)
        # Subscribe only after all state read by transport callbacks is ready.
        self.gz.subscribe(Pose_V, '/world/course/dynamic_pose/info', self.on_truth)
        for topic in self.contact_topics:
            self.gz.subscribe(Contacts, topic, self.on_contacts)
        self.get_logger().info('GAZEBO PHYSICS: wheel drive, gravity/contact lift, rendered camera; no pose commands')

    def on_truth(self, msg):
        with self.lock:
            self.latest_poses = {p.name: {'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                'qx': p.orientation.x, 'qy': p.orientation.y, 'qz': p.orientation.z, 'qw': p.orientation.w,
                'yaw': yaw(p.orientation)} for p in msg.pose if p.name == 'robot' or p.name.startswith('rack_')}
            self.pose_received = time.monotonic()
            stamp = msg.header.stamp.sec+msg.header.stamp.nsec/1e9
            if 'robot' in self.latest_poses and (self.last_truth_time is None or stamp > self.last_truth_time):
                if len(self.truth_rows) == self.truth_rows.maxlen:
                    self.dropped_truth_rows += 1
                # Use physics timestamps, independently of /clock bridge startup
                # and ROS executor scheduling. Never interpolate missing samples.
                self.truth_rows.append(self.snapshot(stamp, dict(self.latest_poses)))
                self.last_truth_time = stamp

    def snapshot(self, stamp, poses):
        return {'sim_seconds': stamp, 'mission_state': self.state, 'rack_index': self.index,
            'task_elapsed_s': self.task_elapsed, 'delivered_count': self.delivered,
            'carried': self.carried, 'poses': poses,
            'lift_position_m': self.lift_position, 'encoder_velocity': list(self.velocity)}

    def flush_truth(self):
        with self.lock:
            rows = list(self.truth_rows)
            self.truth_rows.clear()
        for row in rows:
            self.trace.write(json.dumps(row)+'\n')

    def on_contacts(self, msg):
        with self.lock:
            self.contact_samples += 1
            # Keep nonempty samples between ROS timer callbacks; do not lose a transient contact.
            for c in msg.contact:
                pair = tuple(sorted((c.collision1.name, c.collision2.name)))
                self.contacts[pair] = max(self.contacts.get(pair, 0.), max(c.depth, default=0.))

    def on_command(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.command = msg if all(math.isfinite(v) for v in values) else Twist()
        self.command.linear.x = max(-.4, min(.4, self.command.linear.x))
        self.command.linear.y = max(-.3, min(.3, self.command.linear.y))
        self.command.angular.z = max(-1.5, min(1.5, self.command.angular.z))
        self.command_received = time.monotonic()

    def on_odom(self, msg):
        # These velocities come from Gazebo's wheel-joint encoder integration.
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        for i in range(6):
            msg.twist.covariance[i*7] = .001 if i in (0, 1, 5) else 1.
        self.velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        self.odom_pub.publish(msg)

    def on_imu(self, msg):
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance[0] = -1.
        msg.linear_acceleration_covariance[0] = -1.
        for i in (0, 4, 8): msg.angular_velocity_covariance[i] = .0005
        self.imu_pub.publish(msg)

    def on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == 'lift_joint':
                self.lift_position, self.joint_received = position, time.monotonic()

    def on_status(self, msg):
        data = json.loads(msg.data)
        self.state, self.index = data['state'], data['rack_index']
        self.delivered = data.get('delivered_count', self.index)
        self.task_elapsed = data.get('task_elapsed_s', 0.)

    def event(self, kind, **data):
        event = {'event': kind, 'sim_seconds': self.get_clock().now().nanoseconds/1e9, **data}
        self.events_file.write(json.dumps(event)+'\n'); self.events_file.flush()
        self.event_pub.publish(String(data=json.dumps(event)))

    def on_lift(self, msg):
        if msg.data not in (0, 1) or self.pending is not None:
            return
        self.pending = {'up': bool(msg.data), 'index': self.index, 'started': False}
        self.event('lift_command', command=msg.data)

    def process_lift(self, poses, dt):
        if not self.pending or self.emergency:
            return
        up, index = self.pending['up'], self.pending['index']
        if not 0 <= index < 4:
            return
        label = self.scene['racks'][index]['label']
        robot, rack = poses.get('robot'), poses.get('rack_'+label)
        if not robot or not rack or time.monotonic()-self.joint_received > .5:
            return
        if self.state != ('LIFT_RACK' if up else 'LOWER_RACK'):
            return
        if max(abs(v) for v in self.velocity) > .01:
            return
        if not self.pending['started']:
            error = math.hypot(robot['x']-rack['x'], robot['y']-rack['y'])
            yaw_error = abs(math.atan2(math.sin(robot['yaw']-rack['yaw']), math.cos(robot['yaw']-rack['yaw'])))
            if up and (error > .025 or yaw_error > .05):
                self.event('lift_rejected', reason='Rack not centred over physical tray', error_m=error)
                self.pending = None
                return
            self.pending['started'] = True
        limit = self.settings['lift_stroke_mm']/1000 if up else 0.
        step = self.settings['lift_speed_mm_s']/1000*dt
        self.target += max(-step, min(step, limit-self.target))
        self.target_pub.publish(Float64(data=self.target))
        reached = abs(self.lift_position-limit) < .001
        grounded = abs(rack['z']) < .002
        elevated = rack['z'] > .01
        if reached and (elevated if up else grounded):
            self.carried = label if up else None
            self.lift_pub.publish(Int8(data=int(up)))
            move = {'label': label, 'up': up, 'rack_pose': rack, 'robot_pose': robot}
            self.moves.append(move)
            self.event('rack_lifted' if up else 'rack_placed', **move)
            self.pending = None

    def permitted(self, a, b):
        # Ground support is expected only for wheel tires and rack legs.
        for ground, other in ((a, b), (b, a)):
            if 'field::ground::floor' in ground:
                return ('wheel_link::tire' in other or ('rack_' in other and '::leg_' in other))
        label = self.carried
        if self.pending and self.pending['started']:
            label = self.scene['racks'][self.pending['index']]['label']
        for tray, rack in ((a, b), (b, a)):
            if 'robot::lift_tray::' in tray and label and f'rack_{label}::rack::deck' in rack:
                return True
        return False

    def tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        dt = min(.05, max(0., now-(self.last_sim if self.last_sim is not None else now)))
        self.last_sim = now
        self.flush_truth()
        with self.lock:
            poses, fresh, contacts = dict(self.latest_poses), self.pose_received, self.contacts
            self.contacts = {}
        for (a, b), depth in contacts.items():
            if self.permitted(a, b):
                self.allowed_contacts += 1
            else:
                self.unexpected_contact_samples += 1
                if len(self.bad_contacts) < 100:
                    self.bad_contacts.append({'a': a, 'b': b, 'depth_m': depth, 'state': self.state, 't': now})
                if not self.emergency:
                    self.event('unexpected_contact', collision1=a, collision2=b, depth_m=depth)
                    self.get_logger().error(f'Unexpected Gazebo contact: {a} / {b}')
                self.emergency = True
        self.velocity_pub.publish(self.command if not self.emergency and time.monotonic()-self.command_received < .4 else Twist())
        if self.emergency and not self.stop_requested and self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
            self.stop_requested = True
        if time.monotonic()-fresh > .5 or 'robot' not in poses:
            return
        self.process_lift(poses, dt)
        robot = poses['robot']
        msg = PoseStamped(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = robot['x'], robot['y'], robot['z']
        q = msg.pose.orientation
        q.x, q.y, q.z, q.w = [robot[k] for k in ('qx', 'qy', 'qz', 'qw')]
        self.pose_pub.publish(msg)
        self.latest_snapshot = self.snapshot(self.last_truth_time or now, poses)

    def report(self):
        data = {**self.latest_snapshot, 'engine': 'Gazebo Harmonic', 'moves': self.moves,
                'contact_samples_received': self.contact_samples, 'allowed_contact_samples': self.allowed_contacts,
                'unexpected_contact_samples': self.unexpected_contact_samples, 'unexpected_contacts': self.bad_contacts,
                'emergency_latched': self.emergency, 'dropped_truth_rows': self.dropped_truth_rows}
        temp = self.directory/'gazebo_summary.tmp'
        temp.write_text(json.dumps(data, indent=2), encoding='utf-8')
        temp.replace(self.directory/'gazebo_summary.json')
        self.trace.flush()

    def destroy_node(self):
        if rclpy.ok():
            self.velocity_pub.publish(Twist())
        self.gz.unsubscribe('/world/course/dynamic_pose/info')
        for topic in self.contact_topics:
            self.gz.unsubscribe(topic)
        self.flush_truth()
        self.report()
        self.trace.close(); self.events_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboIO()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Jazzy may invalidate a wait-set capsule while SIGINT shuts the context down.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
