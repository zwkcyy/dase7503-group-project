"""Adapt the documented EC ROS bridge without inventing lift completion."""

import copy
import json
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8, String, UInt8
from std_srvs.srv import SetBool, Trigger


def rotation(rpy):
    if len(rpy) != 3 or not all(math.isfinite(x) for x in rpy):
        raise ValueError('imu_to_base_rpy_rad requires three finite angles')
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def rotate_vector(vector, matrix):
    values = [vector.x, vector.y, vector.z]
    vector.x, vector.y, vector.z = [sum(row[j]*values[j] for j in range(3)) for row in matrix]


def rotate_covariance(values, matrix, fallback):
    values = list(values)
    if not any(values):
        values = [fallback, 0., 0., 0., fallback, 0., 0., 0., fallback]
    return [sum(matrix[i][a]*values[3*a+b]*matrix[j][b]
                for a in range(3) for b in range(3)) for i in range(3) for j in range(3)]


class EcAdapter(Node):
    def __init__(self):
        super().__init__('ec_adapter')
        defaults = {'enable_motion': False, 'enable_lift': False,
                    'telemetry_max_age_s': .5, 'command_timeout_s': .25,
                    'max_linear_speed_m_s': .5, 'max_angular_speed_rad_s': 2.,
                    'expected_imu_frame': 'base_link', 'imu_to_base_rpy_rad': [0., 0., 0.],
                    'imu_angular_variance': .0025, 'imu_acceleration_variance': .25,
                    'lift_up_text': 'lift up', 'lift_down_text': 'lift down',
                    'lift_stop_text': 'lift stop', 'lift_confirmation_timeout_s': 60.}
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        for key in ('telemetry_max_age_s', 'command_timeout_s', 'max_linear_speed_m_s',
                    'max_angular_speed_rad_s', 'imu_angular_variance',
                    'imu_acceleration_variance', 'lift_confirmation_timeout_s'):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if self.settings['command_timeout_s'] >= .5:
            raise ValueError('Command timeout must precede the documented 500 ms EC watchdog')
        for key in ('expected_imu_frame', 'lift_up_text', 'lift_down_text', 'lift_stop_text'):
            if not self.settings[key].strip() or '\n' in self.settings[key] or '\r' in self.settings[key]:
                raise ValueError(f'{key} must be one nonempty line')
        self.rotation = rotation(self.settings['imu_to_base_rpy_rad'])
        self.last_data = {'odom': None, 'imu': None}
        self.issues = {'odom': 'No odometry received', 'imu': 'No IMU received'}
        self.last_command = None
        self.moving = False
        self.target = None
        self.target_received = None
        self.target_sequence = 0
        self.pending = None
        self.request_id = 0
        self.last_lift_result = 'No command issued'
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.velocity_pub = self.create_publisher(Twist, '/ec/cmd_vel', 10)
        self.lift_pub = self.create_publisher(String, '/ec/lift_cmd', 10)
        self.completion_pub = self.create_publisher(Int8, '/lift_state', 10)
        self.status_pub = self.create_publisher(String, '/ec_adapter/status', 10)
        self.create_subscription(Odometry, '/ec/wheel/odom', self.odometry, qos_profile_sensor_data)
        self.create_subscription(Imu, '/ec/imu/data_raw', self.imu, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/ec/lift_state', self.lift_target, 10)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
        self.create_subscription(Int8, '/lift_cmd', self.lift_command, 10)
        self.create_subscription(String, '/mission/status', self.mission_status, 10)
        self.create_service(SetBool, '/ec_adapter/confirm_lift', self.confirm_lift)
        self.create_service(Trigger, '/ec_adapter/stop_lift', self.stop_lift)
        self.create_timer(.05, self.tick)
        self.create_timer(.5, self.publish_status)
        self.get_logger().warning(
            'EC return-form adapter: physical calibration unverified; lift targets are not completion. '
            'Motion/lift are disabled by default. No arm or keepalive commands are generated.')

    def reject(self, key, reason):
        self.issues[key] = reason
        self.last_data[key] = None
        self.get_logger().warning(f'{key}: {reason}', throttle_duration_sec=5.)

    def valid_header(self, key, header):
        stamp = header.stamp.sec + header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        if stamp <= 0 or not -.1 <= now-stamp <= self.settings['telemetry_max_age_s']:
            self.reject(key, 'Missing, stale or future timestamp; EC bridge must use the Pi ROS clock')
            return False
        return True

    def odometry(self, message):
        if not self.valid_header('odom', message.header):
            return
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            self.reject('odom', 'Expected odom/base_link frames; no silent relabelling')
            return
        pose, twist = message.pose.pose, message.twist.twist
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z,
                  *message.pose.covariance, *message.twist.covariance]
        if not all(math.isfinite(x) for x in values):
            self.reject('odom', 'Nonfinite odometry values')
            return
        self.odom_pub.publish(message)
        self.last_data['odom'] = time.monotonic()
        self.issues.pop('odom', None)

    def imu(self, message):
        if not self.valid_header('imu', message.header):
            return
        if message.header.frame_id != self.settings['expected_imu_frame']:
            self.reject('imu', 'Input frame differs from the configured EC frame')
            return
        values = [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                  message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z,
                  *message.angular_velocity_covariance, *message.linear_acceleration_covariance]
        if not all(math.isfinite(x) for x in values) or message.angular_velocity_covariance[0] < 0:
            self.reject('imu', 'Invalid values or unavailable angular velocity')
            return
        output = copy.deepcopy(message)
        rotate_vector(output.angular_velocity, self.rotation)
        rotate_vector(output.linear_acceleration, self.rotation)
        output.angular_velocity_covariance = rotate_covariance(
            output.angular_velocity_covariance, self.rotation, self.settings['imu_angular_variance'])
        if output.linear_acceleration_covariance[0] >= 0:
            output.linear_acceleration_covariance = rotate_covariance(
                output.linear_acceleration_covariance, self.rotation, self.settings['imu_acceleration_variance'])
        output.header.frame_id = 'base_link'
        output.orientation.x = output.orientation.y = output.orientation.z = 0.
        output.orientation.w = 1.
        output.orientation_covariance = [-1., 0., 0., 0., 0., 0., 0., 0., 0.]
        self.imu_pub.publish(output)
        self.last_data['imu'] = time.monotonic()
        self.issues.pop('imu', None)

    def telemetry_ready(self):
        now = time.monotonic()
        return all(t is not None and now-t <= self.settings['telemetry_max_age_s']
                   for t in self.last_data.values())

    def send_zero(self):
        self.velocity_pub.publish(Twist())
        self.moving = False

    def velocity(self, message):
        if not self.settings['enable_motion']:
            return
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(x) for x in values):
            if self.moving:
                self.send_zero()
            return
        if any(values) and not self.telemetry_ready():
            if self.moving:
                self.send_zero()
            self.get_logger().warning('Motion rejected: EC odometry/IMU are not fresh', throttle_duration_sec=5.)
            return
        output = Twist()
        speed = math.hypot(values[0], values[1])
        scale = min(1., self.settings['max_linear_speed_m_s']/speed) if speed else 1.
        output.linear.x, output.linear.y = values[0]*scale, values[1]*scale
        limit = self.settings['max_angular_speed_rad_s']
        output.angular.z = max(-limit, min(limit, values[2]))
        self.velocity_pub.publish(output)
        self.last_command = time.monotonic()
        self.moving = any((output.linear.x, output.linear.y, output.angular.z))

    def lift_target(self, message):
        self.target = int(message.data)
        self.target_received = time.monotonic()
        self.target_sequence += 1
        # UInt8 0/1 are commanded targets, 2 means no target. None proves arrival.

    def lift_command(self, message):
        value = int(message.data)
        if not self.settings['enable_lift']:
            self.last_lift_result = 'Lift command rejected: enable_lift is false'
            return
        if value not in (0, 1) or self.pending is not None:
            self.last_lift_result = 'Lift command rejected: invalid value or another command is pending'
            return
        if self.lift_pub.get_subscription_count() != 1:
            self.last_lift_result = 'Lift command rejected: expected one EC String receiver'
            return
        self.request_id += 1
        self.pending = {'id': self.request_id, 'target': value,
                        'sent': time.monotonic(), 'sequence': self.target_sequence}
        self.completion_pub.publish(Int8(data=-1))
        self.lift_pub.publish(String(data=self.settings['lift_up_text' if value else 'lift_down_text']))
        self.last_lift_result = 'Command sent; waiting for fresh target telemetry and physical operator confirmation'

    def confirm_lift(self, request, response):
        pending = self.pending
        now = time.monotonic()
        expected = int(request.data)
        if pending is None or pending['target'] != expected:
            response.message = 'No matching pending lift command'
        elif now-pending['sent'] >= self.settings['lift_confirmation_timeout_s']:
            self.pending = None
            response.message = 'Pending command expired; confirmation rejected'
        elif (self.target_received is None or now-self.target_received > self.settings['telemetry_max_age_s']
              or self.target_sequence <= pending['sequence'] or self.target != expected):
            response.message = 'Fresh matching EC target telemetry is required'
        else:
            self.completion_pub.publish(Int8(data=expected))
            response.success = True
            response.message = f"Command {pending['id']} completed by OPERATOR observation, not an EC arrival sensor"
            self.pending = None
        self.last_lift_result = response.message
        self.publish_status()
        return response

    def stop_lift(self, request, response):
        if not self.settings['enable_lift'] or self.lift_pub.get_subscription_count() != 1:
            response.message = 'Lift disabled or EC receiver unavailable'
            return response
        # Only an explicit service request sends stop; it may release servo holding.
        self.lift_pub.publish(String(data=self.settings['lift_stop_text']))
        self.pending = None
        self.completion_pub.publish(Int8(data=-1))
        response.success = True
        response.message = 'Stop command sent; physical holding behaviour remains unverified'
        self.last_lift_result = response.message
        return response

    def mission_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict) and status.get('state') in ('ERROR', 'COMPLETE', 'WAIT_START'):
            self.pending = None

    def tick(self):
        now = time.monotonic()
        if self.moving and (self.last_command is None
                or now-self.last_command >= self.settings['command_timeout_s'] or not self.telemetry_ready()):
            # One zero then silence; do not keep the EC watchdog alive indefinitely.
            self.send_zero()
        if self.pending is not None and now-self.pending['sent'] >= self.settings['lift_confirmation_timeout_s']:
            self.pending = None
            self.last_lift_result = 'Operator confirmation timed out; no completion published'

    def publish_status(self):
        now = time.monotonic()
        state = {'source_contract': 'EC form 2026-10-25 / 40d3041',
                 'physical_acceptance': False, 'telemetry_ready': self.telemetry_ready(),
                 'input_age_s': {key: None if value is None else round(now-value, 3)
                                 for key, value in self.last_data.items()},
                 'issues': self.issues, 'motion_enabled': self.settings['enable_motion'],
                 'lift_enabled': self.settings['enable_lift'], 'reported_lift_target': self.target,
                 'pending_lift_id': self.pending['id'] if self.pending else None,
                 'lift_completion_source': 'operator_only', 'lift_result': self.last_lift_result,
                 'imu_axes': 'EC declared frame plus configured rotation; physical sign unverified'}
        self.status_pub.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = EcAdapter()
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok() and node.moving:
                node.send_zero()
            node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
