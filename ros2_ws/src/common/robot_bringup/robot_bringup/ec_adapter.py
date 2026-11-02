"""Adapt EC telemetry, with explicit legacy/manual and binary-feedback modes."""

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Int8, String, UInt8
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
        defaults = {'enable_motion': False, 'enable_lift': False, 'allow_lateral_motion': True,
                    'telemetry_max_age_s': .5, 'command_timeout_s': .25,
                    'max_linear_speed_m_s': .5, 'max_angular_speed_rad_s': 2.,
                    'expected_imu_frame': 'base_link', 'imu_to_base_rpy_rad': [0., 0., 0.],
                    'imu_angular_variance': .0025, 'imu_acceleration_variance': .25,
                    'lift_up_text': 'lift up', 'lift_down_text': 'lift down',
                    'lift_stop_text': 'lift stop', 'lift_confirmation_timeout_s': 60.,
                    'lift_completion_mode': 'operator'}
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
        if self.settings['lift_completion_mode'] not in ('operator', 'ec_state'):
            raise ValueError('lift_completion_mode must be operator or ec_state')
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
        self.stop_requests = []
        self.service_group = ReentrantCallbackGroup()
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.velocity_pub = self.create_publisher(Twist, '/ec/cmd_vel', 10)
        self.lift_pub = self.create_publisher(String, '/ec/lift_cmd', 10)
        self.cancel_lift_pub = self.create_publisher(Empty, '/ec_bridge/cancel_lift', 10)
        self.completion_pub = self.create_publisher(Int8, '/lift_state', 10)
        self.status_pub = self.create_publisher(String, '/ec_adapter/status', 10)
        self.create_subscription(Odometry, '/ec/wheel/odom', self.odometry, qos_profile_sensor_data)
        self.create_subscription(Imu, '/ec/imu/data_raw', self.imu, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/ec/lift_state', self.lift_target, 10)
        self.create_subscription(String, '/ec_bridge/lift_event', self.lift_event, 10)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
        self.create_subscription(Int8, '/lift_cmd', self.lift_command, 10)
        self.create_subscription(String, '/mission/status', self.mission_status, 10)
        self.create_service(SetBool, '/ec_adapter/confirm_lift', self.confirm_lift)
        self.lift_stop_client = self.create_client(Trigger, '/ec_bridge/lift_stop',
                                                   callback_group=self.service_group)
        self.create_service(Trigger, '/ec_adapter/stop_lift', self.stop_lift,
                            callback_group=self.service_group)
        self.create_timer(.05, self.tick)
        self.create_timer(.5, self.publish_status)
        self.get_logger().warning(
            f'EC adapter: lift_completion_mode={self.settings["lift_completion_mode"]}; '
            'EC state is a firmware-timed endpoint, not a measured arrival sensor. '
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
        # Include message age at arrival, rather than granting another full
        # timeout to a packet that was already almost stale in transport.
        self.last_data[key] = time.monotonic() - max(0., now-stamp)
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
        q = pose.orientation
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .01:
            self.reject('odom', 'Odometry orientation is not a unit quaternion')
            return
        self.odom_pub.publish(message)
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
        if not self.settings['allow_lateral_motion'] and values[1] != 0.:
            self.send_zero()
            self.get_logger().error('Lateral command rejected: use differential NAV and docking', throttle_duration_sec=2.)
            return
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
        now = time.monotonic()
        if self.settings['lift_completion_mode'] == 'ec_state':
            if int(message.data) not in (0, 1):
                self.cancel_pending_lift('Invalid EC lift state')
                self.target_received = None
                return
            if (self.pending is not None and self.target_received is not None
                    and now-self.target_received > self.settings['telemetry_max_age_s']):
                self.cancel_pending_lift('EC lift feedback interrupted; new command required')
        self.target = int(message.data)
        self.target_received = now
        self.target_sequence += 1
        self.complete_from_ec(now)

    def cancel_pending_lift(self, reason, notify_bridge=True):
        self.pending = None
        self.last_lift_result = reason
        self.completion_pub.publish(Int8(data=-1))
        if notify_bridge:
            self.cancel_lift_pub.publish(Empty())

    def lift_event(self, message):
        if self.settings['lift_completion_mode'] != 'ec_state' or self.pending is None:
            return
        try:
            event = json.loads(message.data)
            if not isinstance(event, dict):
                return
            stamp = int(event['stamp_ns'])
            key = (str(event['session']), int(event['request_id']))
            direction = int(event['direction'])
        except (ValueError, TypeError, KeyError):
            return
        pending = self.pending
        if (stamp < pending['stamp_ns'] or stamp > self.get_clock().now().nanoseconds + 100_000_000
                or direction != pending['target']):
            return
        if event.get('event') == 'submitted':
            if pending['bridge_key'] is not None:
                self.cancel_pending_lift('Another bridge lift command superseded this request')
                return
            pending['bridge_key'] = key
        elif key == pending['bridge_key']:
            if event.get('event') == 'accepted':
                pending['acknowledged'] = True
                self.complete_from_ec(time.monotonic())
            elif event.get('event') in ('cancelled', 'failed'):
                self.cancel_pending_lift('EC bridge: ' + str(event.get('reason', 'cancelled')),
                                         notify_bridge=False)

    def complete_from_ec(self, now):
        pending = self.pending
        if self.settings['lift_completion_mode'] != 'ec_state' or pending is None:
            return
        if now-pending['sent'] >= self.settings['lift_confirmation_timeout_s']:
            self.cancel_pending_lift('EC lift completion timed out')
            return
        if (pending['acknowledged'] and pending['baseline'] != pending['target']
                and self.target_sequence > pending['sequence'] and self.target == pending['target']
                and self.target_received is not None
                and now-self.target_received <= self.settings['telemetry_max_age_s']):
            self.completion_pub.publish(Int8(data=pending['target']))
            self.last_lift_result = (f"Command {pending['id']} completed by F0 acceptance and EC state transition; "
                                     'firmware-timed endpoint, not a physical arrival sensor')
            self.pending = None
            self.publish_status()

    def lift_command(self, message):
        value = int(message.data)
        now = time.monotonic()
        if not self.settings['enable_lift']:
            self.last_lift_result = 'Lift command rejected: enable_lift is false'
            return
        if value not in (0, 1) or self.pending is not None:
            self.last_lift_result = 'Lift command rejected: invalid value or another command is pending'
            return
        if self.lift_pub.get_subscription_count() != 1:
            self.last_lift_result = 'Lift command rejected: expected one EC String receiver'
            return
        if self.settings['lift_completion_mode'] == 'ec_state':
            if (self.target_received is None or self.target not in (0, 1)
                    or now-self.target_received > self.settings['telemetry_max_age_s']):
                self.last_lift_result = 'Lift rejected: wait for fresh EC endpoint feedback before commanding'
                return
            if self.target == value:
                self.last_lift_result = 'Lift rejected: EC already reports this endpoint; no new transition to confirm'
                return
        self.request_id += 1
        self.pending = {'id': self.request_id, 'target': value,
                        'sent': now, 'sequence': self.target_sequence,
                        'stamp_ns': self.get_clock().now().nanoseconds,
                        'baseline': self.target, 'bridge_key': None, 'acknowledged': False}
        self.completion_pub.publish(Int8(data=-1))
        self.lift_pub.publish(String(data=self.settings['lift_up_text' if value else 'lift_down_text']))
        self.last_lift_result = ('Command sent; waiting for F0 acceptance and EC endpoint state transition'
                                if self.settings['lift_completion_mode'] == 'ec_state' else
                                'Command sent; waiting for fresh telemetry and physical operator confirmation')

    def confirm_lift(self, request, response):
        if self.settings['lift_completion_mode'] != 'operator':
            response.success = False
            response.message = 'Manual completion is disabled in ec_state mode; waiting for EC acknowledgement and transition'
            return response
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

    async def stop_lift(self, request, response):
        self.cancel_pending_lift('Lift stop requested; completion invalidated')
        if not self.lift_stop_client.service_is_ready():
            response.success = False
            response.message = 'Bridge stop service unavailable; retry cancellation requested, physical stop unconfirmed'
            return response
        # Relay the bridge's capability/result; publishing text is not success.
        try:
            remote = self.lift_stop_client.call_async(Trigger.Request())
        except Exception as error:
            response.success = False
            response.message = f'Bridge stop request failed: {error}; physical stop unconfirmed'
            return response
        finished = Future()
        self.stop_requests.append((remote, finished, time.monotonic()+1.))
        result = await finished
        response.success = result is not None and result.success
        response.message = (result.message if result is not None else
                            'Bridge stop response timed out/failed; physical stop unconfirmed')
        self.last_lift_result = response.message
        return response

    def mission_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict) and status.get('state') in ('ERROR', 'COMPLETE', 'WAIT_START'):
            if self.pending is not None:
                self.cancel_pending_lift('Mission ' + status['state'])

    def tick(self):
        now = time.monotonic()
        for remote, finished, deadline in list(self.stop_requests):
            if remote.done() or now >= deadline:
                result = None
                if remote.done():
                    try:
                        result = remote.result()
                    except Exception as error:
                        self.get_logger().warning(f'Lift stop service failed: {error}')
                else:
                    self.lift_stop_client.remove_pending_request(remote)
                    remote.cancel()
                finished.set_result(result)
                self.stop_requests.remove((remote, finished, deadline))
        if self.moving and (self.last_command is None
                or now-self.last_command >= self.settings['command_timeout_s'] or not self.telemetry_ready()):
            # One zero then silence; do not keep the EC watchdog alive indefinitely.
            self.send_zero()
        if self.pending is not None and now-self.pending['sent'] >= self.settings['lift_confirmation_timeout_s']:
            self.cancel_pending_lift('Lift confirmation timed out; no completion published')
        elif (self.pending is not None and self.settings['lift_completion_mode'] == 'ec_state'
              and (self.target_received is None
                   or now-self.target_received > self.settings['telemetry_max_age_s'])):
            self.cancel_pending_lift('EC lift feedback timed out; new command required')

    def publish_status(self):
        now = time.monotonic()
        ec_state = self.settings['lift_completion_mode'] == 'ec_state'
        state = {'source_contract': ('EC binary UART / 2026-11-02' if ec_state else 'EC form 2026-10-25 / 40d3041'),
                 'physical_acceptance': False, 'telemetry_ready': self.telemetry_ready(),
                 'input_age_s': {key: None if value is None else round(now-value, 3)
                                 for key, value in self.last_data.items()},
                 'issues': self.issues, 'motion_enabled': self.settings['enable_motion'],
                 'allow_lateral_motion': self.settings['allow_lateral_motion'],
                 'lift_enabled': self.settings['enable_lift'], 'reported_lift_target': self.target,
                 'pending_lift_id': self.pending['id'] if self.pending else None,
                 'lift_completion_source': 'ec_timed_state_transition' if ec_state else 'operator_only',
                 'lift_feedback_age_s': None if self.target_received is None else round(now-self.target_received, 3),
                 'lift_acknowledged': self.pending is not None and self.pending.get('acknowledged', False),
                 'lift_result': self.last_lift_result,
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
