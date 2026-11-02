"""ROS 2 bridge between the EC binary UART firmware and the NAV adapter topics.

Topics (all remappable, names match ``robot_bringup/launch/ec_bridge.launch.py``):

  subscribe /ec/cmd_vel        geometry_msgs/Twist   -> type 01
  subscribe /ec/lift_cmd       std_msgs/String|UInt8 -> type 02 or 07
  publish   /ec/wheel/odom     nav_msgs/Odometry     <- type 81
  publish   /ec/imu/data_raw   sensor_msgs/Imu       <- type 82
  publish   /ec/lift_state     std_msgs/UInt8        <- type 83
  publish   /ec_bridge/status  std_msgs/String       <- local diagnostics

The bridge forwards and reports; it does not invent motion. Nothing is published
when the uplink has been silent for ``telemetry_timeout_s``, so the adapter sees a
stale stream instead of a repeated stale value. Frame timestamps come from the Pi
ROS clock, because the firmware ``sample_ms`` is an uptime counter since boot.
"""

import json
import math
import time
import uuid

import rclpy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, String, UInt8
from std_srvs.srv import SetBool, Trigger

from . import protocol
from .framing import PacketParser
from .lift_retry import ACTION_GIVE_UP, LiftCommandRetrier
from .transport import SerialTransport

COVARIANCE_LENGTH = 6


def yaw_to_quaternion(yaw):
    return Quaternion(x=0., y=0., z=math.sin(yaw * .5), w=math.cos(yaw * .5))


def diagonal_covariance(diagonal):
    values = [0.] * 36
    for index, value in enumerate(diagonal):
        values[index * 7] = float(value)
    return values


class EcSerialBridge(Node):
    def __init__(self):
        super().__init__('ec_serial_bridge')
        defaults = {
            'port': '/dev/ttyACM0',
            'baudrate': 115200,
            'read_timeout_s': .01,
            'write_timeout_s': .05,
            'reconnect_interval_s': 1.,
            'resync_timeout_s': .1,
            'telemetry_timeout_s': 1.,
            'lift_cmd_mode': 'text',
            'allow_lateral_motion': False,
            'lift_up_text': 'lift up',
            'lift_down_text': 'lift down',
            'lift_stop_text': 'lift stop',
            'lift_ack_timeout_s': .2,
            'lift_retry_max': 5,
            'lift_command_max_age_s': 2.,
            'enable_lift_stop': False,
            'lift_state_mode': 'passthrough',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'imu_frame_id': 'base_link',
            'pose_covariance_diagonal': [0.] * COVARIANCE_LENGTH,
            'twist_covariance_diagonal': [0.] * COVARIANCE_LENGTH,
            'request_status_on_open': True,
            'status_period_s': 1.,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        self._validate()

        self.parser = PacketParser()
        self.transport = SerialTransport(
            self.settings['port'], self.settings['baudrate'], self.settings['read_timeout_s'],
            self.settings['write_timeout_s'], self.settings['reconnect_interval_s'])

        self.rx_frames = 0
        self.tx_frames = 0
        self.malformed = 0
        self.last_rx = {}
        self.last_frame_time = None
        self.last_byte_time = None
        self.opened_once = False
        self.last_ack = None
        self.ec_status = None
        self.lift_direction = None
        self.session_id = uuid.uuid4().hex
        self.lift_request_id = 0
        self.active_lift = None
        self.serial_generation = 0
        self.last_lift_cancel = ''
        self.lift_retrier = LiftCommandRetrier(self.settings['lift_ack_timeout_s'],
                                               int(self.settings['lift_retry_max']),
                                               self.settings['lift_command_max_age_s'])
        self.counters = {'command': 0, 'rejected_command': 0, 'nonfinite_velocity': 0,
                         'lateral_command_rejected': 0,
                         'unknown_text': 0, 'lift_state_published': 0,
                         'lift_retry': 0, 'lift_ack_lost': 0}

        self.odom_pub = self.create_publisher(Odometry, '/ec/wheel/odom', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/ec/imu/data_raw', qos_profile_sensor_data)
        self.lift_state_pub = self.create_publisher(UInt8, '/ec/lift_state', 10)
        self.status_pub = self.create_publisher(String, '/ec_bridge/status', 10)
        self.lift_event_pub = self.create_publisher(String, '/ec_bridge/lift_event', 10)

        self.create_subscription(Twist, '/ec/cmd_vel', self.on_velocity, 10)
        lift_type = UInt8 if self.settings['lift_cmd_mode'] == 'uint8' else String
        self.create_subscription(lift_type, '/ec/lift_cmd', self.on_lift_command, 10)
        self.create_subscription(Empty, '/ec_bridge/cancel_lift',
                                 lambda _: self.cancel_lift('NAV cancelled the pending lift'), 10)
        self.create_subscription(String, '/mission/status', self.on_mission_status, 10)
        self.create_service(SetBool, '/ec_bridge/set_arm', self.on_set_arm)
        self.create_service(SetBool, '/ec_bridge/estop', self.on_estop)
        self.create_service(Trigger, '/ec_bridge/stop', self.on_stop)
        self.create_service(Trigger, '/ec_bridge/lift_stop', self.on_lift_stop)
        self.create_service(Trigger, '/ec_bridge/request_status', self.on_request_status)

        self.create_timer(self.settings['read_timeout_s'], self.poll)
        self.create_timer(self.settings['status_period_s'], self.publish_status)
        self.get_logger().warning(
            'EC binary bridge: no arm or keepalive command is generated; '
            f'lift_cmd_mode={self.settings["lift_cmd_mode"]}, '
            f'lift_state_mode={self.settings["lift_state_mode"]}')

    def _validate(self):
        if self.settings['lift_cmd_mode'] not in ('text', 'uint8'):
            raise ValueError("lift_cmd_mode must be 'text' or 'uint8'")
        if self.settings['lift_state_mode'] not in ('passthrough', 'off', 'target_mirror'):
            raise ValueError("lift_state_mode must be 'passthrough', 'off' or 'target_mirror'")
        for key in ('read_timeout_s', 'write_timeout_s', 'reconnect_interval_s', 'resync_timeout_s',
                    'telemetry_timeout_s', 'status_period_s', 'lift_command_max_age_s'):
            value = self.settings[key]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('port', 'odom_frame_id', 'base_frame_id', 'imu_frame_id'):
            if not str(self.settings[key]).strip():
                raise ValueError(f'{key} must not be empty')
        for key in ('lift_up_text', 'lift_down_text', 'lift_stop_text'):
            text = self.settings[key]
            if not text.strip() or '\n' in text or '\r' in text:
                raise ValueError(f'{key} must be one nonempty line')
        for key in ('pose_covariance_diagonal', 'twist_covariance_diagonal'):
            if len(self.settings[key]) != COVARIANCE_LENGTH:
                raise ValueError(f'{key} must hold {COVARIANCE_LENGTH} values')
        ack_timeout = self.settings['lift_ack_timeout_s']
        if not math.isfinite(ack_timeout) or ack_timeout <= 0:
            raise ValueError('lift_ack_timeout_s must be finite and positive')
        retry_max = self.settings['lift_retry_max']
        if isinstance(retry_max, bool) or not isinstance(retry_max, int) or retry_max < -1:
            raise ValueError('lift_retry_max must be -1 (unlimited) or >= 0')

    # ---- downlink ---------------------------------------------------------

    def send(self, frame, label):
        if not self.transport.write(frame):
            self.invalidate_link('serial write failed')
            self.counters['rejected_command'] += 1
            self.get_logger().warning(f'{label} not sent: {self.transport.last_error or "link closed"}',
                                      throttle_duration_sec=5.)
            return False
        self.tx_frames += 1
        self.counters['command'] += 1
        return True

    def lift_event(self, event, reason=''):
        if self.active_lift is not None:
            self.lift_event_pub.publish(String(data=json.dumps({
                **self.active_lift, 'event': event, 'reason': reason,
                'stamp_ns': self.get_clock().now().nanoseconds})))

    def cancel_lift(self, reason, event='cancelled'):
        self.lift_event(event, reason)
        self.lift_retrier.clear()
        self.active_lift = None
        self.lift_direction = None
        self.last_lift_cancel = reason

    def invalidate_link(self, reason):
        self.cancel_lift(reason)
        self.parser.reset()
        self.last_rx.clear()
        self.ec_status = None
        self.last_frame_time = None
        self.last_byte_time = None
        self.opened_once = False

    def on_mission_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict) and status.get('state') in ('ERROR', 'COMPLETE', 'WAIT_START'):
            self.cancel_lift('mission ' + status['state'])

    def on_velocity(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not self.settings['allow_lateral_motion'] and values[1] != 0.:
            self.counters['lateral_command_rejected'] += 1
            self.get_logger().error('Nonzero vy rejected; sending zero velocity. Check NAV drive_mode.',
                                    throttle_duration_sec=2.)
            self.send(protocol.encode_velocity(0., 0., 0.), 'lateral-command stop')
            return
        if not all(math.isfinite(value) for value in values):
            # The firmware disarms on a non-finite command; keep that behaviour visible.
            self.counters['nonfinite_velocity'] += 1
            self.get_logger().warning('non-finite velocity forwarded: the EC will disarm',
                                      throttle_duration_sec=5.)
        self.send(protocol.encode_velocity(*values), 'velocity')

    def on_lift_command(self, message):
        if self.settings['lift_cmd_mode'] == 'uint8':
            value = int(message.data)
            if value not in (0, 1, 2):
                self.get_logger().warning(f'lift command {value} outside 0/1/2 ignored')
                return
            if value == 2:
                self._send_lift_stop()
                return
            self._send_lift_direction(value)
            return
        text = str(message.data).strip()
        if text == self.settings['lift_up_text']:
            self._send_lift_direction(1)
        elif text == self.settings['lift_down_text']:
            self._send_lift_direction(0)
        elif text == self.settings['lift_stop_text']:
            self._send_lift_stop()
        else:
            self.counters['unknown_text'] += 1
            self.get_logger().warning(f'unrecognised lift text {text!r} ignored', throttle_duration_sec=5.)

    def _send_lift_direction(self, direction):
        self.cancel_lift('superseded by a new lift command')
        self.lift_request_id += 1
        self.active_lift = {'session': self.session_id, 'request_id': self.lift_request_id,
                            'direction': direction}
        self.lift_event('submitted')
        if not self.link_ok(time.monotonic()) or not self.opened_once:
            self.cancel_lift('no fresh serial session; issue a new command after reconnection', 'failed')
            return
        if self.send(protocol.encode_lift(direction), 'lift'):
            self.lift_direction = direction
            # 02 is an event command: one frame, then wait for the EC's F0 ack and
            # resend only if it does not arrive within lift_ack_timeout_s.
            self.lift_retrier.arm(direction, time.monotonic())
            self.lift_event('sent')

    def service_lift_retry(self, now):
        """Resend the outstanding 02 frame when its ack is overdue."""
        action, direction = self.lift_retrier.plan(now)
        if action is None:
            return
        if action == ACTION_GIVE_UP:
            self.counters['lift_ack_lost'] += 1
            self.get_logger().error(
                f'no F0 ack for lift {direction} after {self.lift_retrier.attempts} frames, '
                'giving up; check the link and the firmware ack path')
            self.cancel_lift('lift acknowledgement retry/age limit reached', 'failed')
            return
        if self.send(protocol.encode_lift(direction), 'lift retry'):
            self.counters['lift_retry'] += 1
            self.lift_retrier.retried(now)
            self.get_logger().warning(
                f'lift {direction}: no F0 ack within {self.settings["lift_ack_timeout_s"]:.2f} s, '
                f'resent (attempt {self.lift_retrier.attempts})', throttle_duration_sec=2.)

    def _send_lift_stop(self):
        # Cancellation stops future transmissions, not a lift already moving.
        self.cancel_lift('lift stop requested')
        if not self.settings['enable_lift_stop']:
            message = 'lift stop disabled: EC cancelled type 07; pending retries cancelled only'
            self.get_logger().warning(message)
            return False, message
        ok = self.send(protocol.encode_lift_stop(), 'lift stop')
        return ok, ('lift stop frame sent; physical stop not confirmed' if ok
                    else f'not sent: {self.transport.last_error}')

    def on_set_arm(self, request, response):
        if not request.data:
            self.cancel_lift('disarm requested')
        ok = self.send(protocol.encode_arm(request.data), 'arm')
        response.success = ok
        response.message = 'arm frame sent' if ok else f'not sent: {self.transport.last_error}'
        return response

    def on_estop(self, request, response):
        if request.data:
            self.cancel_lift('emergency stop requested')
        ok = self.send(protocol.encode_estop(request.data), 'estop')
        response.success = ok
        response.message = 'estop frame sent' if ok else f'not sent: {self.transport.last_error}'
        return response

    def on_stop(self, request, response):
        self.cancel_lift('stop requested')
        ok = self.send(protocol.encode_stop(), 'stop')
        response.success = ok
        response.message = 'stop frame sent' if ok else f'not sent: {self.transport.last_error}'
        return response

    def on_lift_stop(self, request, response):
        response.success, response.message = self._send_lift_stop()
        return response

    def on_request_status(self, request, response):
        ok = self.send(protocol.encode_status_request(), 'status request')
        response.success = ok
        response.message = 'status request sent' if ok else f'not sent: {self.transport.last_error}'
        return response

    # ---- uplink -----------------------------------------------------------

    def poll(self):
        now = time.monotonic()
        if not self.transport.is_open:
            self.invalidate_link('serial connection closed')
        chunk = self.transport.poll(now)
        if not self.transport.is_open:
            self.invalidate_link('serial connection lost')
            return
        if self.transport.open_count != self.serial_generation:
            self.invalidate_link('serial session changed; old commands discarded')
            self.serial_generation = self.transport.open_count
            self.opened_once = True
            self.get_logger().info(f'serial link open on {self.settings["port"]}; no commands replayed')
            if self.settings['request_status_on_open']:
                if not self.send(protocol.encode_status_request(), 'startup status request'):
                    return
        if chunk:
            for frame in self.parser.feed(chunk):
                self.handle_frame(frame, now)
            self.last_byte_time = now
        elif self.parser.pending():
            if now - self.last_byte_time >= self.settings['resync_timeout_s']:
                for frame in self.parser.expire():
                    self.handle_frame(frame, now)
                self.get_logger().warning(
                    f'abandoned a partial frame after {self.settings["resync_timeout_s"]:.2f} s',
                    throttle_duration_sec=5.)
        if not self.link_ok(now):
            self.cancel_lift('valid serial telemetry timed out')
        elif self.transport.is_open:
            self.service_lift_retry(now)

    def handle_frame(self, frame, now):
        decoded = protocol.decode(frame)
        if decoded is None:
            self.malformed += 1
            self.get_logger().warning(
                f'malformed frame type=0x{frame.type:02X} length={len(frame.data)} dropped',
                throttle_duration_sec=5.)
            return
        self.rx_frames += 1
        self.last_frame_time = now
        kind, payload = decoded
        self.last_rx[kind] = now
        if kind == 'odometry':
            self.publish_odometry(payload, now)
        elif kind == 'imu':
            self.publish_imu(payload, now)
        elif kind == 'lift_state':
            self.publish_lift_state(payload, now)
        elif kind == 'status':
            self.ec_status = payload
            if payload.has(protocol.FLAG_ESTOP) or not payload.has(protocol.FLAG_ARMED):
                self.cancel_lift('EC reports emergency stop or disarmed state')
            if payload.fault != protocol.FAULT_NONE:
                self.cancel_lift('EC reports fault ' + str(payload.fault), 'failed')
                self.get_logger().warning(
                    f'EC fault {payload.fault} ({protocol.FAULT_NAMES.get(payload.fault, "unknown")})',
                    throttle_duration_sec=5.)
        else:
            self.last_ack = str(payload)
            if not payload.accepted:
                self.counters['rejected_command'] += 1
                self.get_logger().warning(f'EC rejected 0x{payload.command_type:02X}',
                                          throttle_duration_sec=5.)
            result = self.lift_retrier.settle(payload.command_type, payload.accepted, now)
            if result == 'ok':
                self.lift_event('accepted')
            elif result in ('rejected', 'expired'):
                self.get_logger().error(
                    f'lift command {self.lift_direction} acknowledgement {result}; not resending',
                    throttle_duration_sec=5.)
                self.cancel_lift('lift acknowledgement ' + result, 'failed')

    def fresh(self, kind, now):
        stamp = self.last_rx.get(kind)
        return stamp is not None and now - stamp <= self.settings['telemetry_timeout_s']

    def publish_odometry(self, sample, now):
        if not self.fresh('odometry', now):
            return
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.settings['odom_frame_id']
        message.child_frame_id = self.settings['base_frame_id']
        message.pose.pose.position.x = sample.x
        message.pose.pose.position.y = sample.y
        message.pose.pose.position.z = 0.
        message.pose.pose.orientation = yaw_to_quaternion(sample.yaw)
        message.twist.twist = Twist(linear=Vector3(x=sample.vx, y=sample.vy, z=0.),
                                    angular=Vector3(x=0., y=0., z=sample.wz))
        message.pose.covariance = diagonal_covariance(self.settings['pose_covariance_diagonal'])
        message.twist.covariance = diagonal_covariance(self.settings['twist_covariance_diagonal'])
        self.odom_pub.publish(message)

    def publish_imu(self, sample, now):
        if not self.fresh('imu', now):
            return
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.settings['imu_frame_id']
        # Type 82 carries raw chip axes and no orientation: declare no attitude.
        message.orientation.w = 1.
        message.orientation_covariance = [-1.] + [0.] * 8
        message.angular_velocity = Vector3(x=sample.gx, y=sample.gy, z=sample.gz)
        message.linear_acceleration = Vector3(x=sample.ax, y=sample.ay, z=sample.az)
        self.imu_pub.publish(message)

    def publish_lift_state(self, payload, now):
        mode = self.settings['lift_state_mode']
        if mode == 'off':
            return
        if not self.fresh('lift_state', now):
            return
        if mode == 'target_mirror':
            if self.lift_direction is None:
                return
            value = self.lift_direction
        else:
            value = payload.state
        self.lift_state_pub.publish(UInt8(data=value))
        self.counters['lift_state_published'] += 1

    # ---- diagnostics ------------------------------------------------------

    def link_ok(self, now):
        return (self.transport.is_open and self.last_frame_time is not None
                and now - self.last_frame_time <= self.settings['telemetry_timeout_s'])

    def publish_status(self):
        now = time.monotonic()
        ec_status = self.ec_status if self.fresh('status', now) else None
        status = {
            'port': self.settings['port'],
            'open': self.transport.is_open,
            'open_count': self.transport.open_count,
            'link_ok': self.link_ok(now),
            'rx_frames': self.rx_frames,
            'tx_frames': self.tx_frames,
            'malformed_frames': self.malformed,
            'crc_errors': self.parser.crc_errors,
            'tail_errors': self.parser.tail_errors,
            'oversize_frames': self.parser.oversize_frames,
            'discarded_bytes': self.parser.discarded_bytes,
            'pending_bytes': self.parser.pending(),
            'last_ack': self.last_ack,
            'ec_armed': None if ec_status is None else ec_status.has(protocol.FLAG_ARMED),
            'ec_estop': None if ec_status is None else ec_status.has(protocol.FLAG_ESTOP),
            'ec_fault': None if ec_status is None else ec_status.fault,
            'last_error': self.transport.last_error,
            'lift_cmd_mode': self.settings['lift_cmd_mode'],
            'lift_state_mode': self.settings['lift_state_mode'],
            'lift_direction_sent': self.lift_direction,
            'lift_ack_pending': self.lift_retrier.pending,
            'lift_frames_sent': self.lift_retrier.attempts,
            'session': self.session_id,
            'last_lift_cancel': self.last_lift_cancel,
            'enable_lift_stop': self.settings['enable_lift_stop'],
            'allow_lateral_motion': self.settings['allow_lateral_motion'],
            'lift_command_max_age_s': self.settings['lift_command_max_age_s'],
            'counters': dict(self.counters),
            'odometry_age_s': None if self.last_rx.get('odometry') is None
            else round(now - self.last_rx['odometry'], 3),
            'imu_age_s': None if self.last_rx.get('imu') is None
            else round(now - self.last_rx['imu'], 3),
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        self.transport.close()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = EcSerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
