"""Exercise real EC ROS nodes through an isolated Linux PTY, never hardware.

The only serial path supplied to the bridge is a temporary symlink to a PTY
created by this process. Synthetic motor/lift commands cannot reach a real EC.
Run after sourcing ROS 2 and the project overlay; argument 1 is an output folder.
"""
import errno
import fcntl
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
import tty

import rclpy
import serial
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8, String, UInt8
from std_srvs.srv import Trigger

from ec_serial_bridge import protocol
from ec_serial_bridge.framing import PacketParser, encode_packet


DOMAIN = 205


class PtyChecks:
    def __init__(self, output):
        if sys.platform != 'linux':
            raise RuntimeError('This test requires Linux openpty; no real serial fallback exists')
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(prefix='dase7503-ec-pty-')
        self.port = Path(self.temporary.name) / 'virtual-ec'
        self.master = self.slave = None
        self.children = []
        self.logs = []
        self.parser = PacketParser()
        self.downlink = []
        self.events = []
        self.received = {}
        self.counts = {}
        self.checks = []
        self.telemetry_enabled = True
        self.sensor_enabled = True
        self.respond_status = True
        self.last_telemetry = 0.
        self.sample_ms = 0
        self.freeze_samples = False
        self.node = None
        self.executor = None
        self.context = rclpy.context.Context()

    def new_pty(self):
        self.close_pty()
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        slave_path = os.ttyname(self.slave)
        if not slave_path.startswith('/dev/pts/'):
            raise RuntimeError('Refusing any serial endpoint outside /dev/pts/')
        flags = fcntl.fcntl(self.master, fcntl.F_GETFL)
        fcntl.fcntl(self.master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        if self.port.is_symlink():
            self.port.unlink()
        self.port.symlink_to(slave_path)
        self.parser = PacketParser()

    def close_pty(self):
        for attr in ('master', 'slave'):
            descriptor = getattr(self, attr)
            if descriptor is not None:
                os.close(descriptor)
                setattr(self, attr, None)

    def spawn(self, module, parameters):
        log = (self.output / (module + '.log')).open('w')
        self.logs.append(log)
        args = [sys.executable, '-c', f'from {module} import main; main()', '--ros-args']
        for key, value in parameters.items():
            args += ['-p', f'{key}:={value}']
        env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN),
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', PYTHONDONTWRITEBYTECODE='1')
        child = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                 env=env, start_new_session=True)
        self.children.append(child)

    def start(self):
        self.new_pty()
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        rclpy.init(context=self.context, domain_id=DOMAIN)
        self.node = rclpy.create_node('ec_pty_check', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.node.create_subscription(String, '/ec_bridge/status',
                                      lambda msg: self.store('bridge', json.loads(msg.data)), 10)
        self.node.create_subscription(String, '/ec_adapter/status',
                                      lambda msg: self.store('adapter', json.loads(msg.data)), 10)
        self.node.create_subscription(String, '/ec_bridge/lift_event',
                                      lambda msg: self.events.append(json.loads(msg.data)), 10)
        for key, topic, kind in (
                ('raw_odom', '/ec/wheel/odom', Odometry),
                ('raw_imu', '/ec/imu/data_raw', Imu),
                ('odom', '/odom/wheel', Odometry), ('imu', '/imu/data_raw', Imu)):
            self.node.create_subscription(kind, topic, lambda msg, k=key: self.store(k, msg),
                                          qos_profile_sensor_data)
        self.node.create_subscription(UInt8, '/ec/lift_state', lambda msg: self.store('lift', msg), 10)
        self.velocity = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.lift = self.node.create_publisher(Int8, '/lift_cmd', 10)
        # Direct bridge input is used only to exercise retry cancellation on PTY.
        self.bridge_lift = self.node.create_publisher(String, '/ec/lift_cmd', 10)
        self.request_status = self.node.create_client(Trigger, '/ec_bridge/request_status')
        self.spawn('ec_serial_bridge.node', {
            'port': str(self.port), 'baudrate': 115200, 'status_period_s': 0.1,
            'reconnect_interval_s': 0.1, 'request_status_on_open': 'true',
            'lift_ack_timeout_s': 0.2, 'lift_retry_max': 5,
            'lift_command_max_age_s': 2.0, 'allow_lateral_motion': 'false'})
        self.spawn('robot_bringup.ec_adapter', {
            'enable_motion': 'false', 'enable_lift': 'false',
            'allow_lateral_motion': 'false', 'lift_completion_mode': 'ec_state'})

    def store(self, key, value):
        self.received[key] = value
        self.counts[key] = self.counts.get(key, 0) + 1

    def write(self, data):
        if self.master is not None:
            try:
                os.write(self.master, data)
            except OSError as exc:
                if exc.errno not in (errno.EIO, errno.EAGAIN):
                    raise

    def telemetry(self):
        if not self.freeze_samples:
            self.sample_ms = (self.sample_ms + 100) & 0xFFFFFFFF
        status_flags = (protocol.FLAG_INITIALIZED | protocol.FLAG_ARMED
                        | protocol.FLAG_ODOMETRY_VALID | protocol.FLAG_IMU_VALID
                        | protocol.FLAG_LIFT_STATE_KNOWN)
        data = encode_packet(protocol.TYPE_STATUS, bytes((status_flags, 0)))
        data += encode_packet(protocol.TYPE_LIFT_STATE, b'\x00')
        if self.sensor_enabled:
            data += encode_packet(protocol.TYPE_ODOMETRY,
                                  struct.pack('<I6f', self.sample_ms, 1.25, -.5, .2, 0., 0., 0.))
            data += encode_packet(protocol.TYPE_IMU,
                                  struct.pack('<I6f', self.sample_ms, 0., 0., 9.81, 0., 0., .01))
        self.write(data)

    def pump(self):
        for child in self.children:
            if child.poll() is not None:
                raise AssertionError(f'ROS child exited unexpectedly: {child.args}, code {child.returncode}')
        if self.master is not None:
            try:
                chunk = os.read(self.master, 65536)
            except BlockingIOError:
                chunk = b''
            except OSError as exc:
                if exc.errno != errno.EIO:
                    raise
                chunk = b''
            for frame in self.parser.feed(chunk):
                self.downlink.append(frame)
                if frame.type == protocol.TYPE_STATUS_REQUEST and self.respond_status:
                    self.write(encode_packet(protocol.TYPE_ACK, bytes((frame.type, 0))))
                    self.telemetry()
        now = time.monotonic()
        if self.telemetry_enabled and now - self.last_telemetry >= .1:
            self.telemetry()
            self.last_telemetry = now
        self.executor.spin_once(timeout_sec=.01)

    def wait(self, predicate, label, timeout=8.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                return
        raise AssertionError('Timed out: ' + label)

    def settle(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.pump()

    def check(self, name, condition, details=None):
        if not condition:
            raise AssertionError(name + ': ' + repr(details))
        self.checks.append({'name': name, 'passed': True, 'details': details})

    def run(self):
        self.start()
        self.wait(lambda: self.received.get('adapter', {}).get('telemetry_ready')
                  and all(key in self.received for key in ('raw_odom', 'raw_imu', 'lift')),
                  'genuine serial bridge and adapter telemetry')
        odom, imu = self.received['odom'], self.received['imu']
        self.check('valid_uplink_converts_through_both_ros_nodes',
                   abs(odom.pose.pose.position.x - 1.25) < 1e-6
                   and abs(imu.linear_acceleration.z - 9.81) < 1e-4
                   and odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_link'
                   and imu.header.frame_id == 'base_link' and self.received['lift'].data == 0)
        self.wait(lambda: self.received.get('bridge', {}).get('serial_exclusive_requested'),
                  'exclusive serial status')
        refused = False
        try:
            with serial.Serial(str(self.port), 115200, timeout=.01, exclusive=True):
                pass
        except serial.SerialException:
            refused = True
        self.check('cooperating_second_serial_opener_is_rejected', refused)
        self.check('firmware_validity_flags_are_exposed_as_diagnostics',
                   all(self.received['bridge'].get(key) is True for key in (
                       'ec_initialized', 'ec_odometry_valid', 'ec_imu_valid', 'ec_lift_state_known')))

        self.wait(lambda: self.velocity.get_subscription_count() > 0
                  and self.lift.get_subscription_count() > 0, 'adapter subscriptions')
        before = len(self.downlink)
        command = Twist()
        command.linear.x = .1
        for _ in range(5):
            self.velocity.publish(command)
            self.lift.publish(Int8(data=1))
            self.settle(.05)
        self.settle(.5)
        sent = [frame.type for frame in self.downlink[before:]]
        self.check('disabled_adapter_never_sends_motion_or_lift',
                   not any(kind in (protocol.TYPE_VELOCITY, protocol.TYPE_LIFT) for kind in sent), sent)

        bridge_before = dict(self.received['bridge'])
        damaged = bytearray(encode_packet(protocol.TYPE_STATUS, b'\x3b\x00'))
        damaged[-4] ^= 0x80
        self.write(protocol.encode_stop() + protocol.encode_velocity(.1, 0., 0.)
                   + bytes(damaged) + encode_packet(protocol.TYPE_LIFT_STATE, b'\x00'))
        self.wait(lambda: self.received.get('bridge', {}).get('malformed_frames', 0)
                  >= bridge_before['malformed_frames'] + 2
                  and self.received.get('bridge', {}).get('crc_errors', 0)
                  > bridge_before['crc_errors'], 'echo rejection and bad CRC recovery')
        self.check('echo_and_crc_recovery_keep_nodes_alive', self.received['bridge']['link_ok'],
                   {key: self.received['bridge'][key] for key in ('crc_errors', 'malformed_frames')})

        self.wait(self.request_status.service_is_ready, 'status service')
        before = len(self.downlink)
        future = self.request_status.call_async(Trigger.Request())
        self.wait(lambda: future.done(), 'status service reply')
        self.wait(lambda: any(frame.type == protocol.TYPE_STATUS_REQUEST
                              for frame in self.downlink[before:])
                  and self.received.get('bridge', {}).get('last_ack') == 'status_request:accepted',
                  'status request and firmware acknowledgement')
        self.check('status_service_round_trip', future.result().success)

        self.wait(lambda: self.bridge_lift.get_subscription_count() > 0, 'bridge lift subscription')
        before = len(self.downlink)
        self.bridge_lift.publish(String(data='lift up'))
        self.wait(lambda: any(frame.type == protocol.TYPE_LIFT for frame in self.downlink[before:]),
                  'synthetic lift request on PTY')
        self.wait(lambda: self.received['bridge']['lift_ack_pending'], 'unacknowledged request')
        old_generation = self.received['bridge']['open_count']
        old_event_count = len(self.events)
        self.close_pty()
        self.settle(.3)
        self.new_pty()
        before = len(self.downlink)
        self.wait(lambda: self.received.get('bridge', {}).get('open_count', 0) > old_generation
                  and self.received['bridge']['link_ok'], 'reconnection to replacement PTY')
        self.write(encode_packet(protocol.TYPE_ACK, bytes((protocol.TYPE_LIFT, 0))))
        self.settle(2.2)
        replayed = [frame.type for frame in self.downlink[before:]]
        self.check('disconnect_cancels_lift_and_reconnect_does_not_replay',
                   protocol.TYPE_LIFT not in replayed and not self.received['bridge']['lift_ack_pending']
                   and any(event['event'] == 'cancelled' for event in self.events[old_event_count:])
                   and not any(event['event'] == 'accepted' for event in self.events[old_event_count:]),
                   {'reconnected_open_count': self.received['bridge']['open_count'],
                    'post_reconnect_downlink_types': replayed})

        self.sensor_enabled = False
        self.settle(.2)
        before = self.counts.get('raw_odom', 0)
        self.wait(lambda: not self.received.get('adapter', {}).get('telemetry_ready', True),
                  'sensor stream silence despite live status frames', timeout=3.)
        self.settle(.3)
        self.check('missing_sensor_telemetry_is_not_refreshed_by_status',
                   self.counts.get('raw_odom', 0) == before and self.received['bridge']['link_ok'])
        self.sensor_enabled = True
        self.wait(lambda: self.received.get('adapter', {}).get('telemetry_ready'), 'sensor recovery')
        self.check('fresh_sensor_stream_recovers', True)

        self.freeze_samples = True
        self.settle(.2)
        before = self.counts.get('raw_odom', 0)
        self.wait(lambda: not self.received.get('adapter', {}).get('telemetry_ready', True),
                  'frozen sample_ms must become stale even with continuous status/IMU/odom bytes',
                  timeout=3.)
        self.settle(.3)
        self.check('frozen_sample_clock_does_not_refresh_sensor_timestamps',
                   self.counts.get('raw_odom', 0) == before and self.received['bridge']['link_ok'])
        self.freeze_samples = False
        self.wait(lambda: self.received.get('adapter', {}).get('telemetry_ready'),
                  'progressing sample clock after duplicate stream')
        self.check('progressing_samples_recover_after_duplicates', True)

        previous_sample = self.sample_ms
        self.sample_ms = 0
        self.wait(lambda: set(self.received.get('bridge', {}).get('sample_clock', {})
                             .get('clock_faults', {})) == {'odometry', 'imu'}
                  and not self.received.get('adapter', {}).get('telemetry_ready', True),
                  'backward MCU counter must latch both streams')
        before = self.counts.get('raw_odom', 0)
        self.sample_ms = previous_sample + 1000
        self.settle(.7)
        self.check('backward_clock_fault_stays_latched_even_when_counter_catches_up',
                   self.counts.get('raw_odom', 0) == before
                   and not self.received['adapter']['telemetry_ready'])
        old_generation = self.received['bridge']['open_count']
        self.close_pty()
        self.settle(.3)
        self.sample_ms = 0
        self.new_pty()
        self.wait(lambda: self.received.get('bridge', {}).get('open_count', 0) > old_generation
                  and not self.received['bridge'].get('sample_clock', {}).get('clock_faults')
                  and self.received.get('adapter', {}).get('telemetry_ready'),
                  'serial reconnection must reset the sample clock fault')
        self.check('new_serial_session_resets_sample_clock_fault', True)

    def close(self):
        for child in self.children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in self.children:
            try:
                child.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=3.)
        if self.node is not None:
            self.node.destroy_node()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.)
        if self.context.ok():
            self.context.try_shutdown()
        self.close_pty()
        self.temporary.cleanup()
        for log in self.logs:
            log.close()


def main():
    output = Path(sys.argv[1]).resolve()
    checks = PtyChecks(output)
    report = {'passed': False, 'scope': 'Real ROS bridge/adapter nodes; Linux PTY only',
              'domain_id': DOMAIN, 'real_serial_access': False, 'physical_actuation': False}
    try:
        checks.run()
        report['passed'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        report['checks'] = checks.checks
        checks.close()
    (output / 'ec_serial_pty.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
