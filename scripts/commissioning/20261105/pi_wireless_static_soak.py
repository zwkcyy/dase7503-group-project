#!/usr/bin/env python3
"""Static Pi sensor observation plus JSON/JPEG transport over the caller's SSH.

Run only with the chassis disarmed. This observer NEVER publishes commands or
calls arm/stop/disarm services. A failure stops its own driver process groups;
that is not a substitute for a physical emergency stop or a disarm command.
Source the commissioning ROS environment before running this script.
"""
import argparse
import base64
import datetime
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import traceback
import uuid


LIMITS = {
    'camera': (20., 40., .75),
    'scan': (8., 12., .5),
    'raw_odom': (8., 12., .5),
    'wheel_odom': (8., 12., .5),
    'imu': (8., 12., .5),
    'ekf': (25., 35., .5),
    'bridge_status': (.5, 15., 2.5),
    'adapter_status': (.5, 15., 2.5),
}
BOUNDARY = [
    'Static, disarmed observations only; no ground speed, motion or lifting acceptance.',
    'SSH carries <=1 fps compressed JPEG previews and sampled real scan messages; not cross-network raw DDS image validation.',
    'No QR content confirmation, camera calibration, obstacle geometry or localization accuracy acceptance.',
    'No moving disconnect-stop, Wi-Fi coverage/roaming or braking-distance validation.',
    'Command subscriptions observe received messages; they cannot prove absence of every external service call or undelivered command.',
    'Own-process cleanup does not send an emergency-stop or disarm command.',
]


class Stream:
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.first = self.last = self.stamp = None
        self.max_gap = 0.
        self.nonmonotonic = 0

    def add(self, now, stamp=None):
        if stamp is not None:
            if self.stamp is not None and stamp <= self.stamp:
                self.nonmonotonic += 1
                raise ValueError(self.name + ': non-increasing timestamp')
            self.stamp = stamp
        if self.last is not None:
            self.max_gap = max(self.max_gap, now-self.last)
        self.first = now if self.first is None else self.first
        self.last = now
        self.count += 1

    def reset_window(self):
        self.count = 0
        self.first = self.last = None
        self.max_gap = 0.

    def snapshot(self, now, ros_now, window_start=None):
        span = None if self.first is None or self.last is None else self.last-self.first
        return {
            'count': self.count,
            'rate_hz': (self.count-1)/span if span and self.count > 1 else None,
            'window_rate_hz': self.count/(now-window_start) if window_start is not None and now > window_start else None,
            'max_receive_gap_s': self.max_gap,
            'first_delay_s': self.first-window_start if self.first is not None and window_start is not None else None,
            'last_receive_age_s': None if self.last is None else now-self.last,
            'last_stamp_age_s': None if self.stamp is None else ros_now-self.stamp,
            'nonmonotonic_stamps': self.nonmonotonic,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path,
                        help='New leaf directory; existing directories are never overwritten')
    parser.add_argument('--duration', type=float, default=120.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 10. <= args.duration <= 3600.:
        parser.error('--duration must be finite and between 10 and 3600 seconds')
    out = args.output.expanduser().absolute()
    try:
        out.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        print('Cannot create unique output directory: '+str(exc), file=sys.stderr)
        return 2
    run_id = uuid.uuid4().hex
    start = time.monotonic()
    started_at = datetime.datetime.now().astimezone().isoformat()
    failures, processes, interrupted = [], [], []
    cleanup = []
    node = None
    rclpy = None
    observers = []
    stats = {key: Stream(key) for key in LIMITS}
    latest = {'image': None, 'scan': None, 'bridge_status': None, 'adapter_status': None}
    commands = {key: {'received': 0, 'nonzero': 0} for key in
                ('/cmd_vel_nav', '/cmd_vel_requested', '/cmd_vel', '/ec/cmd_vel', '/lift_cmd', '/ec/lift_cmd')}
    parameters = {'verified': False}
    bridge_baseline = {}
    peak = {key: {'translation_m_s': 0., 'yaw_rad_s': 0.} for key in ('raw_odom', 'wheel_odom', 'ekf')}
    measuring_at = None
    measurement_ended_at = None
    measurement_complete = False
    frames_emitted = 0
    seq = 0
    stream_broken = False
    stdout_fd = sys.stdout.fileno()
    previous_handlers = {}
    ndjson = (out/'telemetry.ndjson').open('w', encoding='utf-8')
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0'
    os.environ.setdefault('ROS_LOG_DIR', str(out/'ros_logs'))

    def fail(reason):
        if reason not in failures:
            failures.append(reason)

    def emit(item):
        nonlocal stream_broken
        line = json.dumps(item, allow_nan=False, separators=(',', ':'))+'\n'
        ndjson.write(line)
        ndjson.flush()
        if stream_broken:
            return
        payload = line.encode('utf-8')
        deadline = time.monotonic()+2.
        try:
            while payload:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('stdout transport blocked for 2 seconds')
                if not select.select([], [stdout_fd], [], min(.2, remaining))[1]:
                    continue
                try:
                    written = os.write(stdout_fd, payload)
                except BlockingIOError:
                    continue
                if written <= 0:
                    raise BrokenPipeError('stdout closed')
                payload = payload[written:]
        except (OSError, TimeoutError) as exc:
            stream_broken = True
            raise RuntimeError('JSON transport failed: '+str(exc)) from exc

    def guard(callback):
        def wrapped(message):
            try:
                callback(message)
            except Exception as exc:
                fail(type(exc).__name__+': '+str(exc))
        return wrapped

    def ros_now():
        return node.get_clock().now().nanoseconds/1e9

    def stamp_of(message, key):
        stamp = message.header.stamp
        if not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError(key+': invalid nanoseconds')
        value = stamp.sec+stamp.nanosec/1e9
        if not math.isfinite(value) or value <= 0:
            raise ValueError(key+': missing/invalid timestamp')
        age = ros_now()-value
        if not -.1 <= age <= LIMITS[key][2]:
            raise ValueError(f'{key}: stale/future timestamp ({age:.3f}s)')
        if not message.header.frame_id:
            raise ValueError(key+': missing frame')
        return value

    def finite(values, key):
        if not all(math.isfinite(value) for value in values):
            raise ValueError(key+': nonfinite values')

    def image_cb(message):
        stamp = stamp_of(message, 'camera')
        if message.width <= 0 or message.height <= 0 or message.step <= 0:
            raise ValueError('camera: invalid dimensions/step')
        if not message.encoding or len(message.data) < message.step*message.height:
            raise ValueError('camera: missing encoding/truncated pixel buffer')
        stats['camera'].add(time.monotonic(), stamp)
        latest['image'] = message

    def scan_cb(message):
        stamp = stamp_of(message, 'scan')
        finite((message.angle_min, message.angle_max, message.angle_increment,
                message.range_min, message.range_max, message.scan_time, message.time_increment), 'scan')
        if (len(message.ranges) < 2 or message.angle_increment <= 0 or message.range_min < 0
                or message.range_max <= message.range_min or message.scan_time < 0 or message.time_increment < 0
                or abs(message.angle_max-message.angle_min-(len(message.ranges)-1)*message.angle_increment) > 1e-5):
            raise ValueError('scan: invalid geometry/time/range bounds')
        points = [[index, float(value)] for index, value in enumerate(message.ranges) if math.isfinite(value)]
        if not points or any(not message.range_min <= value <= message.range_max for _, value in points):
            raise ValueError('scan: no finite returns or out-of-range return')
        # NaN/Inf rays mean unknown/no-return; preserve that count, never turn them into free space.
        latest['scan'] = {'stamp_s': stamp, 'frame_id': message.header.frame_id,
                          'angle_min_rad': float(message.angle_min),
                          'angle_increment_rad': float(message.angle_increment),
                          'range_min_m': float(message.range_min), 'range_max_m': float(message.range_max),
                          'beam_count': len(message.ranges), 'finite_count': len(points),
                          'points': points, 'unknown_count': len(message.ranges)-len(points),
                          'minimum_finite_range_m': min(value for _, value in points),
                          'maximum_finite_range_m': max(value for _, value in points)}
        stats['scan'].add(time.monotonic(), stamp)

    def odom_cb(key, message):
        stamp = stamp_of(message, key)
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        finite((p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.linear.z,
                v.angular.x, v.angular.y, v.angular.z, *message.pose.covariance, *message.twist.covariance), key)
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            raise ValueError(key+': expected odom/base_link')
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01:
            raise ValueError(key+': non-unit quaternion')
        translation, yaw = math.hypot(v.linear.x, v.linear.y), abs(v.angular.z)
        peak[key]['translation_m_s'] = max(peak[key]['translation_m_s'], translation)
        peak[key]['yaw_rad_s'] = max(peak[key]['yaw_rad_s'], yaw)
        stats[key].add(time.monotonic(), stamp)
        if translation > .01 or yaw > .03:
            raise ValueError(f'{key}: static feedback exceeds limits ({translation:.6f}m/s, {yaw:.6f}rad/s); '
                             'telemetry/estimate is not proof of physical motion; operator must verify physical stop')

    def imu_cb(message):
        stamp = stamp_of(message, 'imu')
        a, v, q = message.linear_acceleration, message.angular_velocity, message.orientation
        finite((a.x, a.y, a.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w,
                *message.orientation_covariance, *message.angular_velocity_covariance,
                *message.linear_acceleration_covariance), 'imu')
        if message.header.frame_id != 'base_link' or message.angular_velocity_covariance[0] < 0:
            raise ValueError('imu: expected base_link with angular velocity available')
        stats['imu'].add(time.monotonic(), stamp)

    def status_cb(key, message):
        value = json.loads(message.data)
        if not isinstance(value, dict):
            raise ValueError(key+': expected JSON object')
        latest[key] = value
        stats[key].add(time.monotonic())
        if key == 'adapter_status':
            if value.get('motion_enabled') is not False or value.get('lift_enabled') is not False:
                raise ValueError('adapter motion/lift must both remain disabled')
            if measuring_at is not None and value.get('telemetry_ready') is not True:
                raise ValueError('adapter telemetry is not ready')
        else:
            if value.get('ec_armed') is True:
                raise ValueError('EC unexpectedly armed; operator must disarm/stop physically')
            if value.get('ec_fault') not in (None, 0):
                raise ValueError('EC reports a fault')
            if value.get('lift_cmd_mode') != 'text':
                raise ValueError('Expected text lift command contract')
            for counter in ('crc_errors', 'tail_errors', 'oversize_frames', 'malformed_frames'):
                current = value.get(counter)
                if not isinstance(current, int) or current < 0:
                    raise ValueError('Invalid bridge counter '+counter)
                bridge_baseline.setdefault(counter, current)
                if current != bridge_baseline[counter]:
                    raise ValueError('Bridge counter changed: '+counter)
            clocks = value.get('sample_clock', {})
            if clocks.get('clock_faults'):
                raise ValueError('EC sample clock fault')
            if measuring_at is not None:
                if value.get('ec_armed') is not False or value.get('link_ok') is not True or value.get('ec_fault') != 0:
                    raise ValueError('EC lost unarmed/link-ok/fault-free state')
                if value.get('session') != bridge_baseline.get('session') or value.get('open_count') != bridge_baseline.get('open_count'):
                    raise ValueError('EC serial session changed/reconnected during observation')

    def command_cb(topic, message):
        record = commands[topic]
        record['received'] += 1
        if hasattr(message, 'linear'):
            values = (message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z)
            finite(values, topic)
            unsafe = any(value != 0. for value in values)
        else:
            # For lift commands, even zero may mean LOWER and must not be treated as harmless.
            unsafe = True
        if unsafe:
            record['nonzero'] += 1
            raise ValueError(topic+': unexpected motion/lift command observed; static test aborted')

    def snapshot():
        now = time.monotonic()
        return {key: value.snapshot(now, ros_now(), measuring_at) for key, value in stats.items()}

    def health_problem():
        if interrupted:
            return 'Interrupted by '+interrupted[0]
        for kind, process, _ in processes:
            if process.poll() is not None:
                return f'{kind}: owned launch exited ({process.returncode})'
        if measuring_at is not None:
            now, current_ros = time.monotonic(), ros_now()
            for key, stream in stats.items():
                limit = LIMITS[key][2]
                if stream.last is None:
                    if now-measuring_at > limit:
                        return key+': no sample in observation window'
                elif now-stream.last > limit:
                    return key+': receive timeout'
                if stream.stamp is not None and not -.1 <= current_ros-stream.stamp <= limit:
                    return key+': stamp became stale/future'
        return None

    def telemetry(phase, converter, cv2):
        nonlocal seq, frames_emitted
        preview = None
        message = latest['image']
        if message is not None:
            picture = converter.imgmsg_to_cv2(message, desired_encoding='bgr8')
            height, width = picture.shape[:2]
            if width > 640:
                picture = cv2.resize(picture, (640, max(1, round(height*640/width))), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode('.jpg', picture, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            height, width = picture.shape[:2]
            preview = {'jpeg_base64': base64.b64encode(encoded.tobytes()).decode('ascii'),
                       'mime_type': 'image/jpeg', 'width': width, 'height': height,
                       'source_stamp_s': message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                       'source_frame_id': message.header.frame_id}
            frames_emitted += 1
        seq += 1
        bridge = latest['bridge_status'] or {}
        emit({'type': 'telemetry', 'seq': seq, 'run_id': run_id, 'phase': phase,
              'pi_unix_s': time.time(), 'elapsed_s': time.monotonic()-start,
              'streams': snapshot(), 'preview': preview, 'scan': latest['scan'],
              'ec': {key: bridge.get(key) for key in
                     ('ec_armed', 'ec_estop', 'ec_fault', 'link_ok', 'crc_errors', 'session', 'open_count')},
              'parameters_verified': parameters['verified'], 'command_observations': commands})

    def cleanup_owned():
        # Signal every group before waiting so a slow driver cannot prevent others from stopping.
        entries = []
        for kind, process, log in processes:
            item = {'name': kind, 'pid': process.pid, 'signals': [], 'errors': []}
            entries.append((process, log, item))
            cleanup.append(item)
        for sig, timeout in ((signal.SIGINT, 5.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
            for process, _, item in entries:
                try:
                    os.killpg(process.pid, sig)
                    item['signals'].append(signal.Signals(sig).name)
                except ProcessLookupError:
                    pass
                except Exception as exc:
                    item['errors'].append(str(exc))
            deadline = time.monotonic()+timeout
            for process, _, item in entries:
                try:
                    process.wait(timeout=max(.01, deadline-time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
                except Exception as exc:
                    item['errors'].append(str(exc))
            # A group may outlive its launch leader; check groups, not only Popen.poll().
            alive = False
            for process, _, item in entries:
                try:
                    os.killpg(process.pid, 0)
                    alive = True
                except ProcessLookupError:
                    pass
                except Exception:
                    alive = True
            if not alive:
                break
        for process, log, item in entries:
            item['returncode'] = process.poll()
            item['normal_exit'] = item['returncode'] == 0
            item['forced_sigkill'] = 'SIGKILL' in item['signals']
            # A crashed driver also disappears from its process group. Group
            # disappearance alone must never turn an abort into a passing run.
            if not item['normal_exit']:
                fail(f"{item['name']}: abnormal cleanup exit ({item['returncode']}); expected 0")
            if item['forced_sigkill']:
                fail(item['name']+': cleanup required SIGKILL')
            try:
                os.killpg(process.pid, 0)
                item['group_gone'] = False
                fail(item['name']+': owned process group still exists after cleanup')
            except ProcessLookupError:
                item['group_gone'] = True
            except Exception as exc:
                item['group_gone'] = False
                item['errors'].append(str(exc))
                fail(item['name']+': unable to verify owned process cleanup')
            if item['errors']:
                fail(item['name']+': cleanup errors')
            try:
                log.close()
            except Exception as exc:
                item['errors'].append('log close failed: '+str(exc))
                fail(item['name']+': log close failed: '+str(exc))

    try:
        os.set_blocking(stdout_fd, False)
        import cv2
        from cv_bridge import CvBridge
        import rclpy as ros
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from rcl_interfaces.srv import GetParameters
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, Imu, LaserScan
        from std_msgs.msg import Int8, String
        rclpy = ros
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous_handlers[number] = signal.signal(number, lambda n, f: interrupted.append(signal.Signals(n).name))
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = Node('wireless_static_observer_'+run_id[:8])
        def subscribe(kind, topic, callback):
            observers.append(node.create_subscription(kind, topic, guard(callback), qos_profile_sensor_data))
        subscribe(Image, '/camera/image_raw', image_cb)
        subscribe(LaserScan, '/scan', scan_cb)
        for key, topic in (('raw_odom', '/ec/wheel/odom'), ('wheel_odom', '/odom/wheel'), ('ekf', '/odometry/filtered')):
            subscribe(Odometry, topic, lambda message, k=key: odom_cb(k, message))
        subscribe(Imu, '/imu/data_raw', imu_cb)
        for key, topic in (('bridge_status', '/ec_bridge/status'), ('adapter_status', '/ec_adapter/status')):
            subscribe(String, topic, lambda message, k=key: status_cb(k, message))
        for topic in ('/cmd_vel_nav', '/cmd_vel_requested', '/cmd_vel', '/ec/cmd_vel'):
            subscribe(Twist, topic, lambda message, t=topic: command_cb(t, message))
        subscribe(Int8, '/lift_cmd', lambda message: command_cb('/lift_cmd', message))
        subscribe(String, '/ec/lift_cmd', lambda message: command_cb('/ec/lift_cmd', message))
        client = node.create_client(GetParameters, '/ec_adapter/get_parameters')
        commissioning = Path.home()/'dase7503-commissioning'
        launches = [(key, [str(commissioning/('start-'+key+'.sh'))]) for key in ('camera', 'lidar', 'ec')]
        launches.append(('ekf', ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args', '--params-file',
                         str(Path.home()/'dase7503-nav-main-20261105/ros2_ws/src/common/robot_bringup/config/ekf_ec_10hz.yaml')]))
        for kind, command in launches:
            log = (out/(kind+'.log')).open('w', encoding='utf-8')
            try:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            except Exception:
                log.close()
                raise
            processes.append((kind, process, log))
        converter = CvBridge()
        future = None
        next_telemetry = time.monotonic()
        startup_deadline = time.monotonic()+30.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            problem = health_problem()
            if problem:
                fail(problem)
            if failures:
                break
            if future is None and client.service_is_ready():
                future = client.call_async(GetParameters.Request(names=['enable_motion', 'enable_lift']))
            if future is not None and future.done() and not parameters['verified']:
                response = future.result()
                if response is None or len(response.values) != 2 or any(v.type != 1 or v.bool_value for v in response.values):
                    raise RuntimeError('Read-only parameter query did not confirm motion=false and lift=false')
                parameters.update(verified=True, enable_motion=False, enable_lift=False, verified_unix_s=time.time())
            now = time.monotonic()
            if measuring_at is None:
                bridge = latest['bridge_status'] or {}
                adapter = latest['adapter_status'] or {}
                ready = parameters['verified'] and all(s.count >= 2 for s in stats.values())
                ready = ready and all(s.last is not None and now-s.last <= LIMITS[k][2] for k, s in stats.items())
                ready = ready and bridge.get('ec_armed') is False and bridge.get('link_ok') is True and bridge.get('ec_fault') == 0
                ready = ready and adapter.get('telemetry_ready') is True
                if ready:
                    bridge_baseline.update(session=bridge.get('session'), open_count=bridge.get('open_count'))
                    measuring_at = now
                    for stream in stats.values():
                        stream.reset_window()
                elif now >= startup_deadline:
                    raise RuntimeError('30-second startup deadline: missing fresh streams or safe parameter/status confirmation')
            if now >= next_telemetry:
                telemetry('measuring' if measuring_at is not None else 'startup', converter, cv2)
                next_telemetry = time.monotonic()+1.
            if measuring_at is not None and time.monotonic()-measuring_at >= args.duration:
                measurement_ended_at = time.monotonic()
                measurement_complete = True
                break
        if not rclpy.ok():
            fail('ROS context exited')
        if measurement_complete:
            for key, value in snapshot().items():
                low, high, gap_limit = LIMITS[key]
                if value['count'] < 2 or value['window_rate_hz'] is None or not low <= value['window_rate_hz'] <= high:
                    fail(key+': observation-window frequency outside limits')
                if value['rate_hz'] is None or not low <= value['rate_hz'] <= high:
                    fail(key+': sample frequency outside limits')
                if value['max_receive_gap_s'] > gap_limit or value['first_delay_s'] is None or value['first_delay_s'] > gap_limit:
                    fail(key+': excessive initial/receive gap')
            problem = health_problem()
            if problem:
                fail(problem)
        else:
            fail('Requested observation duration did not complete')
    except Exception as exc:
        fail(type(exc).__name__+': '+str(exc))
        (out/'observer-error.log').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        # Preserve end-of-observation ages before expected teardown makes every stream stale.
        observation_finished_at = time.monotonic()
        try:
            end_snapshot = snapshot() if node is not None else {}
        except Exception as exc:
            end_snapshot = {}
            fail('Unable to capture final observation: '+str(exc))
        observation_passed = measurement_complete and not failures
        final_status = {key: latest[key] for key in ('bridge_status', 'adapter_status')}
        for number in previous_handlers:
            signal.signal(number, signal.SIG_IGN)
        try:
            cleanup_owned()
        except Exception as exc:
            fail('Unexpected cleanup failure: '+str(exc))
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                fail('Observer destruction failed: '+str(exc))
        if rclpy is not None:
            try:
                rclpy.try_shutdown()
            except Exception as exc:
                fail('ROS shutdown failed: '+str(exc))
        report = {'type': 'report', 'run_id': run_id, 'passed': measurement_complete and not failures,
                  'observation_passed': observation_passed,
                  'cleanup_passed': bool(cleanup) and all(item.get('normal_exit') is True
                      and item.get('group_gone') is True and not item.get('forced_sigkill')
                      and not item.get('errors') for item in cleanup),
                  'failures': failures, 'started_at': started_at,
                  'completed_at': datetime.datetime.now().astimezone().isoformat(),
                  'requested_duration_s': args.duration,
                  'measurement_duration_s': None if measuring_at is None else (measurement_ended_at or observation_finished_at)-measuring_at,
                  'streams': end_snapshot, 'thresholds': {k: {'min_hz': v[0], 'max_hz': v[1], 'max_gap_and_age_s': v[2]} for k, v in LIMITS.items()},
                  'static_feedback_limits': {'translation_m_s': .01, 'yaw_rad_s': .03},
                  'peak_feedback': peak, 'parameters': parameters, 'command_observations': commands,
                  'bridge_counter_baseline': bridge_baseline, 'final_status_before_cleanup': final_status,
                  'telemetry_lines': seq, 'jpeg_previews': frames_emitted, 'stdout_transport_failed': stream_broken,
                  'observer_command_publishers_created': 0, 'observer_motion_services_called': 0,
                  'cleanup': cleanup, 'boundary': BOUNDARY}
        (out/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        try:
            emit(report)
        except Exception as exc:
            fail(str(exc))
            report.update(passed=False, failures=failures, stdout_transport_failed=True)
            (out/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        ndjson.close()
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
