"""Bounded wheel commissioning; wheels must be clear of the ground.

Fixed feedback ceilings protect against abnormal motion; they are NOT calibration.
This tool never arms, retries a pulse, or permits a protection bypass.
"""
import argparse
import json
import math
import signal
import time


class BenchMonitor:
    """Pure telemetry gate; its duck-typed messages do not require a ROS runtime."""

    MAX_TRANSLATION = .10  # Twice the maximum permitted command, not a tolerance.
    MAX_YAW = .4
    MAX_FEEDBACK_AGE = .5
    MAX_STATUS_AGE = 1.5
    STATIONARY_TRANSLATION = .01
    STATIONARY_YAW = .03
    SOURCES = ('raw', 'wheel')

    def __init__(self):
        self.statuses, self.samples, self.issues, self.last_stamps = {}, {}, {}, {}
        self.peak = {key: {'translation_m_s': 0., 'yaw_rad_s': 0., 'samples': 0}
                     for key in self.SOURCES}
        self.running = False
        self.aborted = None
        self.stationary_counts = {key: 0 for key in self.SOURCES}

    def trip(self, reason):
        self.aborted = self.aborted or reason
        return reason

    def status(self, key, data, now):
        try:
            value = json.loads(data)
            if not isinstance(value, dict):
                raise ValueError('status is not an object')
            self.statuses[key] = (now, value)
        except (ValueError, TypeError):
            self.statuses.pop(key, None)
            if self.running:
                self.trip(f'{key}: malformed status')

    def odometry(self, key, message, now, ros_now):
        try:
            if not 0 <= message.header.stamp.nanosec < 1_000_000_000:
                raise ValueError('invalid timestamp nanoseconds')
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            pose, velocity = message.pose.pose, message.twist.twist
            values = [stamp, ros_now, now, pose.position.x, pose.position.y, pose.position.z,
                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                      velocity.linear.x, velocity.linear.y, velocity.linear.z,
                      velocity.angular.x, velocity.angular.y, velocity.angular.z,
                      *message.pose.covariance, *message.twist.covariance]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('nonfinite odometry')
            if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
                raise ValueError('expected odom/base_link frames')
            q = pose.orientation
            if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01:
                raise ValueError('non-unit orientation')
            age = ros_now-stamp
            if stamp <= 0 or not -.1 <= age <= self.MAX_FEEDBACK_AGE:
                raise ValueError('missing, stale or future timestamp')
            if stamp <= self.last_stamps.get(key, -math.inf):
                raise ValueError('timestamp did not advance')
            self.last_stamps[key] = stamp
            linear, yaw = math.hypot(velocity.linear.x, velocity.linear.y), abs(velocity.angular.z)
            self.peak[key]['samples'] += 1
            self.peak[key]['translation_m_s'] = max(self.peak[key]['translation_m_s'], linear)
            self.peak[key]['yaw_rad_s'] = max(self.peak[key]['yaw_rad_s'], yaw)
            self.samples[key] = {'received': now, 'fresh_since': now-max(0., age),
                                 'stamp': stamp, 'translation': linear, 'yaw': yaw}
            stationary = linear <= self.STATIONARY_TRANSLATION and yaw <= self.STATIONARY_YAW
            self.stationary_counts[key] = self.stationary_counts[key]+1 if stationary else 0
            self.issues.pop(key, None)
            if linear > self.MAX_TRANSLATION or yaw > self.MAX_YAW:
                self.trip(f'{key}: feedback exceeds fixed bench ceiling '
                          f'(translation={linear:.6f} m/s, yaw={yaw:.6f} rad/s)')
        except (ValueError, TypeError, AttributeError) as exc:
            self.stationary_counts[key] = 0
            self.issues[key] = str(exc)
            if self.running:
                self.trip(f'{key}: {exc}')

    def feedback_problem(self, now, ros_now):
        for key in self.SOURCES:
            if key in self.issues:
                return f'{key}: {self.issues[key]}'
            sample = self.samples.get(key)
            if sample is None:
                return f'{key}: no odometry'
            if (not 0 <= now-sample['fresh_since'] <= self.MAX_FEEDBACK_AGE
                    or not -.1 <= ros_now-sample['stamp'] <= self.MAX_FEEDBACK_AGE):
                return f'{key}: stale odometry or clock jump'
        return None

    def problem(self, now, ros_now, publishers=1, subscribers=1):
        reason = self.aborted or self.feedback_problem(now, ros_now)
        if reason is None:
            if not self.running and any(self.stationary_counts[k] < 2 for k in self.SOURCES):
                reason = 'Need two fresh stationary odometry samples from both sources before starting'
        if reason is None:
            for key in ('adapter', 'bridge'):
                if key not in self.statuses or not 0 <= now-self.statuses[key][0] <= self.MAX_STATUS_AGE:
                    reason = f'{key}: missing or stale status'
                    break
        if reason is None:
            adapter, bridge = self.statuses['adapter'][1], self.statuses['bridge'][1]
            if not (adapter.get('motion_enabled') is True and adapter.get('telemetry_ready') is True
                    and bridge.get('link_ok') is True and bridge.get('ec_armed') is True
                    and bridge.get('ec_estop') is False and bridge.get('ec_fault') == 0):
                reason = 'EC/adapter motion, telemetry or arming state is not ready'
            elif publishers != 1 or subscribers < 1:
                reason = 'competing /cmd_vel publisher or missing consumer'
        if reason and self.running:
            self.trip(reason)
        return reason

    def stopped_feedback(self, since, now, ros_now):
        bridge = self.statuses.get('bridge')
        disarmed = bool(bridge and bridge[0] >= since and 0 <= now-bridge[0] <= self.MAX_STATUS_AGE
                        and bridge[1].get('link_ok') is True and bridge[1].get('ec_armed') is False)
        stationary = self.feedback_problem(now, ros_now) is None and all(
            self.samples[key]['fresh_since'] >= since
            and self.stationary_counts[key] >= 2
            and self.samples[key]['translation'] <= self.STATIONARY_TRANSLATION
            and self.samples[key]['yaw'] <= self.STATIONARY_YAW
            for key in self.SOURCES)
        return disarmed, stationary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheels-raised', action='store_true', required=True)
    direction = parser.add_mutually_exclusive_group(required=True)
    direction.add_argument('--forward', type=float, default=0., help='Body x speed, at most +/-0.05 m/s')
    direction.add_argument('--turn', type=float, default=0., help='Yaw rate, at most +/-0.2 rad/s')
    parser.add_argument('--seconds', type=float, default=1.)
    args = parser.parse_args(argv)
    if (not all(math.isfinite(v) for v in (args.forward, args.turn, args.seconds))
            or abs(args.forward) > .05 or abs(args.turn) > .2 or not .1 <= args.seconds <= 2.):
        parser.error('Use |forward| <= 0.05, |turn| <= 0.2 and duration 0.1-2 seconds')
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger

    # Do not raise inside an executor callback/wait: that can leave its task
    # iterator unable to process cleanup service responses. Check this flag
    # after each bounded spin instead, retaining the context for stop/disarm.
    interrupted = []
    def interrupt(signum, frame):
        interrupted.append(signal.Signals(signum).name)
    previous = {number: signal.signal(number, interrupt) for number in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ec_bench_motion')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    stop = node.create_client(Trigger, '/ec_bridge/stop')
    disarm = node.create_client(SetBool, '/ec_bridge/set_arm')
    monitor = BenchMonitor()
    ros_now = lambda: node.get_clock().now().nanoseconds/1e9
    for key, topic in (('adapter', '/ec_adapter/status'), ('bridge', '/ec_bridge/status')):
        node.create_subscription(String, topic, lambda m, k=key: monitor.status(k, m.data, time.monotonic()), 10)
    for key, topic in (('raw', '/ec/wheel/odom'), ('wheel', '/odom/wheel')):
        node.create_subscription(Odometry, topic,
            lambda m, k=key: monitor.odometry(k, m, time.monotonic(), ros_now()), qos_profile_sensor_data)

    def problem():
        if interrupted:
            return monitor.trip('Interrupted by '+interrupted[0])
        reason = monitor.problem(time.monotonic(), ros_now(), node.count_publishers('/cmd_vel'),
                                 pub.get_subscription_count())
        if reason is None and not (stop.service_is_ready() and disarm.service_is_ready()):
            reason = 'stop/disarm services unavailable'
            if monitor.running:
                monitor.trip(reason)
        return reason

    report = {'command': {'forward_m_s': args.forward, 'turn_rad_s': args.turn, 'seconds': args.seconds},
              'feedback_ceilings': {'translation_m_s': .10, 'yaw_rad_s': .4,
                                    'meaning': 'fixed abnormal-motion protection, not calibration'},
              'command_messages': 0, 'pulse_completed': False, 'abort_reason': None}
    cleanup = {'stop': {'requested': False}, 'disarm': {'requested': False}, 'verified': False}
    try:
        deadline = time.monotonic()+5.
        while rclpy.ok() and time.monotonic() < deadline and problem():
            if monitor.aborted or interrupted:
                break
            rclpy.spin_once(node, timeout_sec=.02)
        reason = problem()
        if reason:
            raise RuntimeError(reason)
        monitor.running = True
        command = Twist()
        command.linear.x, command.angular.z = args.forward, args.turn
        node.get_logger().warning('BENCH ONLY: bounded wheel pulse started; vy is zero; feedback limits active')
        deadline = time.monotonic()+args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            reason = problem()
            if reason:
                raise RuntimeError(reason)
            pub.publish(command)
            report['command_messages'] += 1
            end_cycle = min(deadline, time.monotonic()+.05)
            while rclpy.ok() and time.monotonic() < end_cycle:
                rclpy.spin_once(node, timeout_sec=max(0., min(.01, end_cycle-time.monotonic())))
                reason = problem()
                if reason:
                    pub.publish(Twist())
                    raise RuntimeError(reason)
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped before pulse completion')
        report['pulse_completed'] = True
    except (KeyboardInterrupt, Exception) as exc:
        report['abort_reason'] = monitor.aborted or str(exc) or type(exc).__name__
    finally:
        monitor.running = False
        # A second interrupt must not abort this bounded cleanup sequence.
        for number in previous:
            signal.signal(number, signal.SIG_IGN)
        since = time.monotonic()
        try:
            if not rclpy.ok():
                raise RuntimeError('ROS context unavailable; cannot verify stop/disarm')
            pub.publish(Twist())
            futures = {}
            # Dispatch both at once: a slow stop reply must not delay disarm.
            for key, client, request in (('stop', stop, Trigger.Request()),
                                         ('disarm', disarm, SetBool.Request(data=False))):
                if client.service_is_ready():
                    try:
                        futures[key] = client.call_async(request)
                        cleanup[key]['requested'] = True
                    except Exception as exc:
                        cleanup[key]['error'] = str(exc)
                else:
                    cleanup[key]['error'] = 'service unavailable'
            stable = None
            while rclpy.ok() and time.monotonic()-since < 3.:
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.02)
                for key, future in futures.items():
                    if future.done() and 'success' not in cleanup[key] and 'error' not in cleanup[key]:
                        try:
                            response = future.result()
                            cleanup[key].update(success=response.success, message=response.message)
                        except Exception as exc:
                            cleanup[key]['error'] = str(exc)
                disarmed, stationary = monitor.stopped_feedback(since, time.monotonic(), ros_now())
                cleanup.update(disarmed_feedback=disarmed, stationary_telemetry=stationary)
                confirmed = disarmed and stationary and all(cleanup[k].get('success') is True
                                                            for k in ('stop', 'disarm'))
                stable = (stable or time.monotonic()) if confirmed else None
                if stable is not None and time.monotonic()-stable >= .2:
                    cleanup['verified'] = True
                    break
            for key in futures:
                if 'success' not in cleanup[key] and 'error' not in cleanup[key]:
                    cleanup[key]['error'] = 'response timeout'
        except Exception as exc:
            cleanup['error'] = str(exc)
        finally:
            report['abort_reason'] = report['abort_reason'] or monitor.aborted
            report.update(feedback=monitor.peak, cleanup=cleanup)
            report['passed'] = report['pulse_completed'] and report['abort_reason'] is None and cleanup['verified']
            print('BENCH_REPORT '+json.dumps(report, allow_nan=False), flush=True)
            node.destroy_node()
            rclpy.try_shutdown()
            for number, handler in previous.items():
                signal.signal(number, handler)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
