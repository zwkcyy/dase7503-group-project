"""Bounded, low-speed wheel commissioning; requires wheels clear of the ground."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


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
    rclpy.init(args=[])
    node = rclpy.create_node('ec_bench_motion')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    received = {}

    def status(key, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                received[key] = (time.monotonic(), value)
        except (ValueError, TypeError):
            pass

    node.create_subscription(String, '/ec_adapter/status', lambda m: status('adapter', m), 10)
    node.create_subscription(String, '/ec_bridge/status', lambda m: status('bridge', m), 10)

    def ready():
        now = time.monotonic()
        if any(k not in received or now-received[k][0] > 1.5 for k in ('adapter', 'bridge')):
            return False
        adapter, bridge = received['adapter'][1], received['bridge'][1]
        return (adapter.get('motion_enabled') is True and adapter.get('telemetry_ready') is True
                and bridge.get('link_ok') is True and bridge.get('ec_armed') is True
                and bridge.get('ec_estop') is False and bridge.get('ec_fault') == 0
                and node.count_publishers('/cmd_vel') == 1 and pub.get_subscription_count() >= 1)

    try:
        deadline = time.monotonic()+5.
        while rclpy.ok() and time.monotonic() < deadline and not ready():
            rclpy.spin_once(node, timeout_sec=.05)
        if not ready():
            raise RuntimeError('No fresh armed EC/adapter state, or another /cmd_vel publisher exists. Stop NAV first.')
        command = Twist()
        command.linear.x, command.angular.z = args.forward, args.turn
        node.get_logger().warning('BENCH ONLY: bounded wheel pulse started; vy is zero')
        deadline = time.monotonic()+args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            if not ready():
                raise RuntimeError('EC feedback/arming changed or a competing controller appeared')
            pub.publish(command)
            end_cycle = time.monotonic()+.05
            while rclpy.ok() and time.monotonic() < end_cycle:
                rclpy.spin_once(node, timeout_sec=max(0., end_cycle-time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
