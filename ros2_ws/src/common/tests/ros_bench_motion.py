"""Bench runner end-to-end regression in LOCALHOST domain 210; no hardware."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


def run_case(name, out):
    node = Node('bench_fixture_'+name)
    raw = node.create_publisher(Odometry, '/ec/wheel/odom', qos_profile_sensor_data)
    wheel = node.create_publisher(Odometry, '/odom/wheel', 10)
    adapter = node.create_publisher(String, '/ec_adapter/status', 10)
    bridge = node.create_publisher(String, '/ec_bridge/status', 10)
    state = {'moving': name == 'already_moving', 'armed': True, 'first_motion': None, 'fault_at': None,
             'frozen_stamp': None, 'commands': [], 'calls': [], 'interrupt_sent': False}

    def command(message):
        now = time.monotonic()
        nonzero = message.linear.x != 0. or message.angular.z != 0.
        state['commands'].append((now, message.linear.x, message.angular.z))
        if nonzero:
            state['first_motion'] = state['first_motion'] or now
            state['moving'] = True

    node.create_subscription(Twist, '/cmd_vel', command, 10)

    def stop(request, response):
        state['calls'].append(('stop', None))
        state['moving'] = False
        response.success, response.message = True, 'simulated stop'
        return response

    def arm(request, response):
        state['calls'].append(('arm', request.data))
        assert request.data is False, 'Bench must never arm'
        response.success = name != 'disarm_failure'
        response.message = 'simulated disarm result'
        if response.success: state['armed'] = False
        return response

    node.create_service(Trigger, '/ec_bridge/stop', stop)
    node.create_service(SetBool, '/ec_bridge/set_arm', arm)

    def publish():
        stamp = node.get_clock().now().to_msg()
        active = state['moving'] and state['armed']
        for key, publisher in (('raw', raw), ('wheel', wheel)):
            if name == 'missing_wheel' and key == 'wheel': continue
            msg = Odometry()
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
            msg.header.stamp = stamp
            msg.pose.pose.orientation.w = 1.
            if active:
                msg.twist.twist.linear.x = .03
                if name == 'overspeed': msg.twist.twist.linear.x = .24
                if name == 'raw_overspeed' and key == 'raw': msg.twist.twist.linear.x = .314
                if name == 'yaw_overspeed': msg.twist.twist.angular.z = -.41
                if name == 'nonfinite' and key == 'raw': msg.twist.twist.linear.x = float('nan')
                if name == 'stale': msg.header.stamp.sec -= 2
                if name == 'duplicate':
                    state['frozen_stamp'] = state['frozen_stamp'] or stamp
                    msg.header.stamp = state['frozen_stamp']
                if name in ('overspeed', 'raw_overspeed', 'yaw_overspeed', 'nonfinite', 'stale', 'duplicate'):
                    state['fault_at'] = state['fault_at'] or time.monotonic()
            publisher.publish(msg)
        adapter.publish(String(data=json.dumps({'motion_enabled': True, 'telemetry_ready': True})))
        bridge.publish(String(data=json.dumps({'link_ok': True, 'ec_armed': state['armed'],
                                              'ec_estop': False, 'ec_fault': 0})))

    node.create_timer(.02, publish)
    path = out/(name+'.log')
    started = time.monotonic()
    with path.open('w') as log:
        duration = '1.0' if name in ('overspeed', 'raw_overspeed') else '.35'
        process = subprocess.Popen([sys.executable, '-c', 'from robot_bringup.bench_motion import main; main()',
                                    '--wheels-raised', '--forward', '.03', '--seconds', duration],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while process.poll() is None and time.monotonic()-started < 12.:
                rclpy.spin_once(node, timeout_sec=.01)
                if name in ('interrupt', 'terminate') and state['first_motion'] and not state['interrupt_sent']:
                    process.send_signal(signal.SIGINT if name == 'interrupt' else signal.SIGTERM)
                    state['interrupt_sent'] = True
            if process.poll() is None:
                raise AssertionError('bench child did not exit within its bounded run/cleanup')
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
            node.destroy_node()
    reports = [json.loads(line.split(' ', 1)[1]) for line in path.read_text().splitlines()
               if line.startswith('BENCH_REPORT ')]
    assert len(reports) == 1, path.read_text()
    report = reports[0]
    assert state['calls'].count(('stop', None)) == 1, state['calls']
    assert state['calls'].count(('arm', False)) == 1, state['calls']
    assert not any(call == ('arm', True) for call in state['calls'])
    nonzero = [c for c in state['commands'] if c[1] or c[2]]
    assert state['commands'] and state['commands'][-1][1:] == (0., 0.)
    if name == 'healthy':
        assert process.returncode == 0 and report['passed'] and nonzero, report
    else:
        assert process.returncode != 0 and not report['passed'], report
    if name in ('missing_wheel', 'already_moving'): assert not nonzero, nonzero
    if name not in ('missing_wheel', 'disarm_failure'):
        assert report['cleanup']['verified'], report
    else:
        assert not report['cleanup']['verified'], report
    zero_delay = None
    if state['fault_at']:
        assert report['abort_reason'], report
        assert nonzero[-1][0]-state['fault_at'] < .12, nonzero
        assert not report['pulse_completed'], report
        zeros = [c[0] for c in state['commands'] if c[0] >= state['fault_at'] and c[1:] == (0., 0.)]
        assert zeros, state['commands']
        zero_delay = min(zeros)-state['fault_at']
        assert zero_delay < .15, zero_delay
    return {'case': name, 'passed': True, 'seconds': round(time.monotonic()-started, 3),
            'child_exit': process.returncode, 'nonzero_messages': len(nonzero),
            'simulated_first_zero_after_fault_s': zero_delay,
            'service_calls': state['calls'], 'report': report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    args = parser.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID='210', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    rclpy.init(domain_id=210)
    results = []
    try:
        for name in ('healthy', 'overspeed', 'raw_overspeed', 'yaw_overspeed', 'stale',
                     'nonfinite', 'duplicate', 'missing_wheel', 'already_moving',
                     'interrupt', 'terminate', 'disarm_failure'):
            try: result = run_case(name, out)
            except Exception as exc: result = {'case': name, 'passed': False, 'error': str(exc)}
            results.append(result)
            print(json.dumps(result), flush=True)
            (out/'bench_ros.json').write_text(json.dumps(results, indent=2))
    finally:
        rclpy.try_shutdown()
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)


if __name__ == '__main__':
    main()
