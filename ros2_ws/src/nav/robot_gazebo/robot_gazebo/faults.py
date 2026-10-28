"""Explicit, isolated Gazebo actuator faults; never run on a hardware domain.

The contact test deliberately drives into a generated obstacle with navigation
disabled. Its nonzero contact count is expected and is not a delivery audit.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=180)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error('Use an isolated ROS domain 1..232')
    directory = Path(args.output).expanduser().resolve()
    if directory.exists():
        parser.error('Output exists; use a new directory')
    directory.parent.mkdir(parents=True, exist_ok=True)
    scene = yaml.safe_load((Path(get_package_share_directory('robot_simulation'))/'config/course_layout.yaml').read_text())
    scene['obstacles_mm'] = [[2440., 880., 2480., 1140.]]
    scene_path = directory.parent/(directory.name+'-scene.yaml')
    with scene_path.open('x', encoding='utf-8') as stream:
        yaml.safe_dump(scene, stream)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    log_path = directory.parent/(directory.name+'.log')
    log = log_path.open('x', encoding='utf-8')
    process = subprocess.Popen(['ros2', 'launch', 'robot_gazebo', 'course.launch.py',
        'gui:=false', 'navigation:=false', 'mission:=false', 'domain_id:='+str(args.domain),
        'output_dir:='+str(directory), 'scene_file:='+str(scene_path)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = Node('gazebo_fault_observer')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    latest, stops, outputs = {}, [], []

    def pose(msg):
        latest.update(x=msg.pose.position.x, y=msg.pose.position.y, received=time.monotonic())

    def stop(request, response):
        stops.append(time.monotonic())
        response.success = True
        return response

    node.create_subscription(PoseStamped, '/simulation/pose', pose, 10)
    node.create_subscription(Twist, '/gazebo/cmd_vel', lambda m: outputs.append(
        (time.monotonic(), m.linear.x, m.linear.y, m.angular.z)), 10)
    node.create_service(Trigger, '/nav/emergency_stop', stop)

    def pump(seconds, command=None):
        end, sent = time.monotonic()+seconds, 0.
        while time.monotonic() < end and process.poll() is None:
            if command is not None and time.monotonic()-sent >= .05:
                publisher.publish(command)
                sent = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.02)

    def position():
        if not latest or time.monotonic()-latest['received'] > .5:
            raise RuntimeError('Missing fresh Gazebo truth observations')
        return latest['x'], latest['y']

    def summary():
        return json.loads((directory/'gazebo_summary.json').read_text())

    report = {'test': 'intentional simulated faults, not a nominal delivery', 'checks': {}}
    checks = report['checks']
    try:
        deadline = time.monotonic()+60
        while time.monotonic() < deadline and not latest and process.poll() is None:
            pump(.2)
        pump(1.)
        start = position()
        command = Twist(); command.linear.y = -.08
        pump(1., command)
        pump(1.2)  # No new command; wall-clock watchdog must send zero.
        settled = position(); since = time.monotonic()
        pump(1.)
        report['watchdog_motion_m'] = math.dist(start, settled)
        report['watchdog_settled_drift_m'] = math.dist(settled, position())
        checks['watchdog_was_exercised'] = report['watchdog_motion_m'] > .01
        checks['watchdog_stops_motion'] = report['watchdog_settled_drift_m'] < .001
        recent = [v for v in outputs if v[0] >= since]
        checks['watchdog_outputs_zero'] = bool(recent) and all(v[1:] == (0., 0., 0.) for v in recent)
        checks['no_contact_before_injection'] = summary()['unexpected_contact_samples'] == 0
        command = Twist(); command.linear.x = .12
        deadline = time.monotonic()+15
        while time.monotonic() < deadline and not stops and process.poll() is None:
            pump(.2, command)
        pump(1., command)  # Continue commanding to prove that contact stop is latched.
        settled, since = position(), time.monotonic()
        pump(1., command)
        contact = summary()
        report['contact_settled_drift_m'] = math.dist(settled, position())
        report['unexpected_contacts'] = contact['unexpected_contacts'][:3]
        checks['physical_contact_detected'] = contact['unexpected_contact_samples'] > 0
        checks['emergency_service_called'] = bool(stops)
        checks['contact_stop_latched'] = contact['emergency_latched']
        recent = [v for v in outputs if v[0] >= since]
        checks['contact_outputs_zero_despite_commands'] = bool(recent) and all(v[1:] == (0., 0., 0.) for v in recent)
        checks['contact_stops_motion'] = report['contact_settled_drift_m'] < .001

        # Kill only the bridge started by this launch, after validating its executable.
        match = re.search(r'\[parameter_bridge-\d+\]: process started with pid \[(\d+)\]', log_path.read_text())
        if not match:
            raise RuntimeError('Cannot identify this test launch bridge')
        pid = int(match.group(1))
        if b'/ros_gz_bridge/parameter_bridge' not in Path(f'/proc/{pid}/cmdline').read_bytes():
            raise RuntimeError('Bridge PID no longer belongs to the expected executable')
        os.kill(pid, signal.SIGTERM)
        try:
            process.wait(timeout=20)
            checks['bridge_failure_shuts_world_down'] = True
        except subprocess.TimeoutExpired:
            checks['bridge_failure_shuts_world_down'] = False
    except Exception as exc:
        report['error'] = str(exc)
        checks['completed_all_fault_checks'] = False
    finally:
        if process.poll() is None:
            publisher.publish(Twist())
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=8)
        node.destroy_node(); rclpy.try_shutdown(); log.close()
        report['passed'] = bool(checks) and all(checks.values())
        output = directory/'fault_checks.json' if directory.exists() else directory.parent/(directory.name+'-failure.json')
        output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
    raise SystemExit(0 if report['passed'] else 1)
