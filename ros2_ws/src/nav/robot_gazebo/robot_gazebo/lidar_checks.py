"""Isolated physical braking checks through the REAL smoother and motion guard.

Test fixtures deliberately start under rack A for loaded cases. Lifting uses
the physical joint and rack contacts; no attachment/pose teleport is used.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
from std_srvs.srv import SetBool
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_geometry.directional_safety import rear_visibility


def run_case(directory, domain, loaded=False, reverse=False, old_height=False):
    scene = yaml.safe_load((Path(get_package_share_directory('robot_simulation'))/'config/course_layout.yaml').read_text())
    physics = yaml.safe_load((Path(get_package_share_directory('robot_gazebo'))/'config/physics.yaml').read_text())
    scene['initial_pose_mm_rad'] = [2600., 1000., math.pi]
    scene['obstacles_mm'] = [[2940., 975., 2955., 1025.]] if reverse else [[2080., 975., 2095., 1025.]]
    if loaded:
        scene['racks'][0].update(x_mm=2600., y_mm=1000., yaw_rad=math.pi)
        # This obstacle is outside the empty chassis, inside the cargo sweep.
        if not reverse:
            scene['obstacles_mm'] = [[2080., 795., 2095., 810.]]
    physics['start_qr_y_mm'] = 1700.
    if old_height:
        physics['laser_z_mm'] = 70.
    inputs = {}
    for key, value in [('scene', scene), ('physics', physics)]:
        path = directory.parent/(directory.name+'-'+key+'.yaml')
        with path.open('x') as stream:
            yaml.safe_dump(value, stream)
        inputs[key] = path
    log = (directory.parent/(directory.name+'.log')).open('x')
    process = subprocess.Popen(['ros2', 'launch', 'robot_gazebo', 'course.launch.py', 'gui:=false',
        'mission:=false', 'domain_id:='+str(domain), 'output_dir:='+str(directory),
        'scene_file:='+str(inputs['scene']), 'physics_file:='+str(inputs['physics'])],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = Node('lidar_braking_observer')
    cmd_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    phase_pub = node.create_publisher(String, '/mission/status', 10)
    lift_pub = node.create_publisher(Int8, '/lift_cmd', 10)
    envelope = node.create_client(SetBool, '/nav/set_loaded')
    latest, safety, scans, lifts, outputs = {}, [], [], [], []
    phase = 'LIFT_RACK' if loaded else ('EXIT_DESTINATION' if reverse else 'NAV_TO_RACK')
    command = Twist()
    node.create_subscription(PoseStamped, '/simulation/pose', lambda m: latest.update(
        x=m.pose.position.x, y=m.pose.position.y, at=time.monotonic()), 10)
    node.create_subscription(String, '/nav/motion_safety', lambda m: safety.append(json.loads(m.data)), 10)
    node.create_subscription(LaserScan, '/scan_filtered', lambda m: scans.append(m), qos_profile_sensor_data)
    node.create_subscription(Int8, '/lift_state', lambda m: lifts.append(m.data), 10)
    node.create_subscription(Twist, '/gazebo/cmd_vel', lambda m: outputs.append(
        (time.monotonic(), m.linear.x, m.linear.y, m.angular.z)), 10)

    def pump(seconds):
        until, last = time.monotonic()+seconds, 0.
        while time.monotonic() < until and process.poll() is None:
            if time.monotonic()-last >= .05:
                phase_pub.publish(String(data=json.dumps({'state': phase, 'rack_index': 0, 'rack_count': 4})))
                cmd_pub.publish(command)
                last = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.01)

    def position():
        if not latest or time.monotonic()-latest['at'] > .5:
            raise RuntimeError('No fresh independent Gazebo pose')
        return latest['x'], latest['y']

    result = {'loaded': loaded, 'reverse': reverse, 'laser_z_mm': physics['laser_z_mm'], 'checks': {}}
    checks = result['checks']
    try:
        until = time.monotonic()+60
        while time.monotonic() < until and (not scans or not envelope.service_is_ready()):
            pump(.2)
        pump(2.)
        if not scans:
            raise RuntimeError('No rendered scan received')
        if loaded:
            lift_pub.publish(Int8(data=1))
            until = time.monotonic()+15
            while time.monotonic() < until and 1 not in lifts:
                pump(.2)
            if 1 not in lifts:
                raise RuntimeError('Physical rack lift did not finish')
            future = envelope.call_async(SetBool.Request(data=True))
            until = time.monotonic()+5
            while not future.done() and time.monotonic() < until:
                pump(.1)
            if not future.done() or not future.result().success:
                raise RuntimeError('Both costmaps did not accept the loaded footprint')
        phase = 'EXIT_RACK' if reverse else 'NAV_TO_DESTINATION'
        pump(.5)
        start = position()
        command.linear.x = -.08 if reverse else .25
        pump(10. if reverse else 7.)
        settled, since = position(), time.monotonic()
        pump(1.)  # Keep requesting motion: stop must persist ahead of the obstacle.
        summary = json.loads((directory/'gazebo_summary.json').read_text())
        result.update(travel_m=math.dist(start, settled), settled_drift_m=math.dist(settled, position()),
                      final_position_m=list(position()), unexpected_contacts=summary['unexpected_contact_samples'],
                      guard_reasons=sorted(set(v['reason'] for v in safety)),
                      central_rear_visible=rear_visibility(scans[-1]),
                      scan_unknown_rays=sum(math.isnan(v) for v in scans[-1].ranges),
                      loaded_rack_height_m=summary['poses']['rack_A']['z'] if loaded else None)
        recent = [v for v in outputs if v[0] >= since]
        checks['rendered_scan_received'] = len(scans) > 10
        checks['contact_monitor_active'] = summary['contact_samples_received'] > 100
        checks['zero_contacts'] = summary['unexpected_contact_samples'] == 0
        checks['no_contact_emergency'] = not summary['emergency_latched']
        checks['stopped_despite_requests'] = bool(recent) and all(v[1:] == (0., 0., 0.) for v in recent)
        checks['physically_stopped'] = result['settled_drift_m'] < .001
        checks['correct_stop_reason'] = ('central_rear_scan_unknown' if old_height else 'obstacle_in_braking_envelope') in result['guard_reasons']
        checks['motion_or_blind_refusal_exercised'] = result['travel_m'] < .002 if old_height else result['travel_m'] > .02
        if loaded:
            checks['real_rack_carried'] = result['loaded_rack_height_m'] > .005
    except Exception as exc:
        result['error'] = str(exc)
        checks['completed'] = False
    finally:
        command = Twist(); pump(.3)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=8)
        node.destroy_node(); log.close()
    result['passed'] = bool(checks) and all(checks.values())
    (directory.parent/(directory.name+'-check.json')).write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=181)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error('Use an isolated domain 1..232')
    directory = Path(args.output).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    rclpy.init(args=[])
    results = []
    try:
        cases = [('empty-forward', False, False, False), ('empty-reverse', False, True, False),
                 ('loaded-forward', True, False, False), ('loaded-reverse', True, True, False),
                 ('old-height-blind-refusal', False, True, True)]
        for name, loaded, reverse, old in cases:
            result = run_case(directory/name, args.domain, loaded, reverse, old)
            results.append(result)
            print(json.dumps({'case': name, **result}), flush=True)
    finally:
        rclpy.try_shutdown()
    report = {'passed': all(r['passed'] for r in results), 'cases': results,
              'scope': 'Visible physical obstacles and central blind-sector refusal; narrow occluded dynamic targets remain unproven.'}
    (directory/'lidar_checks.json').write_text(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
