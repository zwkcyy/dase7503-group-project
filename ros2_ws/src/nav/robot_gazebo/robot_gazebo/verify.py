"""Explicit full Gazebo verification. Normal startup never invokes tests."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .audit import audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=179)
    parser.add_argument('--scene-file'); parser.add_argument('--physics-file')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--timeout', type=float, default=600.)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232 or not 0 < args.timeout <= 1800:
        parser.error('Use an isolated domain 1..232 and wall timeout 0..1800 seconds')
    directory = Path(args.output).expanduser().resolve()
    if directory.exists(): parser.error('Output directory already exists; use a new run directory')
    directory.parent.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    command = ['ros2', 'launch', 'robot_gazebo', 'course.launch.py', 'gui:='+str(args.gui).lower(),
               'domain_id:='+str(args.domain), 'output_dir:='+str(directory)]
    for key in ('scene_file', 'physics_file'):
        value = getattr(args, key)
        if value: command.append(key+':='+str(Path(value).resolve(strict=True)))
    log = (directory.parent/(directory.name+'.log')).open('x', encoding='utf-8')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=[])
    node = Node('gazebo_verification_observer')
    bridge, state, phases, codes, frames = CvBridge(), {}, [], [], []

    def status(msg):
        value = json.loads(msg.data)
        changed = (value['state'], value['rack_index']) != (state.get('state'), state.get('rack_index'))
        state.clear(); state.update(value)
        if changed:
            phases.append(dict(value))
            print(json.dumps(value), flush=True)
            if frames:
                cv2.imwrite(str(directory/f'camera-{len(phases):02d}-{value["state"]}.png'), frames[-1])

    def picture(msg):
        frames[:] = [bridge.imgmsg_to_cv2(msg, 'bgr8')]

    node.create_subscription(String, '/mission/status', status, 10)
    node.create_subscription(String, '/qr/data', lambda m: codes.append(m.data), 10)
    node.create_subscription(Image, '/camera/image_raw', picture, qos_profile_sensor_data)
    started, terminal = time.monotonic(), None
    result = None
    try:
        while time.monotonic()-started < args.timeout and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
            if state.get('state') in ('COMPLETE', 'ERROR'):
                terminal = terminal or time.monotonic()
                if time.monotonic()-terminal >= 2.:
                    break
        if not directory.exists():
            raise RuntimeError(f'Gazebo did not start; see {log.name}')
        result = {'state': dict(state), 'phases': phases, 'codes': codes,
                  'wall_seconds': time.monotonic()-started, 'launch_returncode': process.poll()}
        (directory/'observer.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        if frames: cv2.imwrite(str(directory/'camera-final.png'), frames[-1])
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=8)
        node.destroy_node(); rclpy.try_shutdown(); log.close()
    report = audit(directory)
    print(json.dumps({'directory': str(directory), **report}, indent=2), flush=True)
    raise SystemExit(0 if report['passed'] else 1)
