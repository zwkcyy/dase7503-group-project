"""Short standalone simulated actuator probe; no Nav2, EC, or physical hardware."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from robot_simulation.planar_base import PlanarBase


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=208)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error('Use an isolated nonzero domain')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    share = Path(get_package_share_directory('robot_simulation'))
    rclpy.init(args=['--ros-args', '-p', 'drive_mode:=differential', '-p',
        'world_file:='+str(share/'config/demo_world.yaml'), '-p',
        'geometry_file:='+str(share/'config/demo_geometry.yaml')], domain_id=args.domain)
    node = PlanarBase()
    report = {'domain': args.domain, 'source': 'standalone_actual_planar_base_node', 'passed': False, 'cases': []}
    try:
        for name, values, moves in [('forward', (.05, 0., 0.), True),
                                    ('pure_y_rejected', (0., .1, 0.), False),
                                    ('mixed_y_rejected', (.1, .01, .2), False)]:
            before = list(node.pose)
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = values
            node.command(command)
            end = time.monotonic()+.25
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.03)
            distance = math.hypot(node.pose[0]-before[0], node.pose[1]-before[1])
            yaw_change = abs(math.atan2(math.sin(node.pose[2]-before[2]), math.cos(node.pose[2]-before[2])))
            passed = distance > .001 if moves else distance < 1e-12 and yaw_change < 1e-12
            report['cases'].append({'case': name, 'requested': values, 'distance_m': distance,
                                    'yaw_change_rad': yaw_change, 'applied': list(node.velocity), 'passed': passed})
            assert passed, report['cases'][-1]
        assert node.metrics['rejected_lateral_commands'] == 2, node.metrics
        assert node.metrics['maximum_applied_abs_vy'] == 0.0, node.metrics
        report.update(passed=True, metrics=dict(node.metrics))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        output = Path(args.output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
