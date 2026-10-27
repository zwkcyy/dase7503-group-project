"""Inject live failures while real Nav2/docking is moving the planar robot."""
import json
import math
from pathlib import Path
import sys
import time
import os
import re
import signal
import uuid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from ros_integration import run_case, spin_until, call, mission_check


def stop_check(fault, log_path):
    def check(node):
        states, poses, commands, racks, safety = [], [], [], [], []
        node.create_subscription(String, '/mission/status', lambda m: states.append(json.loads(m.data)), 10)
        node.create_subscription(PoseStamped, '/simulation/pose', lambda m: poses.append((time.monotonic(), m.pose.position.x, m.pose.position.y)), 10)
        node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append((time.monotonic(), m.linear.x, m.linear.y, m.angular.z)), 10)
        node.create_subscription(String, '/simulation/racks', lambda m: racks.append(json.loads(m.data)),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(String, '/nav/motion_safety', lambda m: safety.append(json.loads(m.data)), 10)
        phase = {'freeze_scan': 'NAV_TO_RACK', 'freeze_wheel': 'NAV_TO_DESTINATION',
                 'guard_shutdown': 'NAV_TO_DESTINATION',
                 'emergency_stop': 'DOCK_RACK'}[fault]
        spin_until(node, lambda: states and states[-1]['state'] == phase and commands and
                   math.hypot(commands[-1][1], commands[-1][2]) > .05, 70)
        cargo_before = racks[-1]['carried']
        injected_at = time.monotonic()
        if fault == 'guard_shutdown':
            # Target only the child whose PID was reported by this test's launch.
            matches = re.findall(r'\[motion_guard-\d+\]: process started with pid \[(\d+)\]', log_path.read_text())
            assert len(matches) == 1, 'Could not identify this launch\'s guard process'
            os.kill(int(matches[0]), signal.SIGINT)
        elif fault == 'emergency_stop':
            client = node.create_client(Trigger, '/nav/emergency_stop')
            assert call(node, client, Trigger.Request()).success
        else:
            client = node.create_client(SetParameters, '/planar_base/set_parameters')
            request = SetParameters.Request(parameters=[Parameter(fault, value=True).to_parameter_msg()])
            assert call(node, client, request).results[0].successful
        spin_until(node, lambda: time.monotonic()-injected_at > 2.5, 4)
        stopped = [p for p in poses if p[0] > injected_at+1.5]
        assert len(stopped) >= 5, 'Missing physical pose samples'
        drift = max(math.hypot(p[1]-stopped[0][1], p[2]-stopped[0][2]) for p in stopped)
        assert drift < .002, f'Robot kept moving after fault: {drift} m'
        assert all(max(abs(v) for v in cmd[1:]) < .001 for cmd in commands if cmd[0] > injected_at+1.), 'Nonzero commands after stop deadline'
        if fault == 'guard_shutdown':
            assert commands[-1][0] > injected_at and max(abs(v) for v in commands[-1][1:]) < .001, 'Guard did not send final zero'
        assert racks[-1]['carried'] == cargo_before, 'Fault unexpectedly lowered/released cargo'
        assert racks[-1]['metrics']['actual_collisions'] == 0
        assert racks[-1]['metrics']['blocked_collision_commands'] == 0
        expected = {'freeze_scan': 'stale_or_invalid_laser', 'freeze_wheel': 'stale_or_invalid_wheel',
                    'guard_shutdown': 'latched_stop',
                    'emergency_stop': 'latched_stop'}[fault]
        assert any(s['reason'] == expected for s in safety), safety
        return {'fault': fault, 'injected_in_phase': phase, 'cargo_preserved': cargo_before,
                'stationary_drift_m': drift, 'zero_commands_after_s': 1., 'collision_count': 0}
    return check


def main(output, selected=None):
    output.mkdir(parents=True, exist_ok=True)
    results = []
    cases = ('freeze_scan', 'freeze_wheel', 'emergency_stop', 'wrong_qr', 'missing_qr', 'guard_shutdown')
    if selected:
        assert selected in cases
    for i, fault in enumerate(cases):
        if selected and fault != selected:
            continue
        directory = output/(fault+'-'+uuid.uuid4().hex[:8])
        args = ['rviz:=false', 'show_field:=false', f'output_dir:={directory}']
        if fault in ('wrong_qr', 'missing_qr'):
            args.append(f'image_scenario:={fault}')
            check = mission_check('ERROR', 60, False, True)
        else:
            check = stop_check(fault, output/('collision_'+fault+'.log'))
        result = run_case('collision_'+fault, 'course_demo.launch.py', args, 131+i, output, check)
        result['artifact_dir'] = str(directory)
        results.append(result)
        (output/'collision_faults.json').write_text(json.dumps(results, indent=2))
    if not all(r['passed'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main(Path(sys.argv[1]), sys.argv[2] if len(sys.argv) > 2 else None)
