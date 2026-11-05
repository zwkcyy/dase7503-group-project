"""Offline Gazebo command-boundary checks; no ROS, Gazebo, or hardware startup."""
import ast
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC/'nav/robot_gazebo'))
from robot_gazebo.drive_policy import (
    admitted_velocity, command_audit, observe_velocity, validate_drive_mode, velocity_metrics)


class Twist:
    def __init__(self, x=0., y=0., z=0.):
        self.linear = SimpleNamespace(x=x, y=y, z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=z)


def command_boundary():
    # Execute the real small IO methods with fake messages; do not import or
    # start the ROS/Gazebo runtime in this offline suite.
    source = ast.parse((SRC/'nav/robot_gazebo/robot_gazebo/io.py').read_text(encoding='utf-8'))
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'GazeboIO')
    selected = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                ('on_command', 'publish_velocity', 'on_transport_command')]
    namespace = dict(Twist=Twist, math=math, time=time, admitted_velocity=admitted_velocity,
                     observe_velocity=observe_velocity)
    exec(compile(ast.Module(body=selected, type_ignores=[]), '<GazeboIO boundary>', 'exec'), namespace)
    return type('Boundary', (), {n.name: namespace[n.name] for n in selected})


Boundary = command_boundary()


class GazeboDriveModeTests(unittest.TestCase):
    def boundary(self, mode='differential'):
        node = Boundary()
        node.drive_mode = mode
        node.lock = threading.Lock()
        node.command_metrics = {k: velocity_metrics() for k in ('input', 'output', 'gazebo_transport')}
        node.rejected_velocity_commands = 0
        node.transport_velocity_fault = None
        node.event = Mock()
        node.get_logger = Mock(return_value=Mock())
        node.velocity_pub = Mock()
        return node

    def test_invalid_mode_is_rejected(self):
        for mode in ('', 'diff', None):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                validate_drive_mode(mode)

    def test_differential_rejects_whole_command(self):
        for y in (.1, -.1, 1e-15):
            self.assertEqual(admitted_velocity((.2, y, .3), 'differential'),
                             ((0., 0., 0.), 'lateral_velocity_not_supported'))

    def test_differential_accepts_forward_reverse_rotation(self):
        for values in ((.15, 0., 0.), (-.08, 0., 0.), (0., 0., .4), (.15, -0., -.4)):
            self.assertEqual(admitted_velocity(values, 'differential'), (values, ''))

    def test_omni_limits_unchanged(self):
        self.assertEqual(admitted_velocity((.5, -.5, 2.), 'omni'), ((.4, -.3, 1.5), ''))

    def test_nonfinite_command_stops_in_both_modes(self):
        for mode in ('omni', 'differential'):
            for index in range(3):
                for value in (math.nan, math.inf, -math.inf):
                    values = [.1, 0., .2]
                    values[index] = value
                    self.assertEqual(admitted_velocity(values, mode), ((0., 0., 0.), 'nonfinite_velocity'))

    def test_input_rejection_keeps_original_and_records_evidence(self):
        node, message = self.boundary(), Twist(.2, .1, .3)
        node.on_command(message)
        self.assertEqual((node.command.linear.x, node.command.linear.y, node.command.angular.z), (0., 0., 0.))
        self.assertEqual((message.linear.x, message.linear.y, message.angular.z), (.2, .1, .3))
        self.assertEqual(node.rejected_velocity_commands, 1)
        self.assertEqual(node.command_metrics['input']['lateral_samples'], 1)
        self.assertEqual(node.event.call_args.kwargs['reason'], 'lateral_velocity_not_supported')

    def test_final_publication_also_rejects_lateral_motion(self):
        node = self.boundary()
        node.publish_velocity(Twist(.2, .1, .3))
        output = node.velocity_pub.publish.call_args.args[0]
        self.assertEqual((output.linear.x, output.linear.y, output.angular.z), (0., 0., 0.))
        self.assertEqual(node.command_metrics['output']['max_abs_vy_m_s'], 0.)

    def test_omni_publication_keeps_lateral_command(self):
        node = self.boundary('omni')
        node.on_command(Twist(.1, -.2, .3))
        node.publish_velocity(node.command)
        self.assertEqual(node.velocity_pub.publish.call_args.args[0].linear.y, -.2)
        self.assertEqual(node.rejected_velocity_commands, 0)

    def test_native_gazebo_bypass_is_detected(self):
        node = self.boundary()
        node.on_transport_command(Twist(.2, .1, .3))
        self.assertEqual(node.transport_velocity_fault, [.2, .1, .3])
        self.assertEqual(node.command_metrics['gazebo_transport']['lateral_samples'], 1)

    def test_native_nonfinite_command_is_detected(self):
        node = self.boundary()
        node.on_transport_command(Twist(math.nan, 0., 0.))
        self.assertEqual(node.transport_velocity_fault, ['nan', 0., 0.])
        self.assertEqual(node.command_metrics['gazebo_transport']['nonfinite_samples'], 1)

    def nominal_summary(self):
        streams = {k: velocity_metrics() for k in ('input', 'output', 'gazebo_transport')}
        for metrics in streams.values():
            observe_velocity(metrics, (.1, 0., .2))
        return dict(drive_mode='differential', command_metrics=streams, rejected_velocity_commands=0)

    def test_differential_audit_requires_nonzero_transport_observation(self):
        data = self.nominal_summary()
        self.assertTrue(all(command_audit(data, 'differential').values()))
        data['command_metrics']['gazebo_transport'] = velocity_metrics()
        self.assertFalse(all(command_audit(data, 'differential').values()))

    def test_gazebo_lateral_bypass_fails_audit(self):
        data = self.nominal_summary()
        observe_velocity(data['command_metrics']['gazebo_transport'], (0., .1, 0.))
        checks = command_audit(data, 'differential')
        self.assertFalse(checks['differential_gazebo_transport_vy_zero'])

    def test_differential_missing_evidence_fails(self):
        self.assertFalse(all(command_audit({}, 'differential').values()))

    def test_existing_omni_audits_need_no_new_evidence(self):
        self.assertEqual(command_audit({}, 'omni'), {})

    def test_omni_override_cannot_hide_recorded_differential_mode(self):
        self.assertFalse(all(command_audit(self.nominal_summary(), 'omni').values()))

    def test_launch_threads_drive_mode_to_all_three_consumers(self):
        source = ast.parse((SRC/'nav/robot_gazebo/launch/course.launch.py').read_text(encoding='utf-8'))
        settings = []
        for node in ast.walk(source):
            if isinstance(node, ast.Dict):
                settings.extend(value for key, value in zip(node.keys, node.values)
                                if isinstance(key, ast.Constant) and key.value == 'drive_mode')
        forwarded = [n for n in settings if isinstance(n, ast.Subscript)
                     and isinstance(n.value, ast.Name) and n.value.id == 'args'
                     and isinstance(n.slice, ast.Constant) and n.slice.value == 'drive_mode']
        self.assertEqual(len(forwarded), 3)


if __name__ == '__main__':
    unittest.main()
