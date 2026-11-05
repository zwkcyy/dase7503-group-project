"""Exercise the actual launch/YAML pipeline, including Chinese map paths."""
from contextlib import ExitStack
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from nav2_common.launch import RewrittenYaml
from robot_geometry.launch_parameters import ExpandedParameterFile


class LaunchParameterChecks(unittest.TestCase):
    def expanded(self, filename):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'参数.yaml'
            source.write_text(yaml.safe_dump({
                'map_server': {'ros__parameters': {'yaml_filename': filename}},
                'amcl': {'ros__parameters': {'initial_pose': {'x': 0.}}}},
                allow_unicode=True), encoding='utf-8')
            parameter_file = ParameterFile(RewrittenYaml(
                source_file=ExpandedParameterFile(str(source)),
                param_rewrites={'amcl.ros__parameters.initial_pose.x': '2.72'},
                convert_types=True), allow_substs=False)
            output = parameter_file.evaluate(LaunchContext())
            return yaml.safe_load(output.read_text())

    def test_chinese_path_and_spaces_survive(self):
        path = '/mnt/e/研究生/sem1课程/课程 地图.yaml'
        data = self.expanded(path)
        self.assertEqual(data['map_server']['ros__parameters']['yaml_filename'], path)

    def test_template_package_expression_and_initial_pose(self):
        data = self.expanded('$(find-pkg-share robot_navigation)/maps/field_map.yaml')
        expected = get_package_share_directory('robot_navigation')+'/maps/field_map.yaml'
        self.assertEqual(data['map_server']['ros__parameters']['yaml_filename'], expected)
        self.assertEqual(data['amcl']['ros__parameters']['initial_pose']['x'], 2.72)

    def test_plain_path_stays_unchanged(self):
        data = self.expanded('/tmp/course_map.yaml')
        self.assertEqual(data['map_server']['ros__parameters']['yaml_filename'], '/tmp/course_map.yaml')


class HardwareLaunchParameterChecks(unittest.TestCase):
    """Evaluate the real launch parameter pipeline without executing any node."""

    root = Path(__file__).resolve().parents[4]

    def package_share(self, name):
        for group in ('common', 'nav', 'vi'):
            path = self.root / 'ros2_ws/src' / group / name
            if path.is_dir():
                return str(path)
        raise AssertionError('Unexpected package requested by launch: ' + name)

    def parameters(self, relative_path, **overrides):
        spec = importlib.util.spec_from_file_location('hardware_launch_under_test', self.root / relative_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        captured = []

        def capture(**kwargs):
            captured.append(kwargs)
            return Node(**kwargs)

        context = LaunchContext()
        context.launch_configurations.update(overrides)
        with ExitStack() as stack:
            stack.enter_context(patch.object(module, 'Node', side_effect=capture))
            if hasattr(module, 'get_package_share_directory'):
                stack.enter_context(patch.object(module, 'get_package_share_directory', self.package_share))
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            for action in description.entities:
                if isinstance(action, OpaqueFunction):
                    action.execute(context)
        result = {}
        for node in captured:
            # Evaluate precisely the public launch_ros normalization/evaluation
            # pipeline used by Node. Parameter files are separate inputs; these
            # checks target the launch overrides which take precedence over them.
            dictionaries = [item for item in node.get('parameters', []) if isinstance(item, dict)]
            evaluated = evaluate_parameters(context, normalize_parameters(dictionaries))
            result[node['executable']] = {key: value for values in evaluated for key, value in values.items()}
        return result

    def test_composed_ec_baudrate_is_integer(self):
        for baudrate in ('115200', '230400'):
            with self.subTest(baudrate=baudrate):
                params = self.parameters('ros2_ws/src/common/robot_bringup/launch/ec_bridge.launch.py',
                                         start_bridge='true', baudrate=baudrate)
                self.assertIs(type(params['ec_serial_bridge']['baudrate']), int)
                self.assertEqual(params['ec_serial_bridge']['baudrate'], int(baudrate))

    def test_standalone_ec_baudrate_is_integer(self):
        for baudrate in ('115200', '230400'):
            with self.subTest(baudrate=baudrate):
                params = self.parameters('ros2_ws/src/common/ec_serial_bridge/launch/ec_serial_bridge.launch.py',
                                         baudrate=baudrate)['ec_serial_bridge']
                self.assertIs(type(params['baudrate']), int)
                self.assertEqual(params['baudrate'], int(baudrate))

    def test_composed_ec_permissions_are_boolean_and_disabled_by_default(self):
        path = 'ros2_ws/src/common/robot_bringup/launch/ec_bridge.launch.py'
        defaults = self.parameters(path)
        self.assertNotIn('ec_serial_bridge', defaults)
        for key in ('enable_motion', 'enable_lift'):
            self.assertIs(defaults['ec_adapter'][key], False)
        enabled = self.parameters(path, enable_motion='true', enable_lift='true')
        for key in ('enable_motion', 'enable_lift'):
            self.assertIs(enabled['ec_adapter'][key], True)

    def test_composed_ec_rejects_invalid_baudrate_before_node_start(self):
        for baudrate in ('0', '-1', 'not-an-integer', '115200.5'):
            with self.subTest(baudrate=baudrate), self.assertRaises((ValueError, TypeError)):
                self.parameters('ros2_ws/src/common/robot_bringup/launch/ec_bridge.launch.py',
                                start_bridge='true', baudrate=baudrate)

    def test_standalone_off_mode_stays_string_not_yaml_boolean(self):
        params = self.parameters('ros2_ws/src/common/ec_serial_bridge/launch/ec_serial_bridge.launch.py',
                                 lift_state_mode='off')['ec_serial_bridge']
        self.assertIs(type(params['lift_state_mode']), str)
        self.assertEqual(params['lift_state_mode'], 'off')
        self.assertIs(type(params['port']), str)
        self.assertIs(type(params['lift_cmd_mode']), str)

    def test_camera_scanner_and_window_preserve_declared_parameter_types(self):
        params = self.parameters('ros2_ws/src/vi/robot_vision/launch/vision_bringup.launch.py',
                                 use_sim_time='false', preview_hz='7', evidence_source='off')
        for executable in ('camera_node', 'qr_detector_node', 'qr_popup'):
            self.assertIs(params[executable]['use_sim_time'], False)
        self.assertIs(type(params['qr_popup']['preview_hz']), float)
        self.assertEqual(params['qr_popup']['preview_hz'], 7.0)
        self.assertEqual(params['qr_detector_node']['evidence_source'], 'off')
        self.assertIs(type(params['qr_detector_node']['evidence_source']), str)

    def test_ec_ekf_predicts_to_current_time_at_30hz(self):
        path = self.root / 'ros2_ws/src/common/robot_bringup/config/ekf_ec_10hz.yaml'
        config = yaml.safe_load(path.read_text(encoding='utf-8'))['ekf_filter_node']['ros__parameters']
        self.assertEqual(config['frequency'], 30.0)
        self.assertIs(config['predict_to_current_time'], True)
        self.assertEqual(config['odom0'], '/odom/wheel')
        self.assertEqual(config['imu0'], '/imu/data_raw')


if __name__ == '__main__':
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output/'launch_parameters.log').open('w') as log:
        suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(case)
                                   for case in (LaunchParameterChecks, HardwareLaunchParameterChecks))
        result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    report = {'tests': result.testsRun, 'passed': result.wasSuccessful(),
              'failures': [(str(test), reason) for test, reason in result.failures],
              'errors': [(str(test), reason) for test, reason in result.errors]}
    (output/'launch_parameters.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 1)
