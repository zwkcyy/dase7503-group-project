"""Exercise the actual launch/YAML pipeline, including Chinese map paths."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.descriptions import ParameterFile
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


if __name__ == '__main__':
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output/'launch_parameters.log').open('w') as log:
        result = unittest.TextTestRunner(stream=log, verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(LaunchParameterChecks))
    report = {'tests': result.testsRun, 'passed': result.wasSuccessful(),
              'failures': [(str(test), reason) for test, reason in result.failures],
              'errors': [(str(test), reason) for test, reason in result.errors]}
    (output/'launch_parameters.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 1)
