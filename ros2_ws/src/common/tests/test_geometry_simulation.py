"""Pure geometry and simulator checks; no DDS or hardware required."""
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import yaml

NAV = Path(__file__).resolve().parents[2] / 'nav'
sys.path.insert(0, str(NAV / 'robot_geometry'))
sys.path.insert(0, str(NAV / 'robot_simulation'))
from robot_geometry.profile import load_profile, polygon, nav_parameters, inside_filter, transform_point
from robot_simulation.world import advance, collides, ray_distance, segments


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.path = NAV / 'robot_simulation/config/demo_geometry.yaml'
        self.profile = load_profile(self.path, True)

    def changed(self, **values):
        data = {**self.profile, **values}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'geometry.yaml'
            path.write_text(yaml.safe_dump(data))
            return load_profile(path, True)

    def test_unconfirmed_is_rejected(self):
        with self.assertRaises(ValueError):
            load_profile(self.path)

    def test_missing_loaded_is_rejected(self):
        with self.assertRaises(ValueError):
            self.changed(loaded_bounds_mm=None)

    def test_loaded_must_enclose_empty(self):
        with self.assertRaises(ValueError):
            self.changed(loaded_bounds_mm=[-100, -100, 100, 100])

    def test_reject_nonfinite(self):
        with self.assertRaises(ValueError):
            self.changed(laser_xyz_mm=[float('nan'), 0, 100])

    def test_mm_to_m(self):
        self.assertEqual(polygon(self.profile, True)[0], [.18, .18])

    def test_filter_changes_with_load(self):
        self.assertFalse(inside_filter(self.profile, False, .19, 0, .15))
        self.assertTrue(inside_filter(self.profile, True, .19, 0, .15))
        self.assertFalse(inside_filter(self.profile, True, .19, 0, .8))

    def test_nav_uses_both_footprints_without_mutating_template(self):
        template = yaml.safe_load((NAV / 'robot_navigation/config/nav2_params.yaml').read_text())
        original = copy.deepcopy(template)
        result = nav_parameters(template, self.profile, True)
        self.assertEqual(template, original)
        for name in ('local_costmap', 'global_costmap'):
            params = result[name][name]['ros__parameters']
            self.assertEqual(json.loads(params['footprint']), polygon(self.profile, True))
            self.assertGreater(params['inflation_layer']['inflation_radius'], .26)

    def test_transform(self):
        transform = SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.),
                                    translation=SimpleNamespace(x=.1, y=.2, z=.3))
        self.assertEqual(transform_point(transform, 1., 2.), (1.1, 2.2, .3))

    def test_holonomic_sideways_motion(self):
        self.assertEqual(advance([1, 1, 0], [0, .3, 0], 1), [1., 1.3, 0.])

    def test_collision_with_wall_and_box(self):
        box = [-.14, -.14, .14, .14]
        self.assertTrue(collides([.1, .5, 0], box, [3, 2], []))
        self.assertTrue(collides([1.5, 1., .5], box, [3, 2], [[1.4, .8, 1.6, 1.2]]))
        self.assertFalse(collides([.5, .5, 0], box, [3, 2], [[1.4, .8, 1.6, 1.2]]))

    def test_laser_wall_range(self):
        self.assertAlmostEqual(ray_distance(.5, .5, 0, segments([3, 2], [])), 2.5)


if __name__ == '__main__':
    unittest.main()
