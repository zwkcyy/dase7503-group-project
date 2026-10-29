"""Explicit CAD/optical geometry checks in the sourced ROS workspace."""
import json
import math
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml
from robot_gazebo.visibility import cad_triangles, measure, ray_hits, slice_triangles
from robot_gazebo.world import generate


class LidarGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.meshes = cad_triangles()

    def test_old_plane_blocks_central_rear(self):
        for loaded in (False, True):
            self.assertGreater(measure(self.meshes, z=.07, loaded=loaded)[0]['central_rear_blocked_rays'], 40)

    def test_new_plane_clears_central_rear_but_keeps_shadows(self):
        for loaded in (False, True):
            report = measure(self.meshes, z=.09, loaded=loaded)[0]
            self.assertEqual(report['central_rear_blocked_rays'], 0)
            self.assertGreater(report['equivalent_blocked_angle_deg'], 10)

    def test_cargo_adds_occlusion(self):
        empty = measure(self.meshes, loaded=False)[0]
        loaded = measure(self.meshes, loaded=True)[0]
        self.assertGreater(loaded['blocked_fraction'], empty['blocked_fraction'])
        self.assertIn('rack_leg', loaded['occluder_ray_counts'])

    def test_attachment_tolerance_corners(self):
        for x in (-.025, .025):
            for y in (-.025, .025):
                for yaw in (-.05, .05):
                    self.assertEqual(measure(self.meshes, loaded=True, rack_offset=(x,y,yaw))[0]['central_rear_blocked_rays'], 0)

    def test_empty_section_is_clear(self):
        distances, _ = ray_hits([], [0, 0], np.array([0., math.pi]))
        self.assertTrue(np.all(np.isinf(distances)))

    def test_segment_intersection_and_behind_origin(self):
        distances, _ = ray_hits([[[1., -1.], [1., 1.]]], [0., 0.], np.array([0., math.pi]))
        self.assertAlmostEqual(distances[0], 1.)
        self.assertTrue(math.isinf(distances[1]))

    def test_horizontal_triangle_outside_plane(self):
        self.assertEqual(len(slice_triangles(np.array([[[0.,0.,1.], [1.,0.,1.], [0.,1.,1.]]]), .5)), 0)

    def test_generated_footprint_contains_relocated_housing(self):
        directory = generate(output/'generated-world')
        metadata = json.loads((directory/'model_assumptions.json').read_text())
        profile = yaml.safe_load((directory/'geometry.yaml').read_text())
        high = metadata['body_bounds_m'][1]
        self.assertGreater(high[0], .163)
        for key in ('empty_bounds_mm', 'loaded_bounds_mm'):
            self.assertGreaterEqual(profile[key][2]/1000, high[0]-1e-10)
        mission = yaml.safe_load((directory/'mission.yaml').read_text())['delivery_mission']['ros__parameters']
        self.assertGreater(mission['placement_exit_distance_m'], .365)


if __name__ == '__main__':
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(LidarGeometryTests))
    (output/'geometry_checks.json').write_text(json.dumps({'passed': result.wasSuccessful(), 'tests': result.testsRun}))
    raise SystemExit(not result.wasSuccessful())
