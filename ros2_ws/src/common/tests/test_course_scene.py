"""Keep the course view, generated mission and simulation world in agreement."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

NAV = Path(__file__).resolve().parents[2]/'nav'
sys.path.insert(0, str(NAV/'robot_geometry'))
sys.path.insert(0, str(NAV/'robot_simulation'))
from robot_simulation.course_scene import load_scene, mission_config, world_config, map_config, screen_point
from robot_simulation.world import collides
from robot_geometry.profile import load_profile


class CourseSceneTests(unittest.TestCase):
    def setUp(self):
        self.path = NAV/'robot_simulation/config/course_layout.yaml'
        self.scene = load_scene(self.path)

    def changed(self, **values):
        scene = {**self.scene, **values}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'scene.yaml'
            path.write_text(yaml.safe_dump(scene))
            return load_scene(path)

    def test_dimensions_and_zones_match_course_drawing(self):
        self.assertEqual(self.scene['size_mm'], [3020., 2000.])
        self.assertEqual(self.scene['start_zone_mm'], [2420., 600., 3020., 1400.])
        self.assertEqual(self.scene['destination_zone_mm'], [0., 300., 800., 1700.])

    def test_schematic_is_explicitly_not_a_survey(self):
        self.assertEqual(self.scene['position_basis'], 'approximate_centres_from_page_3_not_surveyed')
        with self.assertRaises(ValueError):
            self.changed(layout_kind='physical_robot')

    def test_world_and_mission_derive_from_same_scene(self):
        world = world_config(self.scene)
        mission = mission_config(self.scene)['delivery_mission']['ros__parameters']
        self.assertEqual(world['initial_pose'][:2], [2.72, 1.])
        self.assertEqual([mission['destination_x_m'], mission['destination_y_m']], [.4, 1.])
        for i, rack in enumerate(self.scene['racks']):
            self.assertEqual(mission['rack_x_m'][i], rack['x_mm']/1000.)
            self.assertEqual(mission['rack_y_m'][i], rack['y_mm']/1000.)
            self.assertEqual(mission['rack_qr_codes'][i], rack['qr'])

    def test_course_departure_policy_uses_geometry(self):
        profile = load_profile(NAV/'robot_simulation/config/course_geometry.yaml', True)
        mission = mission_config(self.scene, profile)['delivery_mission']['ros__parameters']
        self.assertFalse(mission['retreat_after_pickup'])
        self.assertAlmostEqual(mission['docking_distance_m'], .42)
        self.assertAlmostEqual(mission['placement_exit_distance_m'], .365)

    def test_invalid_departure_policy_is_rejected(self):
        with self.assertRaises(ValueError):
            self.changed(retreat_after_pickup='false')

    def test_map_dimensions_derive_from_scene_instead_of_another_default(self):
        scene = copy.deepcopy(self.scene)
        scene['size_mm'] = [3040., 2020.]
        config = map_config(scene)
        self.assertEqual(config['field']['length_mm'], scene['size_mm'][0])
        self.assertEqual(config['field']['width_mm'], scene['size_mm'][1])
        self.assertEqual(config['zones']['start'], scene['start_zone_mm'])

    def test_presentation_places_start_below_destination(self):
        world = world_config(self.scene)
        start = screen_point(*world['initial_pose'][:2], world['size'], (0, 0, 200, 302))
        end = screen_point(.4, 1., world['size'], (0, 0, 200, 302))
        self.assertEqual(start[0], end[0])
        self.assertGreater(start[1], end[1])

    def test_labels_follow_page_three_schematic(self):
        positions = {r['label']: screen_point(r['x_mm']/1000., r['y_mm']/1000., [3.02, 2.], (0, 0, 200, 302))
                     for r in self.scene['racks']}
        self.assertLess(positions['A'][0], positions['C'][0])
        self.assertLess(positions['D'][0], positions['B'][0])
        self.assertLess(positions['A'][1], positions['D'][1])
        self.assertLess(positions['C'][1], positions['B'][1])

    def test_no_regression_obstacle_in_course_scene(self):
        self.assertEqual(world_config(self.scene)['obstacles'], [])
        old = yaml.safe_load((NAV/'robot_simulation/config/demo_world.yaml').read_text())
        self.assertTrue(old['obstacles'])

    def test_all_task_poses_clear_the_walls_with_demo_loaded_bounds(self):
        world = world_config(self.scene)
        poses = [world['initial_pose'], [v/1000. for v in self.scene['destination_pose_mm_rad'][:2]]+[0.]]
        poses.extend([r['x_mm']/1000., r['y_mm']/1000., r['yaw_rad']] for r in self.scene['racks'])
        for pose in poses:
            self.assertFalse(collides(pose, [-.18, -.18, .18, .18], world['size'], world['obstacles']))

    def test_generation_does_not_modify_source(self):
        original = copy.deepcopy(self.scene)
        world_config(self.scene)
        mission_config(self.scene)
        self.assertEqual(self.scene, original)

    def test_invalid_zones_and_nonfinite_poses_are_rejected(self):
        for values in ({'start_zone_mm': [0, 0, 4000, 3000]}, {'size_mm': [0, 2000]},
                       {'initial_pose_mm_rad': [float('nan'), 1000, 0]},
                       {'destination_pose_mm_rad': [2000, 1000, 0]}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.changed(**values)

    def test_duplicate_racks_or_non_demo_codes_are_rejected(self):
        for key, value in [('label', 'B'), ('qr', 'RACKA_REAL')]:
            racks = copy.deepcopy(self.scene['racks'])
            racks[0][key] = value
            with self.assertRaises(ValueError):
                self.changed(racks=racks)

    def test_accepted_cad_supports_empty_only_without_full_mission_approval(self):
        path = NAV/'robot_description/config/mechanical_cad_20261025.yaml'
        profile = load_profile(path, empty_only=True)
        self.assertFalse(profile['confirmed'])
        self.assertEqual(profile['loaded_bounds_mm'], profile['empty_bounds_mm'])
        self.assertIsNone(yaml.safe_load(path.read_text())['loaded_bounds_mm'])
        with self.assertRaises(ValueError):
            load_profile(path)

    def test_unaccepted_draft_cannot_use_empty_only_as_general_bypass(self):
        with self.assertRaises(ValueError):
            load_profile(NAV/'robot_description/config/mechanical.yaml', empty_only=True)


if __name__ == '__main__':
    unittest.main()
