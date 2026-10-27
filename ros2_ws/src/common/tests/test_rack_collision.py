"""Geometry and fault regressions for floor, carried and deposited racks."""
import copy
import math
from pathlib import Path
import sys
import unittest

NAV = Path(__file__).resolve().parents[2]/'nav'
sys.path.insert(0, str(NAV/'robot_geometry'))
sys.path.insert(0, str(NAV/'robot_simulation'))
from robot_simulation.course_scene import load_scene, world_config, course_profile
from robot_simulation.rack_world import RackWorld, separation, compose, relative
from robot_simulation.world import corners
from robot_geometry.profile import load_profile


class RackCollisionTests(unittest.TestCase):
    def setUp(self):
        self.scene = load_scene(NAV/'robot_simulation/config/course_layout.yaml')
        self.world = world_config(self.scene)
        self.body = [-.141251, -.143284, .141251, .143284]
        self.model = RackWorld(self.world, self.body)
        self.a = self.model.racks[0]

    def test_four_real_racks_and_sixteen_legs(self):
        self.assertEqual(len(self.model.floor_polygons(self.world['initial_pose'])), 16)
        self.assertGreater(self.model.clearance(self.world['initial_pose'])[0], 0)

    def test_body_can_enter_aligned_but_hits_leg_when_offset(self):
        self.assertGreater(self.model.clearance(self.a['pose'])[0], .04)
        x, y, yaw = self.a['pose']
        self.assertEqual(self.model.clearance([x, y+.08, yaw])[0], 0)

    def test_touching_counts_as_collision(self):
        a = corners([0, 0, 0], [0, 0, 1, 1])
        self.assertEqual(separation(a, corners([0, 0, 0], [1, 0, 2, 1])), 0)
        self.assertAlmostEqual(separation(a, corners([0, 0, 0], [2, 0, 3, 1])), 1.)

    def test_wall_accounts_for_entire_body(self):
        self.assertLess(self.model.clearance([.1, 1., 0.])[0], 0.)

    def test_lift_requires_correct_label_and_pose(self):
        with self.assertRaises(ValueError):
            self.model.lift(self.world['initial_pose'], 'A')
        self.assertIsNone(self.model.carried)
        self.assertEqual(self.a['state'], 'waiting')

    def test_lift_requires_orientation_alignment(self):
        with self.assertRaises(ValueError):
            self.model.lift([*self.a['pose'][:2], 0.], 'A')

    def test_lift_removes_only_attached_rack_from_floor_laser(self):
        self.model.lift(self.a['pose'], 'A')
        self.assertEqual(len(self.model.floor_polygons(self.a['pose'])), 12)
        with self.assertRaises(ValueError):
            self.model.lift(self.model.racks[1]['pose'], 'B')

    def test_attachment_preserves_actual_offset_and_yaw(self):
        robot = [self.a['pose'][0]+.01, self.a['pose'][1]-.005, math.pi-.01]
        original = list(self.a['pose'])
        self.model.lift(robot, 'A')
        for actual, expected in zip(self.model.rack_pose(self.a, robot), original):
            self.assertAlmostEqual(actual, expected)
        moved = [robot[0]+.3, robot[1]+.1, robot[2]+.2]
        self.assertEqual(self.model.rack_pose(self.a, moved), compose(moved, self.model.attachment))

    def test_drop_restores_legs_at_actual_location_without_snapping(self):
        self.model.lift(self.a['pose'], 'A')
        actual = [.225, .615, math.pi]
        self.model.lower(actual, 'A')
        self.assertEqual(self.a['pose'], actual)
        self.assertEqual(self.a['state'], 'delivered')
        self.assertEqual(len(self.model.floor_polygons(actual)), 16)
        self.assertEqual(self.model.clearance([.225, .695, math.pi])[0], 0.)

    def test_dropped_rack_is_obstacle_for_later_cargo(self):
        self.model.lift(self.a['pose'], 'A')
        self.model.lower(self.a['destination'], 'A')
        b = self.model.racks[1]
        self.model.lift(b['pose'], 'B')
        self.assertEqual(self.model.clearance(self.a['destination'])[0], 0.)

    def test_wrong_slot_cannot_complete_delivery(self):
        self.model.lift(self.a['pose'], 'A')
        with self.assertRaises(ValueError):
            self.model.lower([.5, 1., math.pi], 'A')
        self.assertEqual(self.model.carried, 'A')
        self.assertEqual(self.a['state'], 'carried')

    def test_partial_zone_overlap_cannot_count_as_delivery(self):
        self.model.lift(self.a['pose'], 'A')
        self.a['destination'] = [.03, .61, math.pi]
        with self.assertRaises(ValueError):
            self.model.lower(self.a['destination'], 'A')

    def test_no_teleporting_after_rejected_lower(self):
        self.model.lift(self.a['pose'], 'A')
        before = copy.deepcopy(self.a)
        with self.assertRaises(ValueError):
            self.model.lower([.1, .1, 0.], 'A')
        self.assertEqual(self.a, before)

    def test_fast_motion_cannot_tunnel_through_a_leg(self):
        a, b = [1.4, .72, math.pi], [.9, .72, math.pi]
        self.assertGreater(self.model.clearance(a)[0], 0)
        self.assertGreater(self.model.clearance(b)[0], 0)
        self.assertLessEqual(self.model.swept_clearance(a, b)[0], 0)

    def test_rotating_body_cannot_sweep_through_rack_legs(self):
        a = self.a['pose']
        b = [a[0], a[1], a[2]+math.pi/2]
        self.assertGreater(self.model.clearance(a)[0], 0)
        self.assertGreater(self.model.clearance(b)[0], 0)
        self.assertLessEqual(self.model.swept_clearance(a, b)[0], 0)

    def test_sweep_is_conservative_for_dense_intermediate_points(self):
        a, b = [2.7, 1., .1], [2.6, 1.1, .2]
        value, _ = self.model.swept_clearance(a, b)
        dense = min(self.model.clearance([a[j]+(b[j]-a[j])*i/1000 for j in range(3)])[0]
                    for i in range(1001))
        self.assertLessEqual(value, dense)
        self.assertLess(dense-value, .002)

    def test_zero_motion_does_not_invent_negative_clearance(self):
        a = self.a['pose']
        self.assertEqual(self.model.swept_clearance(a, a), self.model.clearance(a))

    def test_loaded_envelope_contains_every_allowed_attachment(self):
        p = load_profile(NAV/'robot_simulation/config/course_geometry.yaml', True)
        x0, y0, x1, y1 = [v/1000 for v in p['loaded_bounds_mm']]
        for angle in (-.05, 0., .05):
            for direction in range(36):
                t = direction*math.tau/36
                offset = [.025*math.cos(t), .025*math.sin(t), angle]
                for x, y in corners(offset, self.model.outer):
                    self.assertTrue(x0 <= x <= x1 and y0 <= y <= y1)

    def test_invalid_initial_overlaps_fail_at_startup(self):
        self.world['racks'][1]['pose'] = self.world['racks'][0]['pose']
        with self.assertRaises(ValueError):
            RackWorld(self.world, self.body)

    def test_all_four_slots_fit_and_leave_reverse_exit(self):
        # Pure geometry test only: actual transit is separately exercised by ROS.
        for rack in self.model.racks:
            self.model.lift(rack['pose'], rack['label'])
            target = rack['destination']
            self.assertGreater(self.model.clearance(target)[0], .005)
            self.model.lower(target, rack['label'])
            exit_pose = [target[0]+.42, target[1], target[2]]
            self.assertGreater(self.model.swept_clearance(target, exit_pose)[0], .005)
        self.assertEqual([r['state'] for r in self.model.racks], ['delivered']*4)

    def test_larger_rack_generates_larger_loaded_footprint(self):
        profile = load_profile(NAV/'robot_simulation/config/course_geometry.yaml', True)
        self.scene['rack_geometry_mm']['width'] = 440.
        larger = course_profile(self.scene, profile)
        self.assertGreater(larger['loaded_bounds_mm'][3], profile['loaded_bounds_mm'][3])

    def test_impossible_underpass_is_rejected_before_launch(self):
        profile = load_profile(NAV/'robot_simulation/config/course_geometry.yaml', True)
        self.scene['rack_geometry_mm']['width'] = 300.
        with self.assertRaises(ValueError):
            course_profile(self.scene, profile)


if __name__ == '__main__':
    unittest.main()
