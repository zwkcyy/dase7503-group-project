import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'nav/robot_geometry'))
from robot_geometry.directional_safety import braking_obstacle, rear_visibility, reverse_policy


class DirectionalSafetyTests(unittest.TestCase):
    def setUp(self):
        self.empty = [-.141251, -.143284, .141251, .143284]
        self.loaded = [-.141251, -.228249, .141251, .228249]
        self.scan = SimpleNamespace(ranges=[1.]*721, angle_min=-math.pi,
                                    angle_increment=2*math.pi/720, range_min=.025, range_max=5.)

    def test_complete_rear_sector(self):
        self.assertTrue(rear_visibility(self.scan))

    def test_one_unknown_rear_ray_blocks(self):
        self.scan.ranges[2] = math.nan
        self.assertFalse(rear_visibility(self.scan))

    def test_rear_inf_is_valid_no_return(self):
        self.scan.ranges = [math.inf]*721
        self.assertTrue(rear_visibility(self.scan))

    def test_missing_rear_sector(self):
        self.scan.angle_min = -.5
        self.scan.angle_increment = 1/720
        self.assertFalse(rear_visibility(self.scan))

    def test_invalid_angles(self):
        for value in (0., math.nan, -.1):
            self.scan.angle_increment = value
            self.assertFalse(rear_visibility(self.scan))

    def test_rear_invalid_range(self):
        for value in (-math.inf, 0., 6.):
            self.scan.ranges[0] = value
            self.assertFalse(rear_visibility(self.scan))

    def test_transit_cannot_reverse_fast(self):
        self.assertEqual(reverse_policy('NAV_TO_RACK', -.1, 0), 'transit_reverse_speed_limit')

    def test_exit_bounded_speed(self):
        self.assertEqual(reverse_policy('EXIT_DESTINATION', -.08, .36), '')
        self.assertEqual(reverse_policy('EXIT_DESTINATION', -.1, 0), 'reverse_speed_limit')

    def test_exit_bounded_distance(self):
        self.assertEqual(reverse_policy('EXIT_RACK', -.08, .51), 'reverse_distance_limit')

    def test_no_unbounded_small_reverse(self):
        self.assertEqual(reverse_policy('NAV_TO_RACK', -.02, .09), 'transit_reverse_distance_limit')

    def test_phase_required(self):
        self.assertEqual(reverse_policy(None, -.02, 0), 'reverse_requires_mission_phase')

    def test_front_obstacle_before_contact(self):
        self.assertTrue(braking_obstacle([(.23, 0)], self.empty, (.3, 0, 0)))

    def test_rear_obstacle_before_contact(self):
        self.assertTrue(braking_obstacle([(-.17, 0)], self.empty, (-.08, 0, 0)))

    def test_far_obstacle_not_stopping(self):
        self.assertFalse(braking_obstacle([(1., 0)], self.empty, (.3, 0, 0)))

    def test_loaded_width_matters(self):
        self.assertFalse(braking_obstacle([(.22, .20)], self.empty, (.2, 0, 0)))
        self.assertTrue(braking_obstacle([(.22, .20)], self.loaded, (.2, 0, 0)))

    def test_rotation_swept_corner(self):
        self.assertFalse(braking_obstacle([(.20, .15)], self.loaded, (0, 0, 0)))
        self.assertTrue(braking_obstacle([(.20, .15)], self.loaded, (0, 0, -1.)))

    def test_zero_command_does_not_erase_momentum(self):
        self.assertTrue(braking_obstacle([(.23, 0)], self.empty, (0, 0, 0), (.3, 0, 0)))

    def test_forward_acceleration_with_residual_yaw_is_checked(self):
        point = [(.34, -.29)]
        self.assertFalse(braking_obstacle(point, self.loaded, (.4, 0, 0)))
        self.assertFalse(braking_obstacle(point, self.loaded, (0, 0, -1.)))
        self.assertTrue(braking_obstacle(point, self.loaded, (.4, 0, 0), (0, 0, -1.)))

    def test_transit_config_rotates_before_following_and_after_arrival(self):
        path = Path(__file__).resolve().parents[2]/'nav/robot_navigation/config/nav2_params.yaml'
        params = yaml.safe_load(path.read_text())['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(params['plugin'], 'nav2_rotation_shim_controller::RotationShimController')
        self.assertEqual(params['primary_controller'], 'nav2_mppi_controller::MPPIController')
        self.assertEqual(params['motion_model'], 'Omni')
        self.assertEqual(params['vx_min'], -.02)
        self.assertTrue(params['rotate_to_goal_heading'])
        self.assertTrue(params['rotate_to_heading_once'])
        self.assertFalse(params['GoalAngleCritic']['enabled'])


if __name__ == '__main__':
    unittest.main()
