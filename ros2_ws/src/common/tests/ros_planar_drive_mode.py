"""Explicit, short ROS-side checks of the planar actuator's actual command gate."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from geometry_msgs.msg import Twist
from robot_simulation.planar_base import PlanarBase
from robot_simulation.world import advance


class PlanarDriveModeTests(unittest.TestCase):
    def state(self, mode):
        return SimpleNamespace(drive_mode=mode, velocity=[.1, 0., .2], command_at=0.,
            metrics={'maximum_requested_abs_vy': 0., 'rejected_lateral_commands': 0})

    def command(self, state, x, y, yaw):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = float(x), float(y), float(yaw)
        PlanarBase.command(state, message)

    def test_pure_lateral_is_rejected_and_does_not_advance_pose(self):
        state = self.state('differential')
        self.command(state, 0., .1, 0.)
        self.assertEqual(state.velocity, [0., 0., 0.])
        for actual, expected in zip(advance([1., 1., .7], state.velocity, .2), [1., 1., .7]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(state.metrics['rejected_lateral_commands'], 1)

    def test_mixed_command_is_rejected_in_full(self):
        state = self.state('differential')
        self.command(state, .1, -.01, .3)
        self.assertEqual(state.velocity, [0., 0., 0.])
        self.assertGreater(state.command_at, 0.)
        self.assertEqual(state.metrics['rejected_lateral_commands'], 1)

    def test_even_small_nonzero_lateral_is_rejected(self):
        state = self.state('differential')
        self.command(state, .1, 1e-12, .3)
        self.assertEqual(state.velocity, [0., 0., 0.])

    def test_forward_reverse_and_turn_remain_available(self):
        state = self.state('differential')
        for x, yaw in ((.1, .3), (-.05, -.2), (0., .4)):
            self.command(state, x, 0., yaw)
            self.assertEqual(state.velocity, [x, 0., yaw])
        self.assertEqual(state.metrics['rejected_lateral_commands'], 0)

    def test_default_omni_retains_lateral_execution_and_limits(self):
        state = self.state('omni')
        self.command(state, .6, -.4, 2.)
        self.assertEqual(state.velocity, [.4, -.3, 1.5])
        self.assertEqual(state.metrics['rejected_lateral_commands'], 0)

    def test_nonfinite_command_stops(self):
        for mode in ('omni', 'differential'):
            state = self.state(mode)
            self.command(state, .1, float('nan'), .2)
            self.assertEqual(state.velocity, [0., 0., 0.])


if __name__ == '__main__':
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output/'planar_drive_mode.log').open('w') as log:
        result = unittest.TextTestRunner(stream=log, verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(PlanarDriveModeTests))
    report = {'passed': result.wasSuccessful(), 'tests': result.testsRun,
              'scope': 'actual PlanarBase command method and planar kinematics; no physical hardware',
              'failures': [(str(test), reason) for test, reason in result.failures],
              'errors': [(str(test), reason) for test, reason in result.errors]}
    (output/'planar_drive_mode.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 1)
