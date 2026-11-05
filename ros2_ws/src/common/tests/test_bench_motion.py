"""Pure bench feedback protection; no ROS runtime or physical devices."""
import json
import io
from contextlib import redirect_stderr
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'robot_bringup'))
from robot_bringup.bench_motion import BenchMonitor, main


def odom(stamp=100., vx=0., vy=0., wz=0.):
    ns = round(stamp*1e9)
    return NS(header=NS(stamp=NS(sec=ns//10**9, nanosec=ns % 10**9), frame_id='odom'),
              child_frame_id='base_link',
              pose=NS(pose=NS(position=NS(x=0., y=0., z=0.),
                             orientation=NS(x=0., y=0., z=0., w=1.)), covariance=[0.]*36),
              twist=NS(twist=NS(linear=NS(x=vx, y=vy, z=0.),
                               angular=NS(x=0., y=0., z=wz)), covariance=[0.]*36))


class BenchMotionTests(unittest.TestCase):
    def test_command_bounds_and_required_ack_cannot_be_bypassed(self):
        cases = [('--forward', '.051'), ('--turn', '.201'), ('--forward', 'nan'),
                 ('--forward', '.03', '--seconds', '2.01'),
                 ('--forward', '.03', '--seconds', '.09'),
                 ('--forward', '.03', '--turn', '.1'),
                 ('--forward', '.03', '--disable-protection')]
        for args in cases:
            with self.subTest(args=args), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exit:
                main(['--wheels-raised', *args])
            self.assertEqual(exit.exception.code, 2)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exit:
            main(['--forward', '.03'])
        self.assertEqual(exit.exception.code, 2)

    def ready(self):
        m = BenchMonitor()
        m.status('adapter', json.dumps({'motion_enabled': True, 'telemetry_ready': True}), 10.)
        m.status('bridge', json.dumps({'link_ok': True, 'ec_armed': True,
                                      'ec_estop': False, 'ec_fault': 0}), 10.)
        for key in m.SOURCES:
            m.odometry(key, odom(99.9), 9.9, 99.9)
            m.odometry(key, odom(), 10., 100.)
        return m

    def test_both_sources_required(self):
        m = self.ready()
        self.assertIsNone(m.problem(10., 100.))
        del m.samples['raw']
        self.assertIn('no odometry', m.problem(10., 100.))

    def test_reported_actual_overspeed_latches(self):
        for speed in (.24, .314):
            with self.subTest(speed=speed):
                m = self.ready(); m.running = True
                m.odometry('raw', odom(100.1, vx=speed), 10.1, 100.1)
                self.assertIn('fixed bench ceiling', m.problem(10.1, 100.1))
                m.odometry('raw', odom(100.2), 10.2, 100.2)
                self.assertIn('fixed bench ceiling', m.problem(10.2, 100.2))

    def test_adapted_source_alone_can_stop(self):
        m = self.ready()
        m.odometry('wheel', odom(100.1, vx=.101), 10.1, 100.1)
        self.assertIn('wheel:', m.problem(10.1, 100.1))

    def test_hypotenuse_and_reverse_are_checked(self):
        for vx, vy in ((.08, .08), (-.11, 0.)):
            m = self.ready()
            m.odometry('raw', odom(100.1, vx=vx, vy=vy), 10.1, 100.1)
            self.assertIsNotNone(m.aborted)

    def test_yaw_checked_in_both_directions(self):
        for wz in (.41, -.41):
            m = self.ready()
            m.odometry('raw', odom(100.1, wz=wz), 10.1, 100.1)
            self.assertIsNotNone(m.aborted)

    def test_fixed_ceilings_and_safe_command_feedback(self):
        for speed, yaw in ((.03, 0.), (.10, .4)):
            m = self.ready(); m.running = True
            m.odometry('raw', odom(100.1, vx=speed, wz=yaw), 10.1, 100.1)
            self.assertIsNone(m.problem(10.1, 100.1))

    def test_preflight_requires_two_consecutive_stationary_samples(self):
        m = self.ready()
        m.odometry('raw', odom(100.1, vx=.03), 10.1, 100.1)
        self.assertIn('stationary', m.problem(10.1, 100.1))
        m.odometry('raw', odom(100.2), 10.2, 100.2)
        self.assertIn('stationary', m.problem(10.2, 100.2))
        m.odometry('raw', odom(100.3), 10.3, 100.3)
        self.assertIsNone(m.problem(10.3, 100.3))

    def test_old_or_future_or_zero_stamp(self):
        for stamp in (0., 98., 100.2):
            m = self.ready(); m.running = True
            m.odometry('raw', odom(stamp), 10.01, 100.01)
            self.assertIn('timestamp', m.aborted)

    def test_repeated_or_backward_stamp(self):
        for stamp in (100., 99.9):
            m = self.ready(); m.running = True
            m.odometry('raw', odom(stamp), 10.1, 100.1)
            self.assertIn('did not advance', m.aborted)

    def test_invalid_nanosecond_field(self):
        m = self.ready(); m.running = True
        message = odom(100.1)
        message.header.stamp.nanosec = 1_000_000_000
        m.odometry('raw', message, 10.1, 100.1)
        self.assertIn('invalid timestamp', m.aborted)

    def test_arrival_age_is_not_reset(self):
        m = self.ready()
        m.odometry('raw', odom(100.1), 10.55, 100.55)
        self.assertIn('stale', m.problem(10.61, 100.61))

    def test_missing_packets_latch_while_running(self):
        m = self.ready(); m.running = True
        self.assertIn('stale', m.problem(10.51, 100.51))
        self.assertIsNotNone(m.aborted)

    def test_clock_jumps_rejected(self):
        for ros_now in (99., 102.):
            m = self.ready(); m.running = True
            self.assertIn('clock jump', m.problem(10.1, ros_now))

    def test_nonfinite_pose_twist_and_covariance(self):
        for field in ('pose', 'twist', 'covariance'):
            m = self.ready(); m.running = True
            message = odom(100.1)
            if field == 'pose': message.pose.pose.position.x = float('nan')
            elif field == 'twist': message.twist.twist.linear.x = float('inf')
            else: message.twist.covariance[10] = float('nan')
            m.odometry('raw', message, 10.1, 100.1)
            self.assertIn('nonfinite', m.aborted)

    def test_bad_frame_and_orientation(self):
        for field in ('frame', 'quaternion'):
            m = self.ready(); m.running = True
            message = odom(100.1)
            if field == 'frame': message.child_frame_id = 'other'
            else: message.pose.pose.orientation.w = 0.
            m.odometry('raw', message, 10.1, 100.1)
            self.assertIsNotNone(m.aborted)

    def test_malformed_status_latches(self):
        m = self.ready(); m.running = True
        m.status('bridge', '{', 10.1)
        self.assertIn('malformed', m.aborted)

    def test_arming_and_competing_controller(self):
        m = self.ready(); m.running = True
        self.assertIn('competing', m.problem(10., 100., publishers=2))
        m = self.ready()
        m.statuses['bridge'][1]['ec_armed'] = False
        self.assertIn('arming', m.problem(10., 100.))

    def test_status_age_and_missing_consumer(self):
        m = self.ready()
        m.statuses['adapter'] = (8., m.statuses['adapter'][1])
        self.assertIn('stale status', m.problem(10., 100.))
        self.assertIn('missing consumer', self.ready().problem(10., 100., subscribers=0))

    def test_cleanup_needs_new_disarmed_and_stationary_samples(self):
        m = self.ready()
        self.assertEqual(m.stopped_feedback(10.05, 10.1, 100.1), (False, False))
        m.status('bridge', json.dumps({'link_ok': True, 'ec_armed': False}), 10.1)
        for key in m.SOURCES:
            m.odometry(key, odom(100.1), 10.1, 100.1)
        self.assertEqual(m.stopped_feedback(10.05, 10.1, 100.1), (True, True))
        m.odometry('raw', odom(100.2, vx=.03), 10.2, 100.2)
        self.assertEqual(m.stopped_feedback(10.05, 10.2, 100.2), (True, False))


if __name__ == '__main__':
    unittest.main()
