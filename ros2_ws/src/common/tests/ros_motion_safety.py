"""Actual ROS node callback regressions; no actuator or navigation commands."""
import json
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.action import GoalResponse
from robot_geometry.motion_guard import MotionGuard, sample_fresh
from robot_mission.docking_controller import DockingController
from tf2_ros import TransformException


class MotionSafetyTests(unittest.TestCase):
    def setUp(self):
        self.node = MotionGuard()
        self.addCleanup(self.node.destroy_node)
        self.tf = TransformStamped()
        self.tf.header.stamp = self.node.get_clock().now().to_msg()
        self.tf.transform.rotation.w = 1.
        self.node.buffer = SimpleNamespace(lookup_transform=lambda *args: self.tf)
        self.scan = LaserScan()
        self.scan.header.stamp = self.tf.header.stamp
        self.scan.header.frame_id = 'laser'
        self.scan.range_min, self.scan.range_max = .03, 5.
        self.scan.ranges = [1.]*360
        self.odom = Odometry()
        self.odom.header.stamp = self.tf.header.stamp
        self.odom.child_frame_id = 'base_link'
        self.node.on_scan(self.scan)
        self.node.on_odom(self.odom)
        self.node.on_odom(self.odom, 'wheel')
        imu = Imu()
        imu.header = self.scan.header
        self.node.on_imu(imu)
        cmd = Twist()
        cmd.linear.x = .1
        self.node.on_command(cmd)
        self.node.raw_command_at = time.monotonic()
        self.node.pub = Mock()

    def test_healthy_commands_pass(self):
        self.assertEqual(self.node.blocked_reason(), '')
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, .1)

    def test_stale_command_sends_zero(self):
        self.node.command_at -= 1
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        self.assertEqual(self.node.reason, 'stale_command')

    def test_missing_scan_stops(self):
        self.node.samples.pop('laser')
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_laser')

    def test_smoother_cannot_keep_dead_controller_command_alive(self):
        self.node.raw_command_at -= 1.
        self.assertEqual(self.node.blocked_reason(), 'stale_controller_command')

    def test_frozen_timestamp_even_with_new_receipts_stops(self):
        self.scan.header.stamp.sec -= 2
        self.node.on_scan(self.scan)
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_laser')

    def test_future_timestamp_stops(self):
        self.scan.header.stamp.sec += 2
        self.node.on_scan(self.scan)
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_laser')

    def test_empty_scan_stops(self):
        self.scan.ranges = []
        self.node.on_scan(self.scan)
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_laser')

    def test_all_nan_scan_stops(self):
        self.scan.ranges = [float('nan')]*360
        self.node.on_scan(self.scan)
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_laser')

    def test_bad_odometry_stops(self):
        self.odom.twist.twist.linear.x = float('nan')
        self.node.on_odom(self.odom)
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_odometry')

    def test_old_tf_stops(self):
        self.tf.header.stamp.sec -= 2
        self.assertEqual(self.node.blocked_reason(), 'stale_transform')

    def test_missing_wheel_cannot_be_hidden_by_fresh_ekf(self):
        self.node.samples.pop('wheel')
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_wheel')

    def test_missing_imu_stops(self):
        self.node.samples.pop('imu')
        self.assertEqual(self.node.blocked_reason(), 'stale_or_invalid_imu')

    def test_missing_tf_stops(self):
        self.node.buffer.lookup_transform = Mock(side_effect=TransformException('missing'))
        self.assertEqual(self.node.blocked_reason(), 'missing_transform')

    def test_qr_and_lift_phases_hold_motion(self):
        for phase in ('WAIT_START', 'WAIT_RACK_QR', 'LIFT_RACK', 'VERIFY_DESTINATION', 'LOWER_RACK', 'COMPLETE'):
            self.node.on_phase(String(data=json.dumps({'state': phase})))
            self.assertEqual(self.node.blocked_reason(), 'mission_hold_or_stale_status')

    def test_fresh_docking_state_permits_motion(self):
        self.node.on_phase(String(data='{"state":"DOCK_RACK"}'))
        self.assertEqual(self.node.blocked_reason(), '')

    def test_dead_mission_process_stops_motion(self):
        self.node.on_phase(String(data='{"state":"NAV_TO_RACK"}'))
        self.node.phase_at -= 1
        self.assertEqual(self.node.blocked_reason(), 'mission_hold_or_stale_status')

    def test_error_is_latched(self):
        self.node.on_phase(String(data='{"state":"ERROR"}'))
        self.node.on_phase(String(data='{"state":"NAV_TO_RACK"}'))
        self.assertEqual(self.node.blocked_reason(), 'latched_stop')

    def test_operator_stop_is_latched(self):
        self.assertTrue(self.node.stop(Trigger.Request(), Trigger.Response()).success)
        self.assertEqual(self.node.blocked_reason(), 'latched_stop')

    def test_malformed_state_stops(self):
        self.node.on_phase(String(data='bad'))
        self.assertEqual(self.node.blocked_reason(), 'latched_stop')

    def test_wrong_state_type_stops_without_crashing(self):
        self.node.on_phase(String(data='{"state":[]}'))
        self.assertEqual(self.node.blocked_reason(), 'latched_stop')

    def test_nonfinite_command_stops(self):
        self.node.command.linear.x = float('nan')
        self.assertEqual(self.node.blocked_reason(), 'nonfinite_command')

    def test_freshness_rejects_zero_and_future_stamps(self):
        for stamp in (0., float('nan'), 12.):
            self.assertFalse(sample_fresh(10., stamp, 10., 10.))


class DockSafetyTests(unittest.TestCase):
    def setUp(self):
        self.node = DockingController()
        self.addCleanup(self.node.destroy_node)
        self.node.pub = Mock()
        self.tf = TransformStamped()
        self.tf.header.stamp = self.node.get_clock().now().to_msg()
        self.tf.transform.rotation.w = 1.
        self.node.buffer = SimpleNamespace(lookup_transform=lambda *args: self.tf)
        self.scan = LaserScan()
        self.scan.header.frame_id = 'laser'
        self.scan.header.stamp = self.tf.header.stamp
        self.scan.range_min, self.scan.range_max = .03, 5.
        self.scan.ranges = [5.]*360
        self.scan.angle_increment = .017453292
        self.node.on_scan(self.scan)
        self.goal = NavigateToPose.Goal()
        self.goal.pose.header.frame_id = 'map'
        self.goal.pose.pose.orientation.w = 1.
        self.goal.pose.pose.position.x = .4
        self.handle = Mock(request=self.goal, is_cancel_requested=False)
        self.future = Mock()
        self.node.active = (self.handle, self.future, time.monotonic())
        self.node.settled = None

    def test_valid_goal_accepted(self):
        self.assertEqual(self.node.accept(self.goal), GoalResponse.ACCEPT)
        self.assertEqual(self.node.accept(self.goal), GoalResponse.REJECT)

    def test_bad_goal_rejected(self):
        for value in (float('nan'), float('inf')):
            self.goal.pose.pose.position.x = value
            self.assertEqual(self.node.accept(self.goal), GoalResponse.REJECT)

    def test_old_scan_timestamp_holds_motion(self):
        self.scan.header.stamp.sec -= 2
        self.node.on_scan(self.scan)
        self.node.tick()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)
        self.assertIsNotNone(self.node.active)

    def test_obstacle_in_braking_envelope_stops(self):
        self.scan.ranges[0] = .17
        cmd = Twist()
        cmd.linear.x = .13
        self.assertTrue(self.node.obstructed(cmd))

    def test_clear_corridor_allows_docking(self):
        self.node.tick()
        self.assertGreater(self.node.pub.publish.call_args.args[0].linear.x, 0.)

    def test_cancellation_stops_and_acknowledges(self):
        self.handle.is_cancel_requested = True
        self.node.tick()
        self.handle.canceled.assert_called_once()
        self.assertIsNone(self.node.active)
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)

    def test_deadline_aborts_with_zero_velocity(self):
        self.node.active = (self.handle, self.future, time.monotonic()-21)
        self.node.tick()
        self.handle.abort.assert_called_once()
        self.assertEqual(self.node.pub.publish.call_args.args[0].linear.x, 0.)

    def test_large_heading_error_inside_rack_refuses_rotation(self):
        self.goal.pose.pose.position.x = .1
        self.goal.pose.pose.orientation.z = .1
        self.goal.pose.pose.orientation.w = (1-.01)**.5
        self.node.tick()
        self.handle.abort.assert_called_once()


if __name__ == '__main__':
    output = Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    profile = Path(__file__).resolve().parents[2]/'nav/robot_simulation/config/course_geometry.yaml'
    rclpy.init(args=['--ros-args', '-p', f'geometry_file:={profile}'], domain_id=179)
    try:
        suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(cls)
                                  for cls in (MotionSafetyTests, DockSafetyTests)])
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        (output/'motion_safety.json').write_text(json.dumps({'passed': result.wasSuccessful(),
            'tests': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors)}, indent=2))
        if not result.wasSuccessful():
            raise SystemExit(1)
    finally:
        rclpy.try_shutdown()
