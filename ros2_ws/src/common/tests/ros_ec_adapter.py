"""Verify adapter callbacks with real ROS messages and captured output only."""
import copy
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8, String, UInt8
from std_srvs.srv import SetBool, Trigger
import robot_bringup.ec_adapter as implementation


class Capture:
    def __init__(self):
        self.messages = []
        self.receivers = 1

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_subscription_count(self):
        return self.receivers


class EcAdapterChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        rclpy.init(domain_id=178)

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = implementation.EcAdapter()
        self.addCleanup(self.node.destroy_node)
        self.now = 10.
        fake_time = SimpleNamespace(monotonic=lambda: self.now)
        self.time_patch = patch.object(implementation, 'time', fake_time)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))
        self.clock_patch = patch.object(self.node, 'get_clock', return_value=clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        for name in ('odom_pub', 'imu_pub', 'velocity_pub', 'lift_pub', 'completion_pub', 'status_pub'):
            setattr(self.node, name, Capture())

    def odom(self, age=0.):
        message = Odometry()
        total = round((100.-age)*1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(total, 1_000_000_000)
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        message.pose.pose.orientation.w = 1.
        return message

    def imu(self, age=0.):
        message = Imu()
        message.header = self.odom(age).header
        message.header.frame_id = 'base_link'
        return message

    def ready(self, age=0.):
        self.node.odometry(self.odom(age))
        self.node.imu(self.imu(age))

    def move(self):
        self.node.settings['enable_motion'] = True
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = .6, .8, 3.
        self.node.velocity(message)

    def lift(self, value=1):
        self.node.settings['enable_lift'] = True
        self.node.lift_command(Int8(data=value))

    def confirm(self, value=True):
        return self.node.confirm_lift(SetBool.Request(data=value), SetBool.Response())

    def test_default_outputs_disabled(self):
        self.ready()
        message = Twist()
        message.linear.x = .2
        self.node.velocity(message)
        self.node.lift_command(Int8(data=1))
        self.assertFalse(self.node.velocity_pub.messages)
        self.assertFalse(self.node.lift_pub.messages)

    def test_transport_age_counts_towards_timeout(self):
        self.ready(.49)
        self.assertTrue(self.node.telemetry_ready())
        self.now += .02
        self.assertFalse(self.node.telemetry_ready())

    def test_invalid_timestamp_rejected(self):
        for age in (.6, -.2, 100.):
            with self.subTest(age=age):
                self.node.odometry(self.odom(age))
                self.node.imu(self.imu(age))
                self.assertFalse(self.node.telemetry_ready())
        self.assertFalse(self.node.odom_pub.messages)
        self.assertFalse(self.node.imu_pub.messages)

    def test_frames_not_silently_relabelled(self):
        message = self.odom()
        message.child_frame_id = 'base_footprint'
        self.node.odometry(message)
        message = self.imu()
        message.header.frame_id = 'imu_unknown'
        self.node.imu(message)
        self.assertFalse(self.node.telemetry_ready())
        self.assertFalse(self.node.odom_pub.messages)
        self.assertFalse(self.node.imu_pub.messages)

    def test_odometry_requires_unit_quaternion(self):
        for value in (0., 2., float('nan')):
            message = self.odom()
            message.pose.pose.orientation.w = value
            self.node.odometry(message)
        self.assertFalse(self.node.odom_pub.messages)
        self.node.odometry(self.odom())
        self.assertEqual(len(self.node.odom_pub.messages), 1)

    def test_nonfinite_odometry_rejected(self):
        message = self.odom()
        message.twist.twist.linear.x = float('inf')
        self.node.odometry(message)
        self.assertFalse(self.node.odom_pub.messages)

    def test_imu_rotation_covariance_and_unknown_orientation(self):
        self.node.rotation = implementation.rotation([0., 0., math.pi/2])
        message = self.imu()
        message.angular_velocity.x = 1.
        message.linear_acceleration.x = 2.
        message.angular_velocity_covariance = [1., 0., 0., 0., 4., 0., 0., 0., 9.]
        self.node.imu(message)
        output = self.node.imu_pub.messages[-1]
        self.assertAlmostEqual(output.angular_velocity.x, 0.)
        self.assertAlmostEqual(output.angular_velocity.y, 1.)
        self.assertAlmostEqual(output.linear_acceleration.y, 2.)
        self.assertAlmostEqual(output.angular_velocity_covariance[0], 4.)
        self.assertAlmostEqual(output.angular_velocity_covariance[4], 1.)
        self.assertEqual(output.linear_acceleration_covariance[0], .25)
        self.assertEqual(output.orientation_covariance[0], -1.)
        self.assertEqual(message.angular_velocity.x, 1.)

    def test_imu_unavailable_gyro_or_nonfinite_rejected(self):
        message = self.imu()
        message.angular_velocity_covariance[0] = -1.
        self.node.imu(message)
        message = self.imu()
        message.angular_velocity.z = float('nan')
        self.node.imu(message)
        self.assertFalse(self.node.imu_pub.messages)

    def test_imu_unknown_acceleration_remains_unavailable(self):
        message = self.imu()
        message.linear_acceleration_covariance[0] = -1.
        self.node.imu(message)
        output = self.node.imu_pub.messages[-1]
        self.assertEqual(output.linear_acceleration_covariance[0], -1.)
        self.assertEqual(output.angular_velocity_covariance[0], .0025)

    def test_motion_requires_fresh_inputs(self):
        self.move()
        self.assertFalse(self.node.velocity_pub.messages)
        self.ready()
        self.move()
        self.assertTrue(self.node.moving)
        self.now += .51
        self.move()
        self.assertFalse(self.node.moving)
        self.assertEqual(self.node.velocity_pub.messages[-1].linear.x, 0.)

    def test_velocity_vector_and_yaw_limits(self):
        self.ready()
        self.move()
        message = self.node.velocity_pub.messages[-1]
        self.assertAlmostEqual(message.linear.x, .3)
        self.assertAlmostEqual(message.linear.y, .4)
        self.assertEqual(message.angular.z, 2.)
        self.assertEqual(message.linear.z, 0.)

    def test_command_timeout_sends_one_zero_then_silence(self):
        self.ready()
        self.move()
        self.now += .26
        self.node.tick()
        self.node.tick()
        self.assertEqual(len(self.node.velocity_pub.messages), 2)
        self.assertFalse(self.node.moving)
        self.assertEqual(self.node.velocity_pub.messages[-1].linear.x, 0.)

    def test_nonfinite_command_stops_motion(self):
        self.ready()
        self.move()
        message = Twist()
        message.angular.z = float('nan')
        self.node.velocity(message)
        self.assertFalse(self.node.moving)
        self.assertEqual(self.node.velocity_pub.messages[-1].angular.z, 0.)

    def test_lift_target_never_means_arrival(self):
        self.lift()
        self.node.lift_target(UInt8(data=1))
        self.node.tick()
        self.assertEqual([m.data for m in self.node.completion_pub.messages], [-1])
        self.assertIsNotNone(self.node.pending)

    def test_lift_confirm_requires_post_command_target(self):
        self.node.lift_target(UInt8(data=1))
        self.lift()
        self.assertFalse(self.confirm().success)
        self.node.lift_target(UInt8(data=1))
        self.assertTrue(self.confirm().success)
        self.assertFalse(self.confirm().success)
        self.assertEqual([m.data for m in self.node.completion_pub.messages], [-1, 1])

    def test_lift_confirm_rejects_wrong_and_stale_targets(self):
        self.lift()
        self.node.lift_target(UInt8(data=0))
        self.assertFalse(self.confirm().success)
        self.node.lift_target(UInt8(data=1))
        self.now += .51
        self.assertFalse(self.confirm().success)
        self.assertFalse(self.confirm(False).success)

    def test_lift_timeout_cannot_publish_completion_or_automatic_stop(self):
        self.lift()
        self.now += 61.
        self.node.tick()
        self.assertIsNone(self.node.pending)
        self.assertFalse(self.confirm().success)
        self.assertEqual([m.data for m in self.node.completion_pub.messages], [-1])
        self.assertEqual([m.data for m in self.node.lift_pub.messages], ['lift up'])

    def test_lift_requires_one_receiver_and_one_pending_command(self):
        for count in (0, 2):
            self.node.lift_pub.receivers = count
            self.lift()
        self.assertFalse(self.node.lift_pub.messages)
        self.node.lift_pub.receivers = 1
        self.lift()
        self.lift(0)
        self.assertEqual([m.data for m in self.node.lift_pub.messages], ['lift up'])

    def test_explicit_stop_clears_pending_without_arrival(self):
        self.lift()
        response = self.node.stop_lift(Trigger.Request(), Trigger.Response())
        self.assertTrue(response.success)
        self.assertIsNone(self.node.pending)
        self.assertEqual([m.data for m in self.node.lift_pub.messages], ['lift up', 'lift stop'])
        self.assertEqual(self.node.completion_pub.messages[-1].data, -1)

    def test_terminal_mission_state_invalidates_confirmation(self):
        for state in ('ERROR', 'COMPLETE', 'WAIT_START'):
            self.lift()
            self.node.lift_target(UInt8(data=1))
            self.node.mission_status(String(data=json.dumps({'state': state})))
            self.assertIsNone(self.node.pending)
            self.assertFalse(self.confirm().success)


if __name__ == '__main__':
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(EcAdapterChecks)
    with (output/'ec_adapter.log').open('w') as log:
        result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    report = {'tests': result.testsRun, 'passed': result.wasSuccessful(),
              'failures': [(str(test), reason) for test, reason in result.failures],
              'errors': [(str(test), reason) for test, reason in result.errors],
              'scope': 'Real ROS message types; captured callbacks; no serial or hardware actuation'}
    (output/'ec_adapter.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 1)
