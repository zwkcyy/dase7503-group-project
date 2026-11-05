"""Pure counter validation; no clocks, ROS, serial or hardware are accessed."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'common/ec_serial_bridge'))
from ec_serial_bridge.sample_clock import SampleClock


class SampleClockTests(unittest.TestCase):
    def test_first_zero_is_valid(self):
        clock = SampleClock()
        self.assertEqual(clock.accept('odometry', 0), (True, 'first'))
        self.assertEqual(clock.accept('odometry', 1), (True, 'advanced'))

    def test_duplicate_does_not_advance_sample(self):
        clock = SampleClock()
        clock.accept('odometry', 100)
        self.assertEqual(clock.accept('odometry', 100), (False, 'duplicate'))
        self.assertEqual(clock.snapshot()['last_sample_ms'], {'odometry': 100})
        self.assertEqual(clock.snapshot()['duplicates'], {'odometry': 1})
        self.assertEqual(clock.accept('odometry', 101), (True, 'advanced'))

    def test_backward_jump_latches_until_reconnect(self):
        clock = SampleClock()
        clock.accept('odometry', 100)
        self.assertEqual(clock.accept('odometry', 99), (False, 'backward'))
        self.assertEqual(clock.accept('odometry', 101), (False, 'clock_fault'))
        self.assertEqual(clock.snapshot()['last_sample_ms']['odometry'], 100)
        self.assertIn('Reconnect', clock.snapshot()['clock_faults']['odometry']['recovery'])
        clock.reset()
        self.assertEqual(clock.accept('odometry', 0), (True, 'first'))
        self.assertEqual(clock.snapshot()['clock_faults'], {})
        self.assertEqual(clock.snapshot()['backward_jumps'], {'odometry': 1})

    def test_uint32_wrap_advances(self):
        clock = SampleClock()
        clock.accept('odometry', 0xFFFFFFF0)
        self.assertEqual(clock.accept('odometry', 0x20), (True, 'advanced'))
        self.assertEqual(clock.snapshot()['clock_faults'], {})

    def test_ambiguous_half_range_is_rejected(self):
        clock = SampleClock()
        clock.accept('odometry', 0)
        self.assertEqual(clock.accept('odometry', 0x80000000), (False, 'backward'))

    def test_streams_are_independent(self):
        clock = SampleClock()
        self.assertEqual(clock.accept('odometry', 100), (True, 'first'))
        self.assertEqual(clock.accept('imu', 5), (True, 'first'))
        clock.accept('odometry', 99)
        self.assertEqual(clock.accept('imu', 6), (True, 'advanced'))
        self.assertEqual(set(clock.snapshot()['clock_faults']), {'odometry'})

    def test_invalid_types_and_range_are_rejected(self):
        for sample in (-1, 0x100000000, True, 1.0, '1', None):
            with self.subTest(sample=sample), self.assertRaises(ValueError):
                SampleClock().accept('imu', sample)


if __name__ == '__main__':
    unittest.main()
