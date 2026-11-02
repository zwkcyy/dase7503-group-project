"""Offline checks for the EC binary UART bridge.

These tests import only the pure byte layers. Neither pyserial nor a serial port
nor ROS is needed, which is what lets ``scripts/check_nav.sh offline`` cover the
frame and payload logic on a host without ``python3-serial``.
"""

import struct
import sys
import unittest
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC/'common/ec_serial_bridge'))
from ec_serial_bridge import lift_retry, protocol
from ec_serial_bridge.framing import (PacketParser, crc16, encode_packet, MAX_PAYLOAD)

# The one example frame printed in the EC communication document: lift up.
DOCUMENTED_LIFT_UP = bytes.fromhex('AA55020101 1050 0D0A'.replace(' ', ''))


def frame(packet_type, payload=b''):
    return encode_packet(packet_type, payload)


def odometry_frame(x, y, yaw, vx, vy, wz, sample_ms=1234):
    return frame(protocol.TYPE_ODOMETRY, struct.pack('<I6f', sample_ms, x, y, yaw, vx, vy, wz))


def imu_frame(ax, ay, az, gx, gy, gz, sample_ms=1234):
    return frame(protocol.TYPE_IMU, struct.pack('<I6f', sample_ms, ax, ay, az, gx, gy, gz))


class CrcTests(unittest.TestCase):
    def test_crc_matches_documented_frame(self):
        self.assertEqual(crc16(DOCUMENTED_LIFT_UP[2:5]), 0x5010)

    def test_crc_rejects_bit_flip(self):
        damaged = bytearray(DOCUMENTED_LIFT_UP)
        damaged[4] ^= 0x01
        parser = PacketParser()
        self.assertEqual(parser.feed(bytes(damaged)), [])
        self.assertEqual(parser.crc_errors, 1)


class EncodeTests(unittest.TestCase):
    def test_documented_frame_round_trip(self):
        self.assertEqual(frame(protocol.TYPE_LIFT, bytes((1,))), DOCUMENTED_LIFT_UP)

    def test_frame_shape(self):
        encoded = frame(protocol.TYPE_LIFT_STOP)
        self.assertEqual(encoded[:2], b'\xaa\x55')
        self.assertEqual(encoded[2], protocol.TYPE_LIFT_STOP)
        self.assertEqual(encoded[3], 0)
        self.assertEqual(encoded[-2:], b'\x0d\x0a')
        self.assertEqual(len(encoded), 8)

    def test_velocity_payload(self):
        encoded = protocol.encode_velocity(0.5, -0.25, 1.5)
        self.assertEqual(len(encoded), 12 + 8)
        self.assertEqual(encoded[2], protocol.TYPE_VELOCITY)
        self.assertEqual(struct.unpack_from('<3f', encoded, 4), (0.5, -0.25, 1.5))

    def test_lift_rejects_other_directions(self):
        for value in (2, -1, 255):
            with self.assertRaises(ValueError):
                protocol.encode_lift(value)

    def test_boolean_frames(self):
        self.assertEqual(protocol.encode_arm(True)[4], 1)
        self.assertEqual(protocol.encode_arm(False)[4], 0)
        self.assertEqual(protocol.encode_estop(True)[4], 1)

    def test_payload_limit(self):
        with self.assertRaises(ValueError):
            frame(0x50, bytes(MAX_PAYLOAD + 1))
        self.assertEqual(len(frame(0x50, bytes(MAX_PAYLOAD))), MAX_PAYLOAD + 8)


class ParserTests(unittest.TestCase):
    def test_one_frame(self):
        parser = PacketParser()
        frames = parser.feed(DOCUMENTED_LIFT_UP)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, protocol.TYPE_LIFT)
        self.assertEqual(frames[0].data, b'\x01')
        self.assertEqual(parser.pending(), 0)

    def test_byte_at_a_time(self):
        parser = PacketParser()
        received = []
        for byte in DOCUMENTED_LIFT_UP:
            received.extend(parser.feed(bytes((byte,))))
        self.assertEqual(len(received), 1)
        self.assertEqual(parser.discarded_bytes, 0)

    def test_sticky_packets(self):
        parser = PacketParser()
        frames = parser.feed(odometry_frame(1., 2., .5, .1, 0., .2) + imu_frame(0., 0., 9.8, 0., 0., .1))
        self.assertEqual([item.type for item in frames], [protocol.TYPE_ODOMETRY, protocol.TYPE_IMU])

    def test_leading_noise(self):
        parser = PacketParser()
        frames = parser.feed(b'\x00\xaa\x55' + DOCUMENTED_LIFT_UP)
        self.assertEqual(len(frames), 1)
        self.assertGreater(parser.discarded_bytes, 0)

    def test_bad_crc_then_recovery(self):
        damaged = bytearray(DOCUMENTED_LIFT_UP)
        damaged[5] ^= 0xFF
        parser = PacketParser()
        frames = parser.feed(bytes(damaged) + DOCUMENTED_LIFT_UP)
        self.assertEqual(len(frames), 1)
        self.assertEqual(parser.crc_errors, 1)

    def test_bad_tail_then_recovery(self):
        damaged = bytearray(DOCUMENTED_LIFT_UP)
        damaged[-1] = 0x00
        parser = PacketParser()
        frames = parser.feed(bytes(damaged) + DOCUMENTED_LIFT_UP)
        self.assertEqual(len(frames), 1)
        self.assertEqual(parser.tail_errors, 1)

    def test_oversize_length_is_rejected(self):
        parser = PacketParser()
        frames = parser.feed(b'\xaa\x55\x81' + bytes((MAX_PAYLOAD + 1,)) + DOCUMENTED_LIFT_UP)
        self.assertEqual(len(frames), 1)
        self.assertEqual(parser.oversize_frames, 1)

    def test_partial_frame_waits(self):
        parser = PacketParser()
        self.assertEqual(parser.feed(DOCUMENTED_LIFT_UP[:-3]), [])
        self.assertEqual(parser.pending(), len(DOCUMENTED_LIFT_UP) - 3)
        remainder = parser.feed(DOCUMENTED_LIFT_UP[-3:])
        self.assertEqual(len(remainder), 1)

    def test_expire_drops_fragment_and_recovers_next_frame(self):
        parser = PacketParser()
        self.assertEqual(parser.feed(b'\xaa\x55\x81\x1c\x00\x00'), [])
        frames = parser.expire()
        self.assertEqual(len(frames), 0)
        self.assertEqual(parser.pending(), 0)
        self.assertGreater(parser.discarded_bytes, 0)

    def test_expire_recovers_complete_frame_behind_fragment(self):
        parser = PacketParser()
        self.assertEqual(parser.feed(b'\xaa\x55\x81\x1c\x00\x00' + frame(protocol.TYPE_STATUS, b'\x01\x00')), [])
        frames = parser.expire()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, protocol.TYPE_STATUS)
        self.assertEqual(parser.pending(), 0)

    def test_reset_discards_buffer(self):
        parser = PacketParser()
        parser.feed(DOCUMENTED_LIFT_UP[:4])
        parser.reset()
        self.assertEqual(parser.pending(), 0)


class DecodeTests(unittest.TestCase):
    def parse_one(self, encoded):
        frames = PacketParser().feed(encoded)
        self.assertEqual(len(frames), 1)
        return protocol.decode(frames[0])

    def test_odometry(self):
        kind, sample = self.parse_one(odometry_frame(1.5, -2.25, .75, .4, -.1, .3))
        self.assertEqual(kind, 'odometry')
        self.assertAlmostEqual(sample.x, 1.5)
        self.assertAlmostEqual(sample.y, -2.25)
        self.assertAlmostEqual(sample.yaw, .75)
        self.assertAlmostEqual(sample.vx, .4)
        self.assertAlmostEqual(sample.vy, -.1)
        self.assertAlmostEqual(sample.wz, .3)
        self.assertEqual(sample.sample_ms, 1234)

    def test_imu(self):
        kind, sample = self.parse_one(imu_frame(.1, -.2, 9.81, .01, .02, -.03))
        self.assertEqual(kind, 'imu')
        self.assertAlmostEqual(sample.az, 9.81, places=4)
        self.assertAlmostEqual(sample.gz, -.03, places=4)

    def test_non_finite_sample_is_rejected(self):
        self.assertIsNone(self.parse_one(odometry_frame(float('nan'), 0., 0., 0., 0., 0.)))
        self.assertIsNone(self.parse_one(imu_frame(float('inf'), 0., 0., 0., 0., 0.)))

    def test_lift_state(self):
        kind, state = self.parse_one(frame(protocol.TYPE_LIFT_STATE, b'\x01'))
        self.assertEqual((kind, state.state), ('lift_state', 1))
        self.assertIsNone(self.parse_one(frame(protocol.TYPE_LIFT_STATE, b'\x02')))

    def test_status_flags(self):
        flags = (protocol.FLAG_INITIALIZED | protocol.FLAG_ARMED | protocol.FLAG_LIFT_STATE_KNOWN)
        kind, status = self.parse_one(frame(protocol.TYPE_STATUS, bytes((flags, protocol.FAULT_NONE))))
        self.assertEqual(kind, 'status')
        self.assertTrue(status.has(protocol.FLAG_ARMED))
        self.assertFalse(status.has(protocol.FLAG_ESTOP))
        self.assertEqual(status.fault, 0)

    def test_ack(self):
        kind, ack = self.parse_one(frame(protocol.TYPE_ACK, bytes((protocol.TYPE_LIFT, 0))))
        self.assertEqual(kind, 'ack')
        self.assertTrue(ack.accepted)
        _, rejected = self.parse_one(frame(protocol.TYPE_ACK, bytes((protocol.TYPE_LIFT, 3))))
        self.assertFalse(rejected.accepted)

    def test_wrong_length_is_rejected(self):
        self.assertIsNone(self.parse_one(frame(protocol.TYPE_ODOMETRY, bytes(27))))
        self.assertIsNone(self.parse_one(frame(protocol.TYPE_ACK, b'\x01')))
        self.assertIsNone(self.parse_one(frame(protocol.TYPE_STOP, b'\x00')))

    def test_unknown_type_is_rejected(self):
        self.assertIsNone(self.parse_one(frame(0x99, b'')))


class LiftRetryTests(unittest.TestCase):
    """The 02 command is acked by F0; a missing ack triggers a resend."""

    def test_no_retry_before_the_window_closes(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.2, max_retries=5)
        retrier.arm(1, now=10.)
        self.assertEqual(retrier.plan(10.19), (None, None))
        self.assertTrue(retrier.pending)

    def test_resend_after_the_window_closes(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.2, max_retries=5)
        retrier.arm(1, now=10.)
        self.assertEqual(retrier.plan(10.2), (lift_retry.ACTION_RESEND, 1))
        retrier.retried(10.2)
        self.assertEqual(retrier.attempts, 2)
        self.assertEqual(retrier.plan(10.3), (None, None))

    def test_ack_clears_the_command(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.2, max_retries=5)
        retrier.arm(0, now=10.)
        self.assertEqual(retrier.settle(protocol.TYPE_LIFT, True, now=10.05), 'ok')
        self.assertFalse(retrier.pending)
        self.assertEqual(retrier.plan(99.), (None, None))

    def test_ack_for_another_command_is_ignored(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.2, max_retries=5)
        retrier.arm(0, now=10.)
        self.assertEqual(retrier.settle(protocol.TYPE_VELOCITY, True, now=10.05), '')
        self.assertTrue(retrier.pending)

    def test_refusal_ends_the_retries(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.2, max_retries=5)
        retrier.arm(1, now=10.)
        self.assertEqual(retrier.settle(protocol.TYPE_LIFT, False, now=10.05), 'rejected')
        self.assertFalse(retrier.pending)

    def test_retry_budget_is_finite(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.1, max_retries=2)
        retrier.arm(1, now=0.)
        now = 0.
        actions = []
        for _ in range(5):
            now += .1
            action, _direction = retrier.plan(now)
            actions.append(action)
            if action == lift_retry.ACTION_RESEND:
                retrier.retried(now)
            elif action == lift_retry.ACTION_GIVE_UP:
                break
        self.assertEqual(actions, [lift_retry.ACTION_RESEND, lift_retry.ACTION_RESEND,
                                   lift_retry.ACTION_GIVE_UP])
        self.assertEqual(retrier.attempts, 3)

    def test_zero_retry_restores_single_shot(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.1, max_retries=0)
        retrier.arm(1, now=0.)
        self.assertEqual(retrier.plan(.1), (lift_retry.ACTION_GIVE_UP, 1))

    def test_unlimited_retry_count_still_expires(self):
        retrier = lift_retry.LiftCommandRetrier(ack_timeout_s=.1, max_retries=-1)
        retrier.arm(1, now=0.)
        now = 0.
        # -1 removes the count limit, not the absolute command lifetime.
        for _ in range(19):
            now += .1
            self.assertEqual(retrier.plan(now), (lift_retry.ACTION_RESEND, 1))
            retrier.retried(now)
        self.assertEqual(retrier.attempts, 20)
        self.assertEqual(retrier.plan(2.0), (lift_retry.ACTION_GIVE_UP, 1))
        self.assertEqual(retrier.plan(5.0), (lift_retry.ACTION_GIVE_UP, 1))

    def test_rejects_bad_configuration(self):
        for kwargs in ({'ack_timeout_s': 0.}, {'ack_timeout_s': float('nan')},
                       {'max_retries': -2}):
            with self.assertRaises(ValueError):
                lift_retry.LiftCommandRetrier(**kwargs)


if __name__ == '__main__':
    unittest.main()
