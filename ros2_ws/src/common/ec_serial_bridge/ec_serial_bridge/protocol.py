"""Payload encode/decode for the EC binary UART protocol (standard library only).

Field order, types and validation follow ``docs/communication.md`` and
``src/communication/serial_link.cpp`` at EC firmware 6985c8a. Downlink frames are
built here; uplink frames are decoded and validated. Nothing in this module reads
a clock, a file or a serial port, so the offline tests can exercise it directly.
"""

import math
import struct
from dataclasses import dataclass

from .framing import encode_packet

# Downlink (Pi -> EC)
TYPE_VELOCITY = 0x01
TYPE_LIFT = 0x02
TYPE_ARM = 0x03
TYPE_ESTOP = 0x04
TYPE_STOP = 0x05
TYPE_STATUS_REQUEST = 0x06
TYPE_LIFT_STOP = 0x07

# Uplink (EC -> Pi)
TYPE_ODOMETRY = 0x81
TYPE_IMU = 0x82
TYPE_LIFT_STATE = 0x83
TYPE_STATUS = 0x84
TYPE_ACK = 0xF0

NAME = {
    TYPE_VELOCITY: 'velocity', TYPE_LIFT: 'lift', TYPE_ARM: 'arm', TYPE_ESTOP: 'estop',
    TYPE_STOP: 'stop', TYPE_STATUS_REQUEST: 'status_request', TYPE_LIFT_STOP: 'lift_stop',
    TYPE_ODOMETRY: 'odometry', TYPE_IMU: 'imu', TYPE_LIFT_STATE: 'lift_state',
    TYPE_STATUS: 'status', TYPE_ACK: 'ack',
}

EXPECTED_LENGTH = {
    TYPE_VELOCITY: 12, TYPE_LIFT: 1, TYPE_ARM: 1, TYPE_ESTOP: 1, TYPE_STOP: 0,
    TYPE_STATUS_REQUEST: 0, TYPE_LIFT_STOP: 0,
    TYPE_ODOMETRY: 28, TYPE_IMU: 28, TYPE_LIFT_STATE: 1, TYPE_STATUS: 2, TYPE_ACK: 2,
}

FLAG_INITIALIZED = 1 << 0
FLAG_ARMED = 1 << 1
FLAG_ESTOP = 1 << 2
FLAG_ODOMETRY_VALID = 1 << 3
FLAG_IMU_VALID = 1 << 4
FLAG_LIFT_STATE_KNOWN = 1 << 5

FAULT_NONE = 0
FAULT_NAMES = {0: 'none', 1: 'configuration', 2: 'motor', 3: 'encoder',
               4: 'timing', 5: 'feedback', 6: 'imu', 7: 'lift'}


@dataclass(frozen=True)
class OdometrySample:
    sample_ms: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class ImuSample:
    sample_ms: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


@dataclass(frozen=True)
class LiftState:
    """Completion flag: 0 = lowering timed out at its endpoint, 1 = raising did."""
    state: int


@dataclass(frozen=True)
class StatusFrame:
    flags: int
    fault: int

    def has(self, flag):
        return bool(self.flags & flag)


@dataclass(frozen=True)
class Ack:
    command_type: int
    result: int

    @property
    def accepted(self):
        return self.result == 0

    def __str__(self):
        return f'{NAME.get(self.command_type, hex(self.command_type))}:' + (
            'accepted' if self.accepted else 'rejected')


def _finite(*values):
    return all(math.isfinite(value) for value in values)


def encode_velocity(vx, vy, wz):
    """Type 01: vx, vy in m/s and wz in rad/s, float32 little endian."""
    return encode_packet(TYPE_VELOCITY, struct.pack('<3f', float(vx), float(vy), float(wz)))


def encode_lift(direction):
    """Type 02: 0 = lower, 1 = raise. The EC rejects anything else."""
    if direction not in (0, 1):
        raise ValueError(f'lift direction must be 0 or 1, got {direction!r}')
    return encode_packet(TYPE_LIFT, bytes((direction,)))


def encode_arm(enabled):
    """Type 03: 1 arms the chassis, 0 disarms it."""
    return encode_packet(TYPE_ARM, bytes((1 if enabled else 0,)))


def encode_estop(engaged):
    """Type 04: 1 engages the chassis emergency stop, 0 releases it."""
    return encode_packet(TYPE_ESTOP, bytes((1 if engaged else 0,)))


def encode_stop():
    """Type 05: stop chassis; whether it also cuts lift PWM depends on firmware."""
    return encode_packet(TYPE_STOP)


def encode_lift_stop():
    """Type 07: stop lift PWM only."""
    return encode_packet(TYPE_LIFT_STOP)


def encode_status_request():
    """Type 06: ask for an immediate telemetry batch."""
    return encode_packet(TYPE_STATUS_REQUEST)


def decode(frame):
    """Decode one :class:`~.framing.Frame`.

    Returns ``(kind, payload)`` where ``kind`` is one of ``odometry``, ``imu``,
    ``lift_state``, ``status`` or ``ack``. Returns ``None`` for an unknown type,
    a wrong length or non-finite float content: the bridge counts those and
    publishes nothing rather than guessing a value.
    """
    packet_type = frame.type
    data = frame.data
    expected = EXPECTED_LENGTH.get(packet_type)
    if expected is None or len(data) != expected:
        return None
    if packet_type == TYPE_ODOMETRY:
        sample_ms = struct.unpack_from('<I', data, 0)[0]
        values = struct.unpack_from('<6f', data, 4)
        if not _finite(*values):
            return None
        return 'odometry', OdometrySample(sample_ms, *values)
    if packet_type == TYPE_IMU:
        sample_ms = struct.unpack_from('<I', data, 0)[0]
        values = struct.unpack_from('<6f', data, 4)
        if not _finite(*values):
            return None
        return 'imu', ImuSample(sample_ms, *values)
    if packet_type == TYPE_LIFT_STATE:
        if data[0] > 1:
            return None
        return 'lift_state', LiftState(data[0])
    if packet_type == TYPE_STATUS:
        return 'status', StatusFrame(data[0], data[1])
    return 'ack', Ack(data[0], data[1])
