"""Detect stale/reordered EC uint32 millisecond samples without guessing UTC.

Odom and IMU are independent streams. Duplicates never become fresh sensor
measurements. A backwards jump latches a fault until a new serial session,
because an in-place MCU reset cannot be distinguished from reordered data.
"""


class SampleClock:
    def __init__(self):
        self.last = {}
        self.faults = {}
        self.duplicates = {}
        self.backward = {}

    def reset(self):
        """Discard session-relative positions/faults, preserving diagnostic counts."""
        self.last.clear()
        self.faults.clear()

    def accept(self, stream, sample_ms):
        if isinstance(sample_ms, bool) or not isinstance(sample_ms, int) or not 0 <= sample_ms <= 0xFFFFFFFF:
            raise ValueError('sample_ms must be an unsigned 32-bit integer')
        if stream in self.faults:
            return False, 'clock_fault'
        previous = self.last.get(stream)
        if previous is None:
            self.last[stream] = sample_ms
            return True, 'first'
        delta = (sample_ms - previous) & 0xFFFFFFFF
        if delta == 0:
            self.duplicates[stream] = self.duplicates.get(stream, 0) + 1
            return False, 'duplicate'
        if delta >= 0x80000000:
            self.backward[stream] = self.backward.get(stream, 0) + 1
            self.faults[stream] = {'previous_ms': previous, 'received_ms': sample_ms,
                                   'recovery': 'Reconnect EC serial or restart the bridge'}
            return False, 'backward'
        self.last[stream] = sample_ms
        return True, 'advanced'

    def snapshot(self):
        return {'last_sample_ms': dict(self.last), 'duplicates': dict(self.duplicates),
                'backward_jumps': dict(self.backward), 'clock_faults': dict(self.faults)}
