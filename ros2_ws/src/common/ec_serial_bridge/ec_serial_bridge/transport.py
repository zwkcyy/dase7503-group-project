"""Serial transport for the EC link.

This is the only module in the package that imports pyserial, which keeps the
protocol and framing modules testable on a host without ``python3-serial``.
Nothing here interprets a byte; it only opens, reads and writes.
"""

import os

import serial


class SerialTransport:
    """Reconnecting UART link. Read returns raw bytes, never frames."""

    def __init__(self, port, baudrate, read_timeout_s, write_timeout_s, reconnect_interval_s):
        self.port = port
        self.baudrate = int(baudrate)
        self.read_timeout_s = float(read_timeout_s)
        self.write_timeout_s = float(write_timeout_s)
        self.reconnect_interval_s = float(reconnect_interval_s)
        self._handle = None
        self._next_attempt = 0.0
        self.last_error = ''
        self.open_count = 0
        # POSIX pyserial uses an advisory flock. Other cooperating instances are
        # rejected; this cannot prevent access by applications which ignore locks.
        # The option is unsupported by the Windows backend, so omit it there.
        self.exclusive = os.name == 'posix'

    @property
    def is_open(self):
        return self._handle is not None

    def _open(self, now):
        if now < self._next_attempt:
            return
        self._next_attempt = now + self.reconnect_interval_s
        try:
            options = {'exclusive': True} if self.exclusive else {}
            self._handle = serial.Serial(port=self.port, baudrate=self.baudrate,
                                         bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE,
                                         timeout=self.read_timeout_s,
                                         write_timeout=self.write_timeout_s, **options)
            self.open_count += 1
            self.last_error = ''
        except (serial.SerialException, OSError, ValueError) as error:
            self._handle = None
            self.last_error = f'{type(error).__name__}: {error}'

    def _close(self, reason=''):
        if self._handle is not None:
            try:
                self._handle.close()
            except (serial.SerialException, OSError):
                pass
        self._handle = None
        if reason:
            self.last_error = reason

    def poll(self, now):
        """Return whatever bytes are available; b'' when closed or idle."""
        if self._handle is None:
            self._open(now)
            if self._handle is None:
                return b''
        try:
            waiting = self._handle.in_waiting
            return self._handle.read(waiting or 1)
        except (serial.SerialException, OSError) as error:
            self._close(f'read failed: {type(error).__name__}: {error}')
            return b''

    def write(self, data):
        """Write one whole frame. A short write or error closes the link."""
        if self._handle is None:
            return False
        try:
            written = self._handle.write(data)
        except (serial.SerialException, OSError) as error:
            self._close(f'write failed: {type(error).__name__}: {error}')
            return False
        if written != len(data):
            self._close(f'short write: {written}/{len(data)} bytes')
            return False
        return True

    def close(self):
        self._close()
