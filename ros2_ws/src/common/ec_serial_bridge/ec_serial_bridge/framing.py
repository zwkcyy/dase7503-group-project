"""Byte-level framing for the EC binary UART protocol.

This module uses only the standard library so the offline test suite can run on a
host that has no pyserial and no serial port. Frame recovery mirrors
``src/communication/serial_protocol.cpp`` in the EC firmware at 6985c8a:

* header AA 55, type, length, data, CRC-16/Modbus (low byte first), tail 0D 0A
* CRC covers type, length and data only
* every rejected byte is dropped one at a time, so a later valid frame in the
  same buffer is still recovered
* after 100 ms of silence the firmware abandons the partial frame and rescan;
  :meth:`PacketParser.expire` reproduces that behaviour
"""

HEADER = b'\xaa\x55'
TAIL = b'\x0d\x0a'
MAX_PAYLOAD = 32
FRAME_OVERHEAD = 8
MAX_FRAME = MAX_PAYLOAD + FRAME_OVERHEAD


def crc16(data):
    """CRC-16/Modbus: init 0xFFFF, polynomial 0xA001 reflected."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def encode_packet(packet_type, data=b''):
    """Encode one frame. ``data`` is the payload only, without header or CRC."""
    payload = bytes(data)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload is {len(payload)} bytes; the EC limit is {MAX_PAYLOAD}')
    body = bytes((packet_type & 0xFF, len(payload))) + payload
    crc = crc16(body)
    return HEADER + body + bytes((crc & 0xFF, crc >> 8)) + TAIL


class Frame:
    """One validated frame: ``type`` plus the raw payload bytes."""

    __slots__ = ('type', 'data')

    def __init__(self, packet_type, data):
        self.type = packet_type
        self.data = data

    def __repr__(self):
        return f'Frame(type=0x{self.type:02X}, length={len(self.data)})'


class PacketParser:
    """Incremental frame decoder with firmware-equivalent resynchronisation."""

    def __init__(self):
        self._buffer = bytearray()
        self.crc_errors = 0
        self.tail_errors = 0
        self.discarded_bytes = 0
        self.oversize_frames = 0

    def reset(self):
        self.discarded_bytes += len(self._buffer)
        self._buffer.clear()

    def pending(self):
        return len(self._buffer)

    def feed(self, chunk):
        """Append received bytes and return every complete frame now available."""
        self._buffer.extend(chunk)
        return self._take(expired=False)

    def expire(self):
        """Abandon an incomplete frame, then recover any complete frame behind it.

        Call after ``resync_timeout_s`` of silence. Any complete frame still in
        the buffer is returned; the remaining fragment is dropped.
        """
        frames = self._take(expired=True)
        self.discarded_bytes += len(self._buffer)
        self._buffer.clear()
        return frames

    def _drop(self, count):
        self.discarded_bytes += count
        del self._buffer[:count]

    def _take(self, expired):
        frames = []
        while self._buffer:
            if self._buffer[0] != HEADER[0]:
                self._drop(1)
                continue
            if len(self._buffer) < 2:
                if expired:
                    self._drop(1)
                    continue
                break
            if self._buffer[1] != HEADER[1]:
                self._drop(1)
                continue
            if len(self._buffer) < 4:
                if expired:
                    self._drop(1)
                    continue
                break
            length = self._buffer[3]
            if length > MAX_PAYLOAD:
                self.oversize_frames += 1
                self._drop(1)
                continue
            total = length + FRAME_OVERHEAD
            if len(self._buffer) < total:
                if expired:
                    self._drop(1)
                    continue
                break
            crc = self._buffer[4 + length] | (self._buffer[5 + length] << 8)
            if self._buffer[6 + length] != TAIL[0] or self._buffer[7 + length] != TAIL[1]:
                self.tail_errors += 1
                self._drop(1)
                continue
            if crc16(bytes(self._buffer[2:4 + length])) != crc:
                self.crc_errors += 1
                self._drop(1)
                continue
            frames.append(Frame(self._buffer[2], bytes(self._buffer[4:4 + length])))
            del self._buffer[:total]
        return frames
