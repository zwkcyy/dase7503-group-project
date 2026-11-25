"""Exercise the single-terminal launcher, keyboard and EC nodes on two Linux PTYs."""
import fcntl
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
from tempfile import TemporaryDirectory
import termios
import time
import tty

from ec_serial_bridge import protocol as ec
from ec_serial_bridge.framing import encode_packet, PacketParser


def exercise(launcher, work, quit_while_driving=False):
    serial, serial_slave = pty.openpty()
    terminal, screen = pty.openpty()
    tty.setraw(serial_slave)
    for fd in (serial, terminal):
        os.set_blocking(fd, False)
    fcntl.ioctl(screen, termios.TIOCSWINSZ, struct.pack('HHHH', 24, 120, 0, 0))
    settings = work / 'session.env'
    settings.write_text('export ROS_DOMAIN_ID=128 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST\n'
                        f'export EC_PORT={os.ttyname(serial_slave)}\n')
    env = dict(os.environ, GROUP5_SESSION_ENV=str(settings), TERM='xterm-256color', LANG='C.UTF-8')
    proc = subprocess.Popen([str(launcher)], stdin=screen, stdout=screen, stderr=screen,
                            start_new_session=True, env=env)
    parser = PacketParser()
    frames = []
    display = bytearray()
    armed = False
    lift = 0
    telemetry = True
    sent_at = 0.
    start = time.monotonic()

    def read(fd):
        try:
            return os.read(fd, 65536)
        except BlockingIOError:
            return b''

    def pump():
        nonlocal armed, lift, sent_at
        now = time.monotonic()
        display.extend(read(terminal))
        for packet in parser.feed(read(serial)):
            data = packet.data
            if packet.type == ec.TYPE_VELOCITY:
                data = tuple(round(x, 3) for x in struct.unpack('<3f', data))
            elif packet.type == ec.TYPE_LIFT:
                lift = data[0]
            elif packet.type == ec.TYPE_ARM:
                armed = bool(data[0])
            elif packet.type == ec.TYPE_STOP:
                armed = False
            frames.append((now, packet.type, data))
            if packet.type != ec.TYPE_VELOCITY:
                os.write(serial, encode_packet(ec.TYPE_ACK, bytes((packet.type, 0))))
        if telemetry and now - sent_at >= .05:
            sent_at = now
            stamp = int((now-start)*1000)+100
            flags = ec.FLAG_INITIALIZED | ec.FLAG_ODOMETRY_VALID | ec.FLAG_IMU_VALID | ec.FLAG_LIFT_STATE_KNOWN
            os.write(serial, encode_packet(ec.TYPE_STATUS, bytes((flags | (ec.FLAG_ARMED if armed else 0), 0)))
                     + encode_packet(ec.TYPE_LIFT_STATE, bytes((lift,)))
                     + encode_packet(ec.TYPE_ODOMETRY, struct.pack('<I6f', stamp, 0, 0, 0, 0, 0, 0))
                     + encode_packet(ec.TYPE_IMU, struct.pack('<I6f', stamp, 0, 0, 9.81, 0, 0, 0)))
        time.sleep(.007)

    def wait(test, seconds=6):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pump()
            if test():
                return
            if proc.poll() is not None:
                break
        raise AssertionError((display[-1500:].decode(errors='replace'), frames[-12:]))

    def run_for(seconds):
        end = time.monotonic() + seconds
        wait(lambda: time.monotonic() >= end, seconds + 1)

    def key(value):
        os.write(terminal, value)

    zero = (0., 0., 0.)
    try:
        wait(lambda: '按 A 手动使能'.encode() in display, 22)
        key(b'i,lj')
        run_for(.4)
        assert not any(kind in (ec.TYPE_ARM, ec.TYPE_LIFT, ec.TYPE_VELOCITY) for _, kind, _ in frames)
        key(b'9999')
        wait(lambda: lift == 1)
        run_for(.9)
        key(b'000')
        wait(lambda: lift == 0)
        assert [v[0] for _, kind, v in frames if kind == ec.TYPE_LIFT] == [1, 0]
        key(b'a')
        wait(lambda: armed and '已使能'.encode() in display)
        if quit_while_driving:
            key(b'i')
            wait(lambda: frames[-1][1:] == (ec.TYPE_VELOCITY, (.4, 0., 0.)))
            before = len(frames)
            key(b'q')
            wait(lambda: proc.poll() is not None, 12)
            ending = [(kind, data) for _, kind, data in frames[before:]]
            assert (ec.TYPE_VELOCITY, zero) in ending
            assert (ec.TYPE_STOP, b'') in ending and (ec.TYPE_ARM, b'\x00') in ending
            assert not armed and proc.returncode == 0
            print('PASS: quit while driving sent zero velocity, stop and disarm.')
            return
        for char, expected in ((b'i', (.4, 0., 0.)), (b',', (-.4, 0., 0.)),
                               (b'j', (0., 0., 2.)), (b'l', (0., 0., -2.))):
            before = len(frames)
            for _ in range(5):
                last = time.monotonic()
                key(char)
                run_for(.06)
            wait(lambda: frames[-1][1:] == (ec.TYPE_VELOCITY, zero))
            moving = [t for t, kind, v in frames[before:] if kind == ec.TYPE_VELOCITY and v == expected]
            assert len(moving) >= 3
            print(char, 'release to zero ms:', round((frames[-1][0]-last)*1000), flush=True)
            assert frames[-1][0]-last < .32
        key(b'i')
        wait(lambda: frames[-1][1:] == (ec.TYPE_VELOCITY, (.4, 0., 0.)))
        key(b'k')
        wait(lambda: frames[-1][1:] == (ec.TYPE_VELOCITY, zero))
        key(b'j')
        wait(lambda: frames[-1][1:] == (ec.TYPE_VELOCITY, (0., 0., 2.)))
        key(b'9')
        wait(lambda: lift == 1)
        run_for(.25)
        assert frames[-1][1:] == (ec.TYPE_VELOCITY, zero)
        telemetry = False
        wait(lambda: not armed)
        assert any(kind == ec.TYPE_STOP for _, kind, _ in frames)
        telemetry = True
        key(b'i')
        before = len(frames)
        run_for(.45)
        assert not any(kind == ec.TYPE_VELOCITY and v != zero for _, kind, v in frames[before:])
        key(b'q')
        wait(lambda: proc.poll() is not None, 12)
        assert proc.returncode == 0 and not armed
        print('PASS: one terminal, lift and drive, hold/release, braking, telemetry loss, exit and cleanup.')
    finally:
        if proc.poll() is None:
            key(b'q')
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
        for fd in (serial, serial_slave, terminal, screen):
            os.close(fd)


if __name__ == '__main__':
    with TemporaryDirectory() as directory:
        exercise(Path(sys.argv[1]).resolve(), Path(directory), '--quit-while-driving' in sys.argv[2:])
