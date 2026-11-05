"""Mocked transport contract; never imports a real serial backend or opens ports."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class SerialError(Exception):
    pass


class SerialTransportTests(unittest.TestCase):
    def setUp(self):
        self.handle = Mock()
        self.handle.in_waiting = 0
        self.handle.read.return_value = b''
        self.backend = SimpleNamespace(Serial=Mock(return_value=self.handle),
                                       SerialException=SerialError, EIGHTBITS=8,
                                       PARITY_NONE='N', STOPBITS_ONE=1)
        path = (Path(__file__).resolve().parents[2]
                / 'common/ec_serial_bridge/ec_serial_bridge/transport.py')
        spec = importlib.util.spec_from_file_location('ec_transport_mock_check', path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'serial': self.backend}):
            spec.loader.exec_module(self.module)

    def transport(self, platform='posix'):
        with patch.object(self.module.os, 'name', platform):
            return self.module.SerialTransport('/never-a-real-device', 115200, .01, .05, .1)

    def test_posix_requests_exclusive_open(self):
        transport = self.transport()
        transport.poll(1.)
        self.assertTrue(transport.exclusive)
        self.assertTrue(self.backend.Serial.call_args.kwargs['exclusive'])
        self.assertEqual(transport.open_count, 1)

    def test_windows_omits_unsupported_option(self):
        transport = self.transport('nt')
        transport.poll(1.)
        self.assertFalse(transport.exclusive)
        self.assertNotIn('exclusive', self.backend.Serial.call_args.kwargs)

    def test_locked_port_is_reported_and_retried_after_interval(self):
        self.backend.Serial.side_effect = [SerialError('locked by another process'), self.handle]
        transport = self.transport()
        self.assertEqual(transport.poll(1.), b'')
        self.assertFalse(transport.is_open)
        self.assertIn('locked', transport.last_error)
        transport.poll(1.05)
        self.assertEqual(self.backend.Serial.call_count, 1)
        transport.poll(1.11)
        self.assertTrue(transport.is_open)
        self.assertEqual(transport.open_count, 1)

    def test_short_write_closes_link_and_does_not_replay_after_reopen(self):
        transport = self.transport()
        transport.poll(1.)
        self.handle.write.return_value = 1
        self.assertFalse(transport.write(b'abc'))
        self.assertFalse(transport.is_open)
        self.assertIn('short write', transport.last_error)
        transport.poll(2.)
        self.assertEqual(transport.open_count, 2)
        self.handle.write.assert_called_once_with(b'abc')

    def test_read_error_closes_link(self):
        transport = self.transport()
        self.handle.read.side_effect = SerialError('disconnected')
        self.assertEqual(transport.poll(1.), b'')
        self.assertFalse(transport.is_open)
        self.assertIn('disconnected', transport.last_error)
        self.handle.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
