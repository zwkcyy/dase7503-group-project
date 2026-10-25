"""Read-only startup validation and confirmation artifacts without ROS."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml
import cv2
import numpy as np

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC/'common/robot_bringup'))
sys.path.insert(0, str(SRC/'vi/robot_vision'))
from robot_bringup.health import freshness
from robot_bringup.configuration import mission_parameters
from robot_vision.evidence import save_confirmation


class ReadinessTests(unittest.TestCase):
    def test_fresh_message(self):
        self.assertEqual(freshness(10., 100., 10.1, 100.1, 2.), '')

    def test_missing_stream(self):
        self.assertIn('no messages', freshness(None, 0., 10., 100., 2.))

    def test_stopped_stream(self):
        self.assertIn('stopped', freshness(10., 100., 13., 103., 2.))

    def test_stale_stamp_despite_new_receipt(self):
        self.assertIn('stale', freshness(10., 90., 10.1, 100., 2.))

    def test_future_and_zero_stamps(self):
        self.assertIn('ahead', freshness(10., 110., 10.1, 100., 2.))
        self.assertIn('zero', freshness(10., 0., 10.1, 100., 2.))

    def test_nonfinite_stamp(self):
        self.assertIn('invalid', freshness(10., float('nan'), 10.1, 100., 2.))

    def changed(self, **values):
        path = SRC/'nav/robot_simulation/config/demo_mission.yaml'
        data = yaml.safe_load(path.read_text())
        data['delivery_mission']['ros__parameters'].update(values)
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp)/'mission.yaml'
            target.write_text(yaml.safe_dump(data))
            return mission_parameters(target)

    def test_valid_mission(self):
        self.assertEqual(len(self.changed()['rack_labels']), 4)

    def test_unconfirmed_mission(self):
        with self.assertRaisesRegex(ValueError, 'unconfirmed'):
            self.changed(coordinates_confirmed=False)

    def test_mismatched_or_partial_codes(self):
        for codes in (['RACKA'], ['RACKB_X', 'RACKB_X', 'RACKC_X', 'RACKD_X']):
            with self.subTest(codes=codes), self.assertRaises(ValueError):
                self.changed(rack_qr_codes=codes)

    def test_invalid_poses_and_timeouts(self):
        for values in ({'rack_x_m': [0.]}, {'destination_x_m': float('nan')},
                       {'qr_timeout_s': 0}, {'rack_labels': ['A', 'A', 'C', 'D']}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.changed(**values)

    def test_wrong_array_types_and_start_deadline(self):
        for values in ({'rack_labels': 'ABCD'}, {'rack_labels': [1, 2]},
                       {'rack_x_m': None}, {'rack_qr_codes': None}, {'start_timeout_s': -1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.changed(**values)


class EvidenceTests(unittest.TestCase):
    def test_saved_image_is_the_confirming_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            frame = np.full((20, 30, 3), 123, np.uint8)
            record = save_confirmation(tmp, 'RACKA_DEMO', 'test', frame, 12345, 'synthetic_test')
            image = cv2.imread(str(Path(tmp)/record['image']))
            self.assertTrue(np.array_equal(frame, image))
            data = json.loads(next(Path(tmp).glob('*.json')).read_text())
            self.assertEqual(data['image_stamp_ns'], 12345)
            self.assertEqual(data['source'], 'synthetic_test')
            self.assertFalse(data['physical_acceptance'])

    def test_payload_does_not_become_a_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = save_confirmation(tmp, '../outside', 'test', np.zeros((10, 10, 3), np.uint8), 1, 'test')
            self.assertNotIn('outside', record['image'])
            self.assertEqual(len(list(Path(tmp).glob('*.png'))), 1)

    def test_identical_codes_keep_separate_records(self):
        with tempfile.TemporaryDirectory() as tmp:
            for _ in range(2):
                save_confirmation(tmp, 'END', 'test', np.zeros((10, 10, 3), np.uint8), 1, 'test')
            self.assertEqual(len(list(Path(tmp).glob('*.json'))), 2)


if __name__ == '__main__':
    unittest.main()
