"""Scanner lifecycle checks and real OpenCV QR decoding on synthetic images."""

import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

from ros_fakes import Image, Node, Trigger, load_module


vision = load_module("vision_under_test",
                     "vi/robot_vision/robot_vision/qr_detector_node.py", True)


def qr_image(text):
    qr = cv2.QRCodeEncoder_create().encode(text)
    qr = np.pad(qr, 4, constant_values=255)
    qr = cv2.resize(qr, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST)
    return cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        timer = patch.object(vision.time, "monotonic", lambda: self.now)
        timer.start()
        self.addCleanup(timer.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        overrides = patch.object(Node, "overrides", {
            "model_dir": str(self.directory / "models"),
            "log_file": str(self.directory / "log.csv"),
        })
        overrides.start()
        self.addCleanup(overrides.stop)
        self.node = vision.QRDetectorNode()
        self.node.debug_pub.subscribers = 0

    def start(self):
        return self.node.start_scan_callback(None, Trigger.Response())

    def frame(self, text="START", stamp_ns=None):
        self.now += 0.11
        if stamp_ns is None:
            self.node.now_ns += 110_000_000
            stamp_ns = self.node.now_ns
        self.node.image_callback(Image(qr_image(text), stamp_ns))

    def test_real_opencv_decodes_all_coursework_markers(self):
        for text in ("START", "END", "RACKA_aB12", "RACKB_12ab",
                     "RACKC_XyZ9", "RACKD_4X6M"):
            with self.subTest(text=text):
                result = self.node.detect_qr(qr_image(text))
                self.assertIsNotNone(result)
                self.assertEqual(result[0], text)

    def test_real_opencv_rejects_unrelated_or_malformed_markers(self):
        for text in ("https://example.org", "RACKA", "RACKE_1234", "start"):
            with self.subTest(text=text):
                self.assertIsNone(self.node.detect_qr(qr_image(text)))

    def test_two_distinct_frames_publish_once_and_return_to_idle(self):
        self.start()
        self.frame("RACKA_aB12")
        self.assertFalse(self.node.data_pub.messages)
        self.frame("RACKA_aB12")
        self.frame("RACKA_aB12")
        self.assertEqual([m.data for m in self.node.data_pub.messages], ["RACKA_aB12"])
        self.assertFalse(self.node.scanning_enabled)
        self.assertIn("RACKA_aB12", (self.directory / "log.csv").read_text())

    def test_old_and_duplicate_frames_do_not_count(self):
        self.start()
        stamp = self.node.now_ns
        self.frame(stamp_ns=stamp - 1)
        self.assertFalse(self.node.hit_history)
        self.frame(stamp_ns=stamp)
        self.frame(stamp_ns=stamp)
        self.assertEqual(len(self.node.hit_history), 1)
        self.assertFalse(self.node.data_pub.messages)
        self.frame(stamp_ns=stamp + 1)
        self.assertEqual(len(self.node.data_pub.messages), 1)

    def test_zero_stamped_sources_remain_supported(self):
        self.start()
        self.frame(stamp_ns=0)
        self.frame(stamp_ns=0)
        self.assertEqual(len(self.node.data_pub.messages), 1)

    def test_stop_is_idempotent_and_clears_session(self):
        self.assertTrue(self.node.stop_scan_callback(None, Trigger.Response()).success)
        self.start()
        self.frame()
        self.assertTrue(self.node.stop_scan_callback(None, Trigger.Response()).success)
        self.assertFalse(self.node.hit_history)
        self.frame()
        self.assertFalse(self.node.data_pub.messages)
        self.assertFalse(self.node.detected_pub.messages[-1].data)

    def test_repeated_start_does_not_clear_confirmation_history(self):
        self.start()
        self.frame()
        self.assertTrue(self.start().success)
        self.assertEqual(len(self.node.hit_history), 1)
        self.frame()
        self.assertEqual(len(self.node.data_pub.messages), 1)

    def test_new_session_resets_history(self):
        self.start()
        self.frame()
        self.node.stop_scan_callback(None, Trigger.Response())
        self.start()
        self.frame()
        self.assertFalse(self.node.data_pub.messages)

    def test_confirmation_window_expires(self):
        self.start()
        self.frame()
        self.now += self.node.confirm_window + 0.1
        self.frame()
        self.assertFalse(self.node.data_pub.messages)
        self.frame()
        self.assertEqual(len(self.node.data_pub.messages), 1)

    def test_different_codes_do_not_confirm_each_other(self):
        self.start()
        self.frame("START")
        self.frame("END")
        self.assertFalse(self.node.data_pub.messages)

    def test_missing_log_directory_permissions_do_not_block_startup(self):
        with patch.object(vision.QRDetectorNode, "prepare_log",
                          side_effect=PermissionError("read-only directory")):
            node = vision.QRDetectorNode()
        self.assertFalse(node.scanning_enabled)
        node.get_logger().warning.assert_called()

    def test_csv_failure_does_not_block_result(self):
        def fail_save(*args):
            self.assertEqual(len(self.node.data_pub.messages), 1)
            raise OSError("disk full")

        with patch.object(self.node, "save_result", side_effect=fail_save):
            self.start()
            self.frame()
            self.frame()
        self.assertFalse(self.node.scanning_enabled)
        self.assertEqual(len(self.node.data_pub.messages), 1)

    def test_no_debug_subscriber_skips_debug_conversion(self):
        self.start()
        with patch.object(self.node.bridge, "cv2_to_imgmsg") as convert:
            self.frame()
            convert.assert_not_called()

    def test_debug_failure_does_not_block_confirmed_result(self):
        self.node.debug_pub.subscribers = 1
        self.start()
        with patch.object(self.node.bridge, "cv2_to_imgmsg",
                          side_effect=RuntimeError("debug conversion failed")):
            self.frame()
            self.frame()
        self.assertEqual(len(self.node.data_pub.messages), 1)

    def test_fallback_accepts_valid_code_without_polygon(self):
        self.node.opencv_detector = Mock()
        self.node.opencv_detector.detectAndDecodeMulti.return_value = (False, (), None, ())
        self.node.opencv_detector.detectAndDecode.return_value = ("END", None, None)
        self.assertEqual(self.node.detect_qr(None)[0], "END")

    def test_unrelated_wechat_result_does_not_suppress_valid_opencv_result(self):
        with patch.object(self.node, "detect_with_wechat", return_value=[
                ("https://example.org", None, "WeChatQRCode")]):
            self.assertEqual(self.node.detect_qr(qr_image("START"))[0], "START")

    def test_multi_detection_chooses_largest_valid_marker(self):
        square = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float32)
        self.node.opencv_detector = Mock()
        self.node.opencv_detector.detectAndDecodeMulti.return_value = (
            True, ("random", "START", "END"),
            np.array([square * 5, square, square * 2]), ())
        self.assertEqual(self.node.detect_qr(None)[0], "END")

    def test_invalid_confirmation_parameters_rejected(self):
        cases = [("process_hz", 0.0), ("process_hz", math.nan),
                 ("confirm_window", math.inf), ("confirm_window", 0.0),
                 ("confirm_hits", 0), ("confirm_hits", 100)]
        for name, value in cases:
            with self.subTest(name=name), patch.dict(Node.overrides, {name: value}):
                with self.assertRaises(ValueError):
                    vision.QRDetectorNode()


if __name__ == "__main__":
    unittest.main()
