"""Test actual QR decoding and result retention without a camera or display."""

import unittest

import cv2
import numpy as np

from robot_vision.qr_core import QRDecoder, ResultLatch, describe_payload


def qr_image(text):
    matrix = cv2.QRCodeEncoder_create().encode(text)
    # Add a white quiet zone and use nearest-neighbor scaling to keep edges sharp.
    matrix = cv2.copyMakeBorder(matrix, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
    matrix = cv2.resize(matrix, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST)
    return cv2.cvtColor(matrix, cv2.COLOR_GRAY2BGR)


class QRTests(unittest.TestCase):
    def test_project_payloads_really_decode(self):
        decoder = QRDecoder()
        for text in ["START", "RACKA_4X6M", "RACKB_AM9L", "RACKC_K8Y3", "RACKD_R983", "END"]:
            with self.subTest(text=text):
                payload, corners = decoder.decode(qr_image(text))
                self.assertEqual(payload, text)
                self.assertEqual(corners.shape, (4, 2))

    def test_blank_and_repeat_keep_previous_result(self):
        latch = ResultLatch()
        self.assertFalse(latch.update(""))
        self.assertIsNone(latch.current)
        self.assertTrue(latch.update("START"))
        first = latch.current
        self.assertFalse(latch.update(""))
        self.assertFalse(latch.update("START"))
        self.assertIs(latch.current, first)
        self.assertTrue(latch.update("RACKA_4X6M"))
        self.assertEqual(latch.current.payload, "RACKA_4X6M")
        self.assertTrue(latch.update("START"))
        self.assertEqual(latch.current.payload, "START")

    def test_blank_image_has_no_result(self):
        payload, corners = QRDecoder().decode(np.full((480, 640, 3), 255, np.uint8))
        self.assertEqual(payload, "")
        self.assertIsNone(corners)

    def test_multiple_codes_choose_center_consistently(self):
        frame = np.full((600, 1000, 3), 255, np.uint8)
        for text, x, y in [("START", 25, 25), ("END", 380, 180)]:
            image = qr_image(text)
            h, w = image.shape[:2]
            frame[y:y+h, x:x+w] = image
        decoder = QRDecoder()
        for _ in range(3):
            self.assertEqual(decoder.decode(frame)[0], "END")

    def test_other_payload_is_preserved(self):
        latch = ResultLatch()
        self.assertTrue(latch.update("hello\nrobot"))
        self.assertEqual(latch.current.payload, "hello\nrobot")
        self.assertIn("Other", describe_payload("hello\nrobot"))


if __name__ == "__main__":
    unittest.main()
