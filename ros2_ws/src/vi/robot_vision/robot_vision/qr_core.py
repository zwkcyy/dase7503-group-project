"""QR decoding and result retention, independent of ROS and the GUI."""

from dataclasses import dataclass
from datetime import datetime
import re

import cv2
import numpy as np


@dataclass(frozen=True)
class ScanResult:
    payload: str
    description: str
    scanned_at: str


def describe_payload(payload: str) -> str:
    """Describe known formats without modifying the original QR payload."""
    if payload == "START":
        return "起点 / Start"
    if payload == "END":
        return "终点 / End"
    match = re.fullmatch(r"RACK([A-D])_([A-Za-z0-9]{4})", payload)
    if match:
        return f"货架 / Rack {match[1]}    编号 / ID: {match[2]}"
    return "其他二维码内容 / Other QR payload"


class ResultLatch:
    """Retain the latest result; missing or repeated codes do not refresh it."""

    def __init__(self):
        self.current = None

    def update(self, payload: str) -> bool:
        if not payload or (self.current and payload == self.current.payload):
            return False
        self.current = ScanResult(
            payload, describe_payload(payload),
            datetime.now().astimezone().isoformat(timespec="seconds"),
        )
        return True


class QRDecoder:
    def __init__(self):
        self.detector = cv2.QRCodeDetector()

    def decode(self, frame: np.ndarray) -> tuple[str, np.ndarray | None]:
        """Choose the decodable code closest to the image center."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        candidates = []
        ok, texts, points, _ = self.detector.detectAndDecodeMulti(gray)
        if ok and points is not None:
            candidates = [(text, quad) for text, quad in zip(texts, points) if text]
        # Fall back to single-code decoding if multi-code decoding fails.
        if not candidates:
            text, points, _ = self.detector.detectAndDecode(gray)
            if text and points is not None:
                candidates = [(text, points.reshape(4, 2))]
        if not candidates:
            return "", None
        height, width = gray.shape
        center = np.array([width / 2, height / 2])
        return min(candidates, key=lambda item: (
            float(np.sum((item[1].mean(axis=0) - center) ** 2)), item[0]
        ))
