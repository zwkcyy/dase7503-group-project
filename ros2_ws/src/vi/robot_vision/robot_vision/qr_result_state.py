"""Retain confirmed QR results independently of preview frames and the GUI.

Adapted from the result retention feature on vision/qr-popup (60a40e3).
"""

from dataclasses import dataclass
from datetime import datetime
import re
from threading import Lock
import time
from typing import Any


def describe_payload(payload: str) -> str:
    """Describe known formats without changing the case-sensitive payload."""
    if payload == "START":
        return "起点 / Start"
    if payload == "END":
        return "终点 / Destination"
    match = re.fullmatch(r"RACK([A-D])_([A-Za-z0-9]+)", payload)
    if match:
        return f"货架 / Rack {match[1]}    编号 / ID: {match[2]}"
    return "其他二维码内容 / Other QR payload"


@dataclass(frozen=True)
class ScanResult:
    payload: str
    description: str
    received_at: str


@dataclass(frozen=True)
class PreviewFrame:
    image: Any
    received_at: float
    sequence: int


@dataclass(frozen=True)
class ViewerSnapshot:
    result: ScanResult | None
    frame: PreviewFrame | None
    status: str
    preview_error: str


class ViewerState:
    """Keep one result and two bounded preview slots across ROS/Tk threads."""

    def __init__(self):
        self._lock = Lock()
        self._result = None
        self._raw_frame = None
        self._debug_frame = None
        self._sequence = 0
        self._status = "正在连接 / Connecting..."
        self._preview_error = ""

    def receive_result(self, payload: str) -> None:
        # Preview updates and empty messages must never erase a confirmed result.
        if not payload.strip():
            return
        with self._lock:
            if self._result is not None and self._result.payload == payload:
                return
            self._result = ScanResult(
                payload,
                describe_payload(payload),
                datetime.now().astimezone().isoformat(timespec="seconds"),
            )

    def receive_frame(self, image: Any, *, annotated: bool = False) -> None:
        with self._lock:
            self._sequence += 1
            frame = PreviewFrame(image, time.monotonic(), self._sequence)
            if annotated:
                self._debug_frame = frame
            else:
                self._raw_frame = frame
            self._preview_error = ""

    def set_status(self, message: str) -> None:
        with self._lock:
            self._status = message

    def set_preview_error(self, message: str) -> None:
        with self._lock:
            self._preview_error = message

    def snapshot(self) -> ViewerSnapshot:
        with self._lock:
            frame = self._raw_frame
            # Show the detector's annotation briefly while scanning, then return
            # to live camera frames when the on-demand detector becomes idle.
            if self._debug_frame is not None and (
                frame is None
                or time.monotonic() - self._debug_frame.received_at <= 0.35
            ):
                frame = self._debug_frame
            return ViewerSnapshot(
                self._result, frame, self._status, self._preview_error,
            )
