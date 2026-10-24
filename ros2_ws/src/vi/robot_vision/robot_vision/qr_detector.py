"""Detect and publish the structured QR payloads used in the game."""

import re
from typing import Iterable

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


QR_PATTERN = re.compile(r"^(?:START|END|RACK[A-D]_[A-Z0-9]{4})$")


class QrDetector(Node):
    """Decode valid game markers from a ROS image stream."""

    def __init__(self) -> None:
        super().__init__("qr_detector")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("qr_topic", "/vision/qr_code")
        self.declare_parameter("debug_image_topic", "/vision/annotated_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("republish_period_s", 0.5)

        image_topic = str(self.get_parameter("image_topic").value)
        qr_topic = str(self.get_parameter("qr_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)
        self._republish_period_ns = int(
            float(self.get_parameter("republish_period_s").value) * 1_000_000_000
        )
        if self._republish_period_ns < 0:
            raise ValueError("republish_period_s must be non-negative")

        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()
        self._last_payload = ""
        self._last_publish_ns = 0
        self._last_invalid_payload = ""

        self._qr_pub = self.create_publisher(String, qr_topic, QoSProfile(depth=10))
        self._debug_pub = self.create_publisher(
            Image, debug_topic, qos_profile_sensor_data
        )
        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"QR detector listening on {image_topic}; publishing valid codes on {qr_topic}"
        )

    def _decode(self, image) -> tuple[list[str], list]:
        """Return decoded strings and their corner points across OpenCV versions."""
        try:
            ok, values, points, _ = self._detector.detectAndDecodeMulti(image)
            if ok and points is not None:
                return [value.strip().upper() for value in values], list(points)
        except (AttributeError, cv2.error):
            pass

        value, points, _ = self._detector.detectAndDecode(image)
        if value and points is not None:
            return [value.strip().upper()], [points.reshape(-1, 2)]
        return [], []

    @staticmethod
    def _area(corners) -> float:
        try:
            return abs(float(cv2.contourArea(corners.astype("float32"))))
        except (AttributeError, cv2.error, TypeError):
            return 0.0

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert camera image: {exc}")
            return

        payloads, corners = self._decode(frame)
        valid = [
            (payload, points)
            for payload, points in zip(payloads, corners)
            if QR_PATTERN.fullmatch(payload)
        ]
        selected = max(valid, key=lambda item: self._area(item[1]), default=None)

        if selected is not None:
            payload, points = selected
            now_ns = self.get_clock().now().nanoseconds
            should_publish = (
                payload != self._last_payload
                or now_ns - self._last_publish_ns >= self._republish_period_ns
            )
            if should_publish:
                out = String()
                out.data = payload
                self._qr_pub.publish(out)
                self._last_payload = payload
                self._last_publish_ns = now_ns
            self._last_invalid_payload = ""
            if self._publish_debug:
                polygon = points.astype("int32").reshape((-1, 1, 2))
                cv2.polylines(frame, [polygon], True, (0, 220, 0), 2)
                x, y = polygon[0, 0]
                cv2.putText(
                    frame,
                    payload,
                    (int(x), max(20, int(y) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
        else:
            self._last_payload = ""
            invalid = next((value for value in payloads if value), "")
            if invalid and invalid != self._last_invalid_payload:
                self.get_logger().warning(
                    f"QR decoded but does not match START, END, or RACK[A-D]_XXXX: {invalid}"
                )
                self._last_invalid_payload = invalid

        if self._publish_debug:
            try:
                annotated = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                annotated.header = msg.header
                self._debug_pub.publish(annotated)
            except CvBridgeError as exc:
                self.get_logger().warning(f"Could not publish debug image: {exc}")


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = QrDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
