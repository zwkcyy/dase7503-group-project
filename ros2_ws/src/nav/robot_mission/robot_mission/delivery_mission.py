"""Run the QR-gated rack pickup and delivery sequence using Nav2."""

import math
import time
from enum import Enum, auto
from typing import Iterable

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger


class MissionState(Enum):
    WAIT_START = auto()
    NAV_TO_RACK = auto()
    WAIT_RACK_QR = auto()
    LIFT_RACK = auto()
    NAV_TO_DESTINATION = auto()
    VERIFY_DESTINATION = auto()
    LOWER_RACK = auto()
    COMPLETE = auto()
    ERROR = auto()


class DeliveryMission(Node):
    """Parameter-driven mission logic; physical poses must be confirmed first."""

    def __init__(self) -> None:
        super().__init__("delivery_mission")
        self.declare_parameter("coordinates_confirmed", False)
        self.declare_parameter("rack_labels", ["A", "B", "C", "D"])
        self.declare_parameter("rack_qr_codes", [])
        self.declare_parameter("rack_x_m", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("rack_y_m", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("rack_yaw_rad", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("destination_x_m", 0.0)
        self.declare_parameter("destination_y_m", 0.0)
        self.declare_parameter("destination_yaw_rad", 0.0)
        self.declare_parameter("qr_topic", "/qr/data")
        self.declare_parameter("qr_start_service", "/qr/start_scan")
        self.declare_parameter("qr_stop_service", "/qr/stop_scan")
        self.declare_parameter("lift_command_topic", "/lift_cmd")
        self.declare_parameter("lift_state_topic", "/lift_state")
        self.declare_parameter("qr_timeout_s", 15.0)
        self.declare_parameter("lift_timeout_s", 8.0)
        self.declare_parameter("navigation_timeout_s", 90.0)
        self.declare_parameter("competition_time_s", 180.0)

        self._coordinates_confirmed = bool(
            self.get_parameter("coordinates_confirmed").value
        )
        self._labels = list(self.get_parameter("rack_labels").value)
        self._rack_qr_codes = [
            str(code).strip().upper()
            for code in self.get_parameter("rack_qr_codes").value
        ]
        self._rack_x = list(self.get_parameter("rack_x_m").value)
        self._rack_y = list(self.get_parameter("rack_y_m").value)
        self._rack_yaw = list(self.get_parameter("rack_yaw_rad").value)
        self._destination = (
            float(self.get_parameter("destination_x_m").value),
            float(self.get_parameter("destination_y_m").value),
            float(self.get_parameter("destination_yaw_rad").value),
        )
        self._qr_timeout = float(self.get_parameter("qr_timeout_s").value)
        self._lift_timeout = float(self.get_parameter("lift_timeout_s").value)
        self._navigation_timeout = float(
            self.get_parameter("navigation_timeout_s").value
        )
        self._competition_time = float(
            self.get_parameter("competition_time_s").value
        )

        if not self._labels or not (
            len(self._labels)
            == len(self._rack_x)
            == len(self._rack_y)
            == len(self._rack_yaw)
        ):
            raise ValueError("rack_labels and all rack pose arrays must have equal nonzero length")
        if any(label not in {"A", "B", "C", "D"} for label in self._labels):
            raise ValueError("rack_labels entries must be one of A, B, C, or D")
        if len(set(self._labels)) != len(self._labels):
            raise ValueError("rack_labels entries must be unique")
        if self._coordinates_confirmed:
            if len(self._rack_qr_codes) != len(self._labels):
                raise ValueError(
                    "Set one full rack_qr_codes value per rack before confirming "
                    "mission coordinates"
                )
            for label, code in zip(self._labels, self._rack_qr_codes):
                prefix = f"RACK{label}_"
                suffix = code[len(prefix) :] if code.startswith(prefix) else ""
                if not suffix.isascii() or not suffix.isalnum():
                    raise ValueError(
                        f"rack_qr_codes must contain the full RACK{label}_... code"
                    )
            if len(set(self._rack_qr_codes)) != len(self._rack_qr_codes):
                raise ValueError("rack_qr_codes entries must be unique")
        if min(
            self._qr_timeout,
            self._lift_timeout,
            self._navigation_timeout,
            self._competition_time,
        ) <= 0.0:
            raise ValueError("mission timeouts must be greater than zero")

        self._state = MissionState.WAIT_START
        self._state_entered = time.monotonic()
        self._rack_index = 0
        self._competition_started_at = None
        self._qr_received = ""
        self._lift_state = -1
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._goal_sent = False
        self._scan_requested = False
        self._scan_stop_pending = False
        self._scan_retry_at = 0.0

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._qr_scan_client = self.create_client(
            Trigger, str(self.get_parameter("qr_start_service").value)
        )
        self._qr_stop_client = self.create_client(
            Trigger, str(self.get_parameter("qr_stop_service").value)
        )
        self._qr_sub = self.create_subscription(
            String,
            str(self.get_parameter("qr_topic").value),
            self._on_qr,
            10,
        )
        self._lift_state_sub = self.create_subscription(
            Int8,
            str(self.get_parameter("lift_state_topic").value),
            self._on_lift_state,
            10,
        )
        self._lift_pub = self.create_publisher(
            Int8, str(self.get_parameter("lift_command_topic").value), 10
        )
        self._timer = self.create_timer(0.1, self._tick)

        if not self._coordinates_confirmed:
            self.get_logger().warning(
                "Mission is waiting for START. Set coordinates_confirmed=true only "
                "after measuring all rack and destination poses."
            )
        else:
            self.get_logger().info("Mission ready; scan START to begin the rack sequence.")

    def _on_qr(self, msg: String) -> None:
        self._qr_received = msg.data.strip().upper()

    def _on_lift_state(self, msg: Int8) -> None:
        self._lift_state = int(msg.data)

    def _transition(self, state: MissionState) -> None:
        self._state = state
        self._state_entered = time.monotonic()
        self._goal_sent = False
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._scan_requested = False
        self._scan_retry_at = 0.0
        if state in {
            MissionState.WAIT_START,
            MissionState.WAIT_RACK_QR,
            MissionState.VERIFY_DESTINATION,
        }:
            # The scanner publishes one result per request; clear any stale result.
            self._qr_received = ""
        self.get_logger().info(f"Mission state: {state.name}")

    def _fail(self, reason: str) -> None:
        if self._state in {MissionState.COMPLETE, MissionState.ERROR}:
            return
        self.get_logger().error(reason)
        goal_handle = self._goal_handle
        self._scan_stop_pending = self._scan_requested
        self._transition(MissionState.ERROR)
        self._try_stop_qr_scan()
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def _try_stop_qr_scan(self) -> None:
        if not self._scan_stop_pending or not self._qr_stop_client.service_is_ready():
            return
        self._scan_stop_pending = False
        future = self._qr_stop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_scan_stop_response)

    def _on_scan_stop_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"QR stop request failed; will retry: {exc}")
            self._scan_stop_pending = True
            return
        if not response.success:
            # The scanner may already have returned to IDLE after publishing.
            self.get_logger().debug(
                f"QR scanner is already idle: {response.message}"
            )

    def _pose_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose
        return goal

    def _send_navigation_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._nav_client.server_is_ready():
            if not self._nav_client.wait_for_server(timeout_sec=0.0):
                return
        goal = self._pose_goal(x, y, yaw)
        self._goal_future = self._nav_client.send_goal_async(goal)
        self._goal_future.add_done_callback(self._on_goal_response)
        self._goal_sent = True

    def _on_goal_response(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as exc:  # surface action transport errors in the mission state
            if self._state not in {MissionState.COMPLETE, MissionState.ERROR}:
                self._fail(f"Navigation goal request failed: {exc}")
            return
        if self._state not in {
            MissionState.NAV_TO_RACK,
            MissionState.NAV_TO_DESTINATION,
        }:
            if self._goal_handle.accepted:
                self._goal_handle.cancel_goal_async()
            return
        if not self._goal_handle.accepted:
            self._fail("Nav2 rejected the mission goal")
            return
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        if self._state not in {
            MissionState.NAV_TO_RACK,
            MissionState.NAV_TO_DESTINATION,
        }:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self._fail(f"Could not read navigation result: {exc}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f"Navigation ended with action status {status}")
            return

        if self._state == MissionState.NAV_TO_RACK:
            self._transition(MissionState.WAIT_RACK_QR)
        elif self._state == MissionState.NAV_TO_DESTINATION:
            self._transition(MissionState.VERIFY_DESTINATION)

    def _publish_lift_command(self, command: int) -> None:
        msg = Int8()
        msg.data = command
        self._lift_pub.publish(msg)

    def _request_qr_scan(self) -> None:
        if (
            self._scan_requested
            or time.monotonic() < self._scan_retry_at
            or not self._qr_scan_client.service_is_ready()
        ):
            return
        self._scan_requested = True
        future = self._qr_scan_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_scan_start_response)

    def _retry_after_unexpected_qr(self, expected: str) -> None:
        if not self._qr_received:
            return
        received = self._qr_received
        self._qr_received = ""
        self._scan_requested = False
        # The vision node becomes idle immediately after publishing a result.
        # Leave a short gap before asking it to start another scan.
        self._scan_retry_at = time.monotonic() + 0.25
        self.get_logger().warning(
            f"Ignoring QR marker {received!r}; expected {expected!r}. Retrying scan."
        )

    def _on_scan_start_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            if self._state not in {MissionState.COMPLETE, MissionState.ERROR}:
                self._fail(f"QR scan request failed: {exc}")
            return
        if self._state in {MissionState.COMPLETE, MissionState.ERROR}:
            if response.success:
                self._scan_stop_pending = True
                self._try_stop_qr_scan()
            return
        if not response.success:
            self._fail(f"QR scanner refused request: {response.message}")

    def _tick(self) -> None:
        if self._state in {MissionState.COMPLETE, MissionState.ERROR}:
            self._try_stop_qr_scan()
            return

        elapsed = time.monotonic() - self._state_entered

        if (
            self._competition_started_at is not None
            and self._state not in {MissionState.COMPLETE, MissionState.ERROR}
            and time.monotonic() - self._competition_started_at
            >= self._competition_time
        ):
            self._fail(
                f"Competition time limit reached ({self._competition_time:.0f} s); "
                "navigation will stop and the lift will remain in its current state"
            )
            return

        if self._state == MissionState.WAIT_START:
            if not self._coordinates_confirmed:
                return
            self._request_qr_scan()
            if self._qr_received == "START":
                self._qr_received = ""
                self._competition_started_at = time.monotonic()
                self.get_logger().info(
                    f"Competition timer started ({self._competition_time:.0f} s)"
                )
                self._transition(MissionState.NAV_TO_RACK)
            elif self._qr_received:
                self._retry_after_unexpected_qr("START")
            elif elapsed > self._qr_timeout:
                self._fail("Timed out waiting for the START QR marker")

        elif self._state == MissionState.NAV_TO_RACK:
            if elapsed > self._navigation_timeout:
                self._fail("Timed out while navigating to the rack")
                return
            if not self._goal_sent:
                self._send_navigation_goal(
                    self._rack_x[self._rack_index],
                    self._rack_y[self._rack_index],
                    self._rack_yaw[self._rack_index],
                )

        elif self._state == MissionState.WAIT_RACK_QR:
            self._request_qr_scan()
            expected_code = self._rack_qr_codes[self._rack_index]
            if self._qr_received == expected_code:
                self.get_logger().info(f"Confirmed rack marker: {self._qr_received}")
                self._qr_received = ""
                self._transition(MissionState.LIFT_RACK)
            elif self._qr_received:
                self._retry_after_unexpected_qr(expected_code)
            elif elapsed > self._qr_timeout:
                self._fail(f"Timed out waiting for QR marker {expected_code}")

        elif self._state == MissionState.LIFT_RACK:
            self._run_lift_step(command=1, expected_state=1, elapsed=elapsed)

        elif self._state == MissionState.NAV_TO_DESTINATION:
            if elapsed > self._navigation_timeout:
                self._fail("Timed out while navigating to the destination")
                return
            if not self._goal_sent:
                self._send_navigation_goal(*self._destination)

        elif self._state == MissionState.VERIFY_DESTINATION:
            self._request_qr_scan()
            if self._qr_received == "END":
                self._qr_received = ""
                self._transition(MissionState.LOWER_RACK)
            elif self._qr_received:
                self._retry_after_unexpected_qr("END")
            elif elapsed > self._qr_timeout:
                self._fail("Timed out waiting for the END QR marker")

        elif self._state == MissionState.LOWER_RACK:
            self._run_lift_step(command=0, expected_state=0, elapsed=elapsed)

    def _run_lift_step(self, command: int, expected_state: int, elapsed: float) -> None:
        if self._lift_state == expected_state:
            if command == 1:
                self._transition(MissionState.NAV_TO_DESTINATION)
            else:
                self.get_logger().info(
                    f"Delivered rack {self._labels[self._rack_index]}"
                )
                self._rack_index += 1
                if self._rack_index >= len(self._labels):
                    elapsed = (
                        time.monotonic() - self._competition_started_at
                        if self._competition_started_at is not None
                        else 0.0
                    )
                    self.get_logger().info(
                        f"Mission complete: delivered {len(self._labels)} racks "
                        f"in {elapsed:.1f} s"
                    )
                    self._transition(MissionState.COMPLETE)
                else:
                    self._transition(MissionState.NAV_TO_RACK)
            return

        if elapsed > self._lift_timeout:
            self._fail(f"Timed out waiting for lift state {expected_state}")
            return

        # Reissue at 2 Hz to tolerate a command dropped during serial startup.
        if int(elapsed * 2) != int(max(0.0, elapsed - 0.1) * 2):
            self._publish_lift_command(command)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DeliveryMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
