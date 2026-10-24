"""Run the QR-gated rack pickup and delivery sequence using Nav2."""

import math
import signal
import time
from enum import Enum, auto
from typing import Iterable

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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


QR_STATES = {MissionState.WAIT_START, MissionState.WAIT_RACK_QR,
             MissionState.VERIFY_DESTINATION}
NAV_STATES = {MissionState.NAV_TO_RACK, MissionState.NAV_TO_DESTINATION}
TERMINAL_STATES = {MissionState.COMPLETE, MissionState.ERROR}


class DeliveryMission(Node):
    """Parameter-driven mission logic; physical poses must be confirmed first."""

    def __init__(self) -> None:
        super().__init__("delivery_mission")
        self.declare_parameter("coordinates_confirmed", False)
        self.declare_parameter("rack_labels", ["A", "B", "C", "D"])
        # A nonempty string array avoids ROS inferring an empty byte array.
        self.declare_parameter("rack_qr_codes", ["", "", "", ""])
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
        self.declare_parameter("start_timeout_s", 0.0)
        self.declare_parameter("request_timeout_s", 2.0)
        self.declare_parameter("lift_timeout_s", 8.0)
        self.declare_parameter("navigation_timeout_s", 90.0)
        self.declare_parameter("competition_time_s", 180.0)

        self._coordinates_confirmed = bool(
            self.get_parameter("coordinates_confirmed").value
        )
        self._labels = list(self.get_parameter("rack_labels").value)
        self._rack_qr_codes = [
            str(code).strip()
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
        self._start_timeout = float(self.get_parameter("start_timeout_s").value)
        self._request_timeout = float(
            self.get_parameter("request_timeout_s").value
        )
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
        timeouts = (
            self._qr_timeout,
            self._request_timeout,
            self._lift_timeout,
            self._navigation_timeout,
            self._competition_time,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in timeouts):
            raise ValueError("mission timeouts must be finite and greater than zero")
        if not math.isfinite(self._start_timeout) or self._start_timeout < 0.0:
            raise ValueError("start_timeout_s must be finite and nonnegative")
        poses = [*self._rack_x, *self._rack_y, *self._rack_yaw, *self._destination]
        if any(not math.isfinite(value) for value in poses):
            raise ValueError("all mission poses must be finite")

        self._state = MissionState.WAIT_START
        self._generation = 0
        self._state_entered = time.monotonic()
        self._rack_index = 0
        self._competition_started_at = None
        self._qr_received = ""
        self._lift_state = -1
        self._lift_feedback_sequence = 0
        self._lift_command_sequence = None
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._goal_sent = False
        self._pending_goal_requests = set()
        self._cancel_handles = []
        self._cancel_future = None
        self._cancel_sent_at = 0.0
        self._cancel_retry_at = 0.0
        self._scan_requested = False
        self._scan_future = None
        self._scan_sent_at = 0.0
        self._scan_stop_pending = False
        self._scan_stop_future = None
        self._scan_stop_sent_at = 0.0
        self._scan_stop_retry_at = 0.0
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
        if self._state in QR_STATES and self._scan_requested and msg.data.strip():
            # The random suffix is case-sensitive; do not normalize its case.
            self._qr_received = msg.data.strip()
            self._scan_requested = False
            # A result proves the start request was processed. Its service reply
            # may arrive later on a different DDS channel; it is no longer needed.
            if self._scan_future is not None:
                future, self._scan_future = self._scan_future, None
                self._qr_scan_client.remove_pending_request(future)
                future.cancel()

    def _on_lift_state(self, msg: Int8) -> None:
        self._lift_state = int(msg.data)
        self._lift_feedback_sequence += 1

    def _transition(self, state: MissionState) -> None:
        self._state = state
        self._generation += 1
        self._state_entered = time.monotonic()
        self._goal_sent = False
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._scan_requested = False
        self._scan_retry_at = 0.0
        self._lift_command_sequence = None
        if state in QR_STATES:
            # The scanner publishes one result per request; clear any stale result.
            self._qr_received = ""
        self.get_logger().info(f"Mission state: {state.name}")

    def _fail(self, reason: str) -> None:
        if self._state in TERMINAL_STATES:
            return
        self.get_logger().error(reason)
        goal_handle = self._goal_handle
        self._scan_stop_pending = True
        self._transition(MissionState.ERROR)
        self._try_stop_qr_scan()
        if goal_handle is not None:
            self._queue_navigation_cancel(goal_handle)

    def request_stop(self) -> None:
        """Stop the mission while the ROS context can still send requests."""
        self._fail("Mission stopped by operator; keeping the lift in its current state")

    def cleanup_pending(self) -> bool:
        return bool(
            self._pending_goal_requests or self._cancel_handles
            or self._scan_future or self._scan_stop_pending or self._scan_stop_future
        )

    def _queue_navigation_cancel(self, handle) -> None:
        if handle not in self._cancel_handles:
            self._cancel_handles.append(handle)
        self._try_cancel_navigation()

    def _try_cancel_navigation(self) -> None:
        now = time.monotonic()
        if self._cancel_future is not None:
            if now - self._cancel_sent_at < self._request_timeout:
                return
            future, self._cancel_future = self._cancel_future, None
            future.cancel()
            self.get_logger().warning("Navigation cancel response timed out; retrying")
            self._cancel_retry_at = now + 0.5
        if not self._cancel_handles or now < self._cancel_retry_at:
            return
        self._cancel_sent_at = now
        try:
            self._cancel_future = self._cancel_handles[0].cancel_goal_async()
            self._cancel_future.add_done_callback(self._on_navigation_cancel)
        except Exception as exc:
            self._cancel_future = None
            self._cancel_retry_at = now + 0.5
            self.get_logger().warning(f"Navigation cancel failed; will retry: {exc}")

    def _on_navigation_cancel(self, future) -> None:
        if future is not self._cancel_future:
            return
        self._cancel_future = None
        self._cancel_retry_at = time.monotonic() + 0.5
        try:
            response = future.result()
            # action_msgs/CancelGoal: 0 = accepted, 3 = already terminated.
            if response.return_code in (0, 3):
                self._cancel_handles.pop(0)
            else:
                self.get_logger().warning(
                    f"Navigation cancel refused ({response.return_code}); will retry"
                )
        except Exception as exc:
            self.get_logger().warning(f"Navigation cancel failed; will retry: {exc}")

    def _try_stop_qr_scan(self) -> None:
        now = time.monotonic()
        if self._scan_stop_future is not None:
            if now - self._scan_stop_sent_at < self._request_timeout:
                return
            future, self._scan_stop_future = self._scan_stop_future, None
            self._qr_stop_client.remove_pending_request(future)
            future.cancel()
            self._scan_stop_pending = True
            self.get_logger().warning("QR stop response timed out; retrying")
        if (not self._scan_stop_pending or now < self._scan_stop_retry_at
                or not self._qr_stop_client.service_is_ready()):
            return
        self._scan_stop_sent_at = now
        self._scan_stop_retry_at = now + 0.5
        try:
            self._scan_stop_future = self._qr_stop_client.call_async(Trigger.Request())
            self._scan_stop_pending = False
            self._scan_stop_future.add_done_callback(self._on_scan_stop_response)
        except Exception as exc:
            self.get_logger().warning(f"QR stop request failed; will retry: {exc}")

    def _on_scan_stop_response(self, future) -> None:
        if future is not self._scan_stop_future:
            return
        self._scan_stop_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"QR stop request failed; will retry: {exc}")
            self._scan_stop_pending = True
            return
        if not response.success:
            self._scan_stop_pending = True
            self.get_logger().warning(f"QR stop refused; will retry: {response.message}")

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
            return
        goal = self._pose_goal(x, y, yaw)
        generation = self._generation
        try:
            self._goal_future = self._nav_client.send_goal_async(goal)
            self._goal_sent = True
            self._pending_goal_requests.add(self._goal_future)
            self._goal_future.add_done_callback(
                lambda future: self._on_goal_response(future, generation)
            )
        except Exception as exc:
            self._fail(f"Could not send navigation goal: {exc}")

    def _on_goal_response(self, future, generation: int) -> None:
        self._pending_goal_requests.discard(future)
        try:
            handle = future.result()
        except Exception as exc:  # surface action transport errors in the mission state
            if generation == self._generation and self._state in NAV_STATES:
                self._fail(f"Navigation goal request failed: {exc}")
            return
        if generation != self._generation or self._state not in NAV_STATES:
            if handle.accepted:
                self._queue_navigation_cancel(handle)
            return
        if not handle.accepted:
            self._fail("Nav2 rejected the mission goal")
            return
        self._goal_handle = handle
        try:
            self._result_future = handle.get_result_async()
            self._result_future.add_done_callback(
                lambda result: self._on_navigation_result(result, generation)
            )
        except Exception as exc:
            self._fail(f"Could not request navigation result: {exc}")

    def _on_navigation_result(self, future, generation: int) -> None:
        if generation != self._generation or self._state not in NAV_STATES:
            return
        if time.monotonic() - self._state_entered >= self._navigation_timeout:
            self._fail("Navigation result arrived after the step deadline")
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
            or self._scan_future is not None
            or self._qr_received
            or time.monotonic() < self._scan_retry_at
            or not self._qr_scan_client.service_is_ready()
        ):
            return
        self._scan_requested = True
        self._scan_sent_at = time.monotonic()
        generation = self._generation
        try:
            self._scan_future = self._qr_scan_client.call_async(Trigger.Request())
            self._scan_future.add_done_callback(
                lambda future: self._on_scan_start_response(future, generation)
            )
        except Exception as exc:
            self._fail(f"Could not send QR scan request: {exc}")

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

    def _on_scan_start_response(self, future, generation: int) -> None:
        if future is not self._scan_future:
            return
        self._scan_future = None
        try:
            response = future.result()
        except Exception as exc:
            if generation == self._generation and self._state in QR_STATES:
                self._fail(f"QR scan request failed: {exc}")
            return
        if self._state in TERMINAL_STATES:
            if response.success:
                self._scan_stop_pending = True
                self._try_stop_qr_scan()
            return
        if generation != self._generation:
            return
        if not response.success:
            self._fail(f"QR scanner refused request: {response.message}")

    def _tick(self) -> None:
        self._try_cancel_navigation()
        if self._state in TERMINAL_STATES:
            self._try_stop_qr_scan()
            return

        elapsed = time.monotonic() - self._state_entered

        if (
            self._competition_started_at is not None
            and time.monotonic() - self._competition_started_at
            >= self._competition_time
        ):
            self._fail(
                f"Competition time limit reached ({self._competition_time:.0f} s); "
                "navigation will stop and the lift will remain in its current state"
            )
            return

        if (self._scan_future is not None
                and time.monotonic() - self._scan_sent_at >= self._request_timeout):
            # Keep the pending future: if a delayed START succeeds after STOP,
            # its callback must send STOP again.
            self._fail("Timed out waiting for the QR start service response")
            return

        if self._state == MissionState.WAIT_START:
            if not self._coordinates_confirmed:
                return
            self._run_qr_step("START", MissionState.NAV_TO_RACK,
                              elapsed, self._start_timeout)

        elif self._state == MissionState.NAV_TO_RACK:
            if elapsed >= self._navigation_timeout:
                self._fail("Timed out while navigating to the rack")
                return
            if not self._goal_sent:
                self._send_navigation_goal(
                    self._rack_x[self._rack_index],
                    self._rack_y[self._rack_index],
                    self._rack_yaw[self._rack_index],
                )

        elif self._state == MissionState.WAIT_RACK_QR:
            expected_code = self._rack_qr_codes[self._rack_index]
            self._run_qr_step(expected_code, MissionState.LIFT_RACK,
                              elapsed, self._qr_timeout)

        elif self._state == MissionState.LIFT_RACK:
            self._run_lift_step(command=1, expected_state=1, elapsed=elapsed)

        elif self._state == MissionState.NAV_TO_DESTINATION:
            if elapsed >= self._navigation_timeout:
                self._fail("Timed out while navigating to the destination")
                return
            if not self._goal_sent:
                self._send_navigation_goal(*self._destination)

        elif self._state == MissionState.VERIFY_DESTINATION:
            self._run_qr_step("END", MissionState.LOWER_RACK,
                              elapsed, self._qr_timeout)

        elif self._state == MissionState.LOWER_RACK:
            self._run_lift_step(command=0, expected_state=0, elapsed=elapsed)

    def _run_qr_step(self, expected: str, next_state: MissionState,
                    elapsed: float, timeout: float) -> None:
        # Check before consuming results: a stream of wrong markers must not
        # keep a step alive forever, or let a late valid marker bypass a deadline.
        if timeout > 0.0 and elapsed >= timeout:
            self._fail(f"Timed out waiting for QR marker {expected}")
            return
        if self._qr_received == expected:
            self.get_logger().info(f"Confirmed QR marker: {expected}")
            if self._state == MissionState.WAIT_START:
                self._competition_started_at = time.monotonic()
                self.get_logger().info(
                    f"Competition timer started ({self._competition_time:.0f} s)"
                )
            self._qr_received = ""
            self._transition(next_state)
        elif self._qr_received:
            self._retry_after_unexpected_qr(expected)
        else:
            self._request_qr_scan()

    def _run_lift_step(self, command: int, expected_state: int, elapsed: float) -> None:
        if elapsed >= self._lift_timeout:
            self._fail(f"Timed out waiting for lift state {expected_state}")
            return
        if self._lift_command_sequence is None:
            # Wait for discovery, then publish once over reliable QoS. Repeating
            # commands can restart an EC implementation that uses a delay timer.
            if self._lift_pub.get_subscription_count() == 0:
                return
            self._lift_command_sequence = self._lift_feedback_sequence
            try:
                self._publish_lift_command(command)
            except Exception as exc:
                self._fail(f"Could not send lift command: {exc}")
            return
        if (self._lift_feedback_sequence > self._lift_command_sequence
                and self._lift_state == expected_state):
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


def main(args: Iterable[str] | None = None) -> None:
    # Keep the context alive after SIGINT/SIGTERM so cancellation can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_shutdown(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        sig: signal.signal(sig, request_shutdown)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = DeliveryMission()
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None and rclpy.ok():
                node.request_stop()
                deadline = time.monotonic() + 2.0
                while node.cleanup_pending() and rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if node.cleanup_pending():
                    node.get_logger().warning(
                        "Shutdown cleanup is unconfirmed; verify the robot has stopped"
                    )
        except ExternalShutdownException:
            pass
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    main()
