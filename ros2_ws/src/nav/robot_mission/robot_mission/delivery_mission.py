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


class MissionState(Enum):
    WAIT_START = auto()
    NAV_TO_RACK = auto()
    WAIT_RACK_QR = auto()
    LIFT_RACK = auto()
    NAV_TO_DESTINATION = auto()
    LOWER_RACK = auto()
    COMPLETE = auto()
    ERROR = auto()


class DeliveryMission(Node):
    """Parameter-driven mission logic; physical poses must be confirmed first."""

    def __init__(self) -> None:
        super().__init__("delivery_mission")
        self.declare_parameter("coordinates_confirmed", False)
        self.declare_parameter("rack_labels", ["A", "B", "C", "D"])
        self.declare_parameter("rack_x_m", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("rack_y_m", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("rack_yaw_rad", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("destination_x_m", 0.0)
        self.declare_parameter("destination_y_m", 0.0)
        self.declare_parameter("destination_yaw_rad", 0.0)
        self.declare_parameter("qr_topic", "/vision/qr_code")
        self.declare_parameter("lift_command_topic", "/lift_cmd")
        self.declare_parameter("lift_state_topic", "/lift_state")
        self.declare_parameter("qr_timeout_s", 15.0)
        self.declare_parameter("lift_timeout_s", 8.0)
        self.declare_parameter("navigation_timeout_s", 90.0)

        self._coordinates_confirmed = bool(
            self.get_parameter("coordinates_confirmed").value
        )
        self._labels = list(self.get_parameter("rack_labels").value)
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
        if min(self._qr_timeout, self._lift_timeout, self._navigation_timeout) <= 0.0:
            raise ValueError("mission timeouts must be greater than zero")

        self._state = MissionState.WAIT_START
        self._state_entered = time.monotonic()
        self._rack_index = 0
        self._qr_received = ""
        self._lift_state = -1
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._goal_sent = False
        self._lift_command_sent = False

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
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
        self._lift_command_sent = False
        if state == MissionState.WAIT_RACK_QR:
            # Require a fresh marker after arriving at the rack approach pose.
            self._qr_received = ""
        self.get_logger().info(f"Mission state: {state.name}")

    def _fail(self, reason: str) -> None:
        self.get_logger().error(reason)
        goal_handle = self._goal_handle
        self._transition(MissionState.ERROR)
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

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
            self._fail(f"Navigation goal request failed: {exc}")
            return
        if not self._goal_handle.accepted:
            self._fail("Nav2 rejected the mission goal")
            return
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
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
            self._transition(MissionState.LOWER_RACK)

    def _publish_lift_command(self, command: int) -> None:
        msg = Int8()
        msg.data = command
        self._lift_pub.publish(msg)

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._state_entered

        if self._state == MissionState.WAIT_START:
            if self._qr_received == "START":
                self._qr_received = ""
                if not self._coordinates_confirmed:
                    self.get_logger().error(
                        "START detected, but mission poses are still unconfirmed; "
                        "set coordinates_confirmed after entering measured values."
                    )
                    return
                self._transition(MissionState.NAV_TO_RACK)

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
            expected_prefix = f"RACK{self._labels[self._rack_index]}_"
            if self._qr_received.startswith(expected_prefix):
                self.get_logger().info(f"Confirmed rack marker: {self._qr_received}")
                self._qr_received = ""
                self._transition(MissionState.LIFT_RACK)
            elif elapsed > self._qr_timeout:
                self._fail(f"Timed out waiting for QR marker {expected_prefix}XXXX")

        elif self._state == MissionState.LIFT_RACK:
            self._run_lift_step(command=1, expected_state=1, elapsed=elapsed)

        elif self._state == MissionState.NAV_TO_DESTINATION:
            if elapsed > self._navigation_timeout:
                self._fail("Timed out while navigating to the destination")
                return
            if not self._goal_sent:
                self._send_navigation_goal(*self._destination)

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
