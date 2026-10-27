"""Deterministic mission regressions, with service/action/message fakes."""

import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from ros_fakes import GoalHandle, Message, Node, Trigger, load_module


mission = load_module("mission_under_test",
                      "nav/robot_mission/robot_mission/delivery_mission.py", True)
State = mission.MissionState


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        timer = patch.object(mission.time, "monotonic", lambda: self.now)
        timer.start()
        self.addCleanup(timer.stop)
        overrides = patch.object(Node, "overrides", {
            "coordinates_confirmed": True,
            "manage_load_envelope": False,
            "rack_qr_codes": [f"RACK{label}_aB12" for label in "ABCD"],
        })
        overrides.start()
        self.addCleanup(overrides.stop)
        self.node = mission.DeliveryMission()

    def qr(self, data):
        self.node._tick()
        self.node._qr_scan_client.calls[-1][1].set_result(Trigger.Response(True))
        self.node._on_qr(Message(data))
        self.node._tick()

    def navigate(self):
        self.node._tick()
        handle = GoalHandle()
        self.node._nav_client.calls[-1][1].set_result(handle)
        handle.result.set_result(SimpleNamespace(status=4))
        return handle

    def lift(self, state):
        self.node._tick()
        self.node._on_lift_state(Message(state))
        self.node._tick()

    def test_complete_four_rack_mission(self):
        self.qr("START")
        for label in "ABCD":
            self.assertEqual(self.node._state, State.NAV_TO_RACK)
            self.navigate()
            self.qr(f"RACK{label}_aB12")
            self.lift(1)
            self.assertEqual(self.node._state, State.NAV_TO_DESTINATION)
            self.navigate()
            self.qr("END")
            self.lift(0)
        self.assertEqual(self.node._state, State.COMPLETE)
        self.assertEqual(len(self.node._nav_client.calls), 8)
        self.assertEqual([m.data for m in self.node._lift_pub.messages], [1, 0] * 4)

    def test_docking_sequence_exits_before_counting_delivery(self):
        overrides = dict(Node.overrides, use_docking=True,
                         placement_x_m=[.22, .22, .57, .57],
                         placement_y_m=[.61, 1.39, 1.39, .61],
                         placement_yaw_rad=[math.pi]*4)
        with patch.object(Node, 'overrides', overrides):
            self.node = mission.DeliveryMission()
        def dock():
            self.node._tick()
            handle = GoalHandle()
            self.node._dock_client.calls[-1][1].set_result(handle)
            handle.result.set_result(SimpleNamespace(status=4))
        self.qr('START')
        for i, label in enumerate('ABCD'):
            self.navigate()
            self.qr(f'RACK{label}_aB12')
            self.assertEqual(self.node._state, State.DOCK_RACK)
            dock()
            self.lift(1)
            self.assertEqual(self.node._state, State.EXIT_RACK)
            dock()
            self.navigate()
            self.assertEqual(self.node._state, State.DOCK_DESTINATION)
            dock()
            self.qr('END')
            self.lift(0)
            self.assertEqual(self.node._state, State.EXIT_DESTINATION)
            self.assertEqual(self.node._rack_index, i)
            dock()
        self.assertEqual(self.node._state, State.COMPLETE)
        self.assertEqual(len(self.node._dock_client.calls), 16)

    def test_duplicate_docking_placements_rejected(self):
        with patch.object(Node, 'overrides', dict(Node.overrides, use_docking=True)):
            with self.assertRaises(ValueError):
                mission.DeliveryMission()

    def test_unconfirmed_coordinates_never_start(self):
        with patch.object(Node, "overrides", {}):
            node = mission.DeliveryMission()
        node._on_qr(Message("START"))
        node._tick()
        self.assertFalse(node._qr_scan_client.calls)
        self.assertFalse(node._nav_client.calls)

    def test_default_wait_for_start_has_no_precompetition_deadline(self):
        self.now += 1000.0
        self.qr("START")
        self.assertEqual(self.node._state, State.NAV_TO_RACK)
        self.assertEqual(self.node._competition_started_at, self.now)

    def test_configurable_start_timeout(self):
        self.node._start_timeout = 30.0
        self.now += 30.0
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)

    def test_wrong_and_late_valid_qr_cannot_bypass_timeout(self):
        for marker in ("END", "RACKA_aB12"):
            with self.subTest(marker=marker):
                node = self.node = mission.DeliveryMission()
                node._transition(State.WAIT_RACK_QR)
                node._tick()
                node._qr_scan_client.calls[-1][1].set_result(Trigger.Response(True))
                self.now += node._qr_timeout
                node._on_qr(Message(marker))
                node._tick()
                self.assertEqual(node._state, State.ERROR)

    def test_qr_suffix_is_case_sensitive(self):
        self.node._transition(State.WAIT_RACK_QR)
        self.qr("RACKA_AB12")
        self.assertEqual(self.node._state, State.WAIT_RACK_QR)
        self.now += 0.3
        self.qr("RACKA_aB12")
        self.assertEqual(self.node._state, State.LIFT_RACK)

    def test_unsolicited_qr_is_ignored(self):
        self.node._on_qr(Message("START"))
        self.assertEqual(self.node._qr_received, "")

    def test_qr_topic_can_arrive_before_start_service_response(self):
        self.node._tick()
        start = self.node._qr_scan_client.calls[-1][1]
        self.node._on_qr(Message("START"))
        self.node._tick()
        self.assertEqual(self.node._state, State.NAV_TO_RACK)
        self.assertTrue(start.cancelled())
        self.now += self.node._request_timeout + 0.1
        self.node._tick()
        self.assertEqual(self.node._state, State.NAV_TO_RACK)

    def test_timed_out_start_acknowledged_after_stop_requests_another_stop(self):
        self.node._tick()
        start = self.node._qr_scan_client.calls[-1][1]
        self.now += self.node._request_timeout
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)
        self.node._qr_stop_client.calls[-1][1].set_result(Trigger.Response(True))
        start.set_result(Trigger.Response(True))
        self.now += 0.6
        self.node._tick()
        self.assertEqual(len(self.node._qr_stop_client.calls), 2)

    def test_stop_timeout_and_refusal_retry(self):
        self.node.request_stop()
        first = self.node._qr_stop_client.calls[-1][1]
        self.now += self.node._request_timeout
        self.node._tick()
        self.assertTrue(first.cancelled())
        second = self.node._qr_stop_client.calls[-1][1]
        second.set_result(Trigger.Response(False, "busy"))
        self.now += 0.6
        self.node._tick()
        self.assertEqual(len(self.node._qr_stop_client.calls), 3)
        self.node._qr_stop_client.calls[-1][1].set_result(Trigger.Response(True))
        self.assertFalse(self.node.cleanup_pending())

    def test_stop_transport_failure_retries_without_crashing(self):
        self.node._qr_stop_client.error = RuntimeError("transport down")
        self.node.request_stop()
        self.assertTrue(self.node._scan_stop_pending)
        self.node._qr_stop_client.error = None
        self.now += 0.6
        self.node._tick()
        self.assertEqual(len(self.node._qr_stop_client.calls), 1)

    def test_lift_requires_command_then_new_feedback(self):
        self.node._on_lift_state(Message(1))
        self.node._transition(State.LIFT_RACK)
        self.node._tick()
        self.assertEqual(self.node._state, State.LIFT_RACK)
        self.assertEqual([m.data for m in self.node._lift_pub.messages], [1])
        self.now += 0.6
        self.node._tick()
        self.assertEqual(len(self.node._lift_pub.messages), 1)
        self.node._on_lift_state(Message(1))
        self.node._tick()
        self.assertEqual(self.node._state, State.NAV_TO_DESTINATION)

    def test_lift_waits_for_subscriber_before_publishing(self):
        self.node._transition(State.LIFT_RACK)
        self.node._lift_pub.subscribers = 0
        self.node._tick()
        self.assertFalse(self.node._lift_pub.messages)
        self.node._lift_pub.subscribers = 1
        self.node._tick()
        self.assertEqual(len(self.node._lift_pub.messages), 1)

    def test_late_lift_feedback_does_not_bypass_timeout(self):
        self.node._transition(State.LIFT_RACK)
        self.node._tick()
        self.now += self.node._lift_timeout
        self.node._on_lift_state(Message(1))
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)

    def test_competition_timeout_cancels_nav_without_lowering_lift(self):
        self.qr("START")
        self.node._tick()
        handle = GoalHandle()
        self.node._nav_client.calls[-1][1].set_result(handle)
        self.now += self.node._competition_time
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)
        self.assertEqual(len(handle.cancel.calls), 1)
        self.assertFalse(self.node._lift_pub.messages)

    def test_late_goal_acceptance_is_cancelled_without_replacing_new_goal(self):
        self.node._transition(State.NAV_TO_RACK)
        self.node._tick()
        old_request = self.node._nav_client.calls[-1][1]
        self.node._transition(State.NAV_TO_RACK)
        self.node._tick()
        current_handle = GoalHandle()
        self.node._nav_client.calls[-1][1].set_result(current_handle)
        stale_handle = GoalHandle()
        old_request.set_result(stale_handle)
        self.assertIs(self.node._goal_handle, current_handle)
        self.assertEqual(len(stale_handle.cancel.calls), 1)
        self.assertFalse(current_handle.cancel.calls)

    def test_result_from_previous_generation_is_ignored(self):
        self.node._transition(State.NAV_TO_RACK)
        self.node._tick()
        handle = GoalHandle()
        self.node._nav_client.calls[-1][1].set_result(handle)
        self.node._transition(State.NAV_TO_RACK)
        handle.result.set_result(SimpleNamespace(status=4))
        self.assertEqual(self.node._state, State.NAV_TO_RACK)

    def test_navigation_result_cannot_bypass_step_deadline(self):
        self.node._transition(State.NAV_TO_RACK)
        self.node._tick()
        handle = GoalHandle()
        self.node._nav_client.calls[-1][1].set_result(handle)
        self.now += self.node._navigation_timeout
        handle.result.set_result(SimpleNamespace(status=4))
        self.assertEqual(self.node._state, State.ERROR)

    def test_nav_rejection_and_aborted_result_stop_mission(self):
        for accepted in (False, True):
            with self.subTest(accepted=accepted):
                self.node = mission.DeliveryMission()
                self.node._transition(State.NAV_TO_RACK)
                self.node._tick()
                handle = GoalHandle(accepted=accepted)
                self.node._nav_client.calls[-1][1].set_result(handle)
                if accepted:
                    handle.result.set_result(SimpleNamespace(status=6))
                self.assertEqual(self.node._state, State.ERROR)

    def test_scan_start_refusal_and_transport_exception_stop_mission(self):
        self.node._tick()
        self.node._qr_scan_client.calls[-1][1].set_result(Trigger.Response(False, "error"))
        self.assertEqual(self.node._state, State.ERROR)
        self.node = mission.DeliveryMission()
        self.node._qr_scan_client.error = RuntimeError("transport down")
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)

    def test_late_goal_acceptance_after_failure_is_cancelled(self):
        self.node._transition(State.NAV_TO_RACK)
        self.node._tick()
        pending = self.node._nav_client.calls[-1][1]
        self.node.request_stop()
        handle = GoalHandle()
        pending.set_result(handle)
        self.assertEqual(len(handle.cancel.calls), 1)
        self.assertFalse(self.node._pending_goal_requests)

    def test_cancel_timeout_and_refusal_retry(self):
        handle = GoalHandle()
        self.node._queue_navigation_cancel(handle)
        first = handle.cancel.calls[-1][1]
        self.now += self.node._request_timeout
        self.node._tick()
        self.assertTrue(first.cancelled())
        self.now += 0.6
        self.node._tick()
        handle.cancel.calls[-1][1].set_result(SimpleNamespace(return_code=1))
        self.now += 0.6
        self.node._tick()
        handle.cancel.calls[-1][1].set_result(SimpleNamespace(return_code=3))
        self.assertFalse(self.node._cancel_handles)

    def test_navigation_send_error_stops_mission(self):
        self.node._transition(State.NAV_TO_RACK)
        self.node._nav_client.error = RuntimeError("transport down")
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)

    def test_result_request_error_cancels_accepted_goal(self):
        self.node._transition(State.NAV_TO_RACK)
        self.node._tick()
        handle = GoalHandle()
        handle.result_error = RuntimeError("transport down")
        self.node._nav_client.calls[-1][1].set_result(handle)
        self.assertEqual(self.node._state, State.ERROR)
        self.assertEqual(len(handle.cancel.calls), 1)

    def test_unavailable_nav_server_reaches_deadline(self):
        self.node._transition(State.NAV_TO_RACK)
        self.node._nav_client.ready = False
        self.node._tick()
        self.now += self.node._navigation_timeout
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)
        self.assertFalse(self.node._nav_client.calls)

    def test_lift_waits_for_envelope_acknowledgement(self):
        self.node._manage_envelope = True
        self.node._transition(State.LIFT_RACK)
        self.lift(1)
        self.assertEqual(self.node._state, State.LIFT_RACK)
        self.assertEqual(len(self.node._envelope_client.calls), 1)
        self.node._envelope_client.calls[0][1].set_result(Trigger.Response(True))
        self.node._tick()
        self.assertEqual(self.node._state, State.NAV_TO_DESTINATION)
        self.assertEqual(len(self.node._lift_pub.messages), 1)

    def test_rejected_envelope_stops_mission(self):
        self.node._manage_envelope = True
        self.node._transition(State.LIFT_RACK)
        self.lift(1)
        self.node._envelope_client.calls[0][1].set_result(Trigger.Response(False, 'rejected'))
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)
        self.assertFalse(self.node._nav_client.calls)

    def test_late_envelope_ack_cannot_bypass_timeout(self):
        self.node._manage_envelope = True
        self.node._transition(State.LIFT_RACK)
        self.lift(1)
        self.now += self.node._request_timeout
        self.node._envelope_client.calls[0][1].set_result(Trigger.Response(True))
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)

    def test_missing_envelope_service_is_bounded_by_lift_deadline(self):
        self.node._manage_envelope = True
        self.node._envelope_client.ready = False
        self.node._transition(State.LIFT_RACK)
        self.lift(1)
        self.now += self.node._lift_timeout
        self.node._tick()
        self.assertEqual(self.node._state, State.ERROR)

    def test_stop_cancels_envelope_wait(self):
        self.node._manage_envelope = True
        self.node._transition(State.LIFT_RACK)
        self.lift(1)
        future = self.node._envelope_future
        self.node.request_stop()
        self.assertTrue(future.cancelled())

    def test_invalid_timeout_and_pose_parameters_rejected(self):
        cases = [("qr_timeout_s", math.nan), ("lift_timeout_s", math.inf),
                 ("competition_time_s", 0.0), ("start_timeout_s", -1.0),
                 ("request_timeout_s", 0.0), ("destination_x_m", math.nan)]
        for name, value in cases:
            with self.subTest(name=name), patch.dict(Node.overrides, {name: value}):
                with self.assertRaises(ValueError):
                    mission.DeliveryMission()

    def test_invalid_full_rack_code_rejected(self):
        with patch.dict(Node.overrides, {"rack_qr_codes": ["RACKA", "", "", ""]}):
            with self.assertRaises(ValueError):
                mission.DeliveryMission()

    def test_main_stops_before_destroying_ros_context(self):
        events = []
        node = Mock()
        node.cleanup_pending.return_value = False
        node.request_stop.side_effect = lambda: events.append("stop")
        node.destroy_node.side_effect = lambda: events.append("destroy")
        ros = SimpleNamespace(
            init=Mock(), ok=lambda: True,
            spin_once=Mock(side_effect=KeyboardInterrupt),
            try_shutdown=lambda: events.append("shutdown"),
        )
        with patch.object(mission, "rclpy", ros), \
                patch.object(mission, "DeliveryMission", return_value=node), \
                patch.object(mission.signal, "signal", return_value="old") as signals:
            mission.main()
        self.assertEqual(events, ["stop", "destroy", "shutdown"])
        self.assertEqual(signals.call_count, 4)
        ros.init.assert_called_once_with(args=None, signal_handler_options=0)

    def test_main_handles_constructor_failure_without_unbound_node(self):
        ros = SimpleNamespace(init=Mock(), try_shutdown=Mock())
        with patch.object(mission, "rclpy", ros), \
                patch.object(mission, "DeliveryMission", side_effect=ValueError("bad config")), \
                patch.object(mission.signal, "signal"), \
                self.assertRaisesRegex(ValueError, "bad config"):
            mission.main()
        ros.try_shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
