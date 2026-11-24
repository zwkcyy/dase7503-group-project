"""Blocking distance/angle commands for a sequential mission controller."""

import argparse
import math
import signal
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import BackUp, DriveOnHeading, Spin
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions


ACTIONS = {
    'forward': ('drive_on_heading', DriveOnHeading),
    'backward': ('backup', BackUp),
    'left': ('spin', Spin),
    'right': ('spin', Spin),
}


def make_goal(direction, amount, timeout_s):
    if direction not in ACTIONS or not math.isfinite(amount) or amount <= 0:
        raise ValueError('Choose forward/backward in metres or left/right in positive degrees.')
    if type(timeout_s) is not int or not 1 <= timeout_s <= 2147483647:
        raise ValueError('timeout_s must be a positive int32 number of seconds.')
    name, action = ACTIONS[direction]
    goal = action.Goal()
    goal.time_allowance.sec = timeout_s
    if action is Spin:
        goal.target_yaw = math.radians(amount if direction == 'left' else -amount)
    else:
        sign = 1.0 if direction == 'forward' else -1.0
        goal.target.x = sign * amount
        goal.speed = sign * 0.5
    return name, action, goal


def move(direction, amount, *, timeout_s=30, cancel=None, namespace=None):
    """Run one relative move to completion. The caller initializes rclpy.

    Use one sequential motion owner per robot. Supply a threading.Event to cancel.
    Raises on rejection, cancellation, timeout, obstacle, or any other Nav2 error.
    """
    name, action, goal = make_goal(direction, amount, timeout_s)
    node = rclpy.create_node('relative_motion_client', namespace=namespace)
    client = ActionClient(node, action, name)
    try:
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'{name} action server unavailable')
        if cancel is not None and cancel.is_set():
            raise RuntimeError('Cancelled before sending the goal')
        request = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, request, timeout_sec=5.0)
        if not request.done():
            raise RuntimeError(f'{name} acceptance unknown. Check the server before retrying.')
        handle = request.result()
        if not handle.accepted:
            raise RuntimeError(f'{name} goal rejected')
        result = handle.get_result_async()
        deadline = time.monotonic() + timeout_s + 2.0
        while not result.done() and time.monotonic() < deadline and rclpy.ok():
            if cancel is not None and cancel.is_set():
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if not result.done():
            cancellation = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancellation, timeout_sec=2.0)
            rclpy.spin_until_future_complete(node, result, timeout_sec=2.0)
        if not result.done():
            raise RuntimeError(f'{name} termination unconfirmed. Check the server before retrying.')
        response = result.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != 0:
            raise RuntimeError(f'{name} status={response.status}, '
                               f'error={response.result.error_code}: {response.result.error_msg}')
        return response.result
    finally:
        client.destroy()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction', choices=ACTIONS)
    parser.add_argument('amount', type=float, help='positive metres for translation, degrees for turns')
    parser.add_argument('--timeout', type=int, default=30, help='Nav2 action deadline in seconds')
    parser.add_argument('--namespace', help='robot namespace for relative action names')
    args = parser.parse_args()
    stop = threading.Event()
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    previous = {s: signal.signal(s, lambda *_: stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        move(args.direction, args.amount, timeout_s=args.timeout, cancel=stop, namespace=args.namespace)
        print(f'{args.direction} {args.amount:g} completed')
    except (ValueError, RuntimeError) as exc:
        parser.exit(1, f'{exc}\n')
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)
        rclpy.try_shutdown()
