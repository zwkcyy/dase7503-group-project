"""Explicit ROS mocks for task sequencing; never a claim of physical success."""

import json
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger, SetBool

from .planar_base import require_isolated_domain


class MockPeers(Node):
    def __init__(self):
        super().__init__('mock_peers')
        require_isolated_domain()
        self.declare_parameter('mock_navigation', True)
        self.declare_parameter('mock_envelope', True)
        self.declare_parameter('mock_qr', True)
        self.declare_parameter('scenario', 'success')
        self.scenario = self.get_parameter('scenario').value
        allowed = {'success', 'wrong_qr', 'missing_qr', 'lift_timeout', 'navigation_abort',
                   'navigation_reject', 'navigation_timeout', 'envelope_failure'}
        if self.scenario not in allowed:
            raise ValueError(f'scenario must be one of {sorted(allowed)}')
        self.state = 'WAIT_START'
        self.index = 0
        self.scan_due = None
        self.lift_pending = None
        self.goals = []
        self.nav_ready = self.get_parameter('mock_navigation').value
        self.nav_state_future = None
        self.nav_state_client = None if self.nav_ready else self.create_client(GetState, '/bt_navigator/get_state')
        self.group = ReentrantCallbackGroup()
        self.qr_pub = (self.create_publisher(String, '/qr/data', 10)
                       if self.get_parameter('mock_qr').value else None)
        self.lift_pub = self.create_publisher(Int8, '/lift_state', 10)
        self.events = self.create_publisher(String, '/simulation/events',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/mission/status', self.status, 10)
        self.create_subscription(Int8, '/lift_cmd', self.lift, 10)
        if self.qr_pub is not None:
            self.create_service(Trigger, '/qr/start_scan', self.start_scan)
            self.create_service(Trigger, '/qr/stop_scan', self.stop_scan)
        if self.get_parameter('mock_envelope').value:
            self.create_service(SetBool, '/nav/set_loaded', self.envelope)
        self.action = None
        if self.get_parameter('mock_navigation').value:
            self.action = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute,
                goal_callback=lambda goal: GoalResponse.REJECT if self.scenario == 'navigation_reject' else GoalResponse.ACCEPT,
                cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=self.group)
        self.create_timer(.05, self.tick, callback_group=self.group)
        self.get_logger().warning(f'MOCK LIFT: scenario={self.scenario}; mock_qr={self.qr_pub is not None}')

    def event(self, name, **fields):
        text = json.dumps({'event': name, **fields})
        self.events.publish(String(data=text))
        self.get_logger().info(text)

    def status(self, msg):
        status = json.loads(msg.data)
        self.state, self.index = status['state'], status['rack_index']

    def start_scan(self, request, response):
        if self.scan_due is None:
            self.scan_due = time.monotonic() + .6
        response.success = True
        response.message = 'Mock scanner started'
        return response

    def stop_scan(self, request, response):
        self.scan_due = None
        self.event('scan_stopped')
        response.success = True
        return response

    def lift(self, msg):
        if msg.data not in (0, 1):
            return
        self.event('lift_command', command=msg.data)
        if self.scenario != 'lift_timeout':
            self.lift_pending = (time.monotonic() + .3, msg.data)

    def envelope(self, request, response):
        response.success = self.scenario != 'envelope_failure'
        response.message = 'Mock envelope acknowledgement' if response.success else 'Injected envelope rejection'
        self.event('envelope', loaded=request.data, success=response.success)
        return response

    async def execute(self, handle):
        future = Future()
        self.goals.append((handle, future, time.monotonic() + .4))
        self.event('navigation_goal')
        await future
        return NavigateToPose.Result()

    def tick(self):
        now = time.monotonic()
        if not self.nav_ready and self.nav_state_client.service_is_ready():
            if self.nav_state_future is None:
                self.nav_state_future = self.nav_state_client.call_async(GetState.Request())
            elif self.nav_state_future.done():
                self.nav_ready = self.nav_state_future.result().current_state.id == 3
                self.nav_state_future = None
        for handle, future, due in list(self.goals):
            if handle.is_cancel_requested:
                handle.canceled()
                self.event('navigation_cancelled')
            elif now >= due and self.scenario != 'navigation_timeout':
                if self.scenario == 'navigation_abort':
                    handle.abort()
                else:
                    handle.succeed()
            else:
                continue
            future.set_result(None)
            self.goals.remove((handle, future, due))
        if self.lift_pending and now >= self.lift_pending[0]:
            self.lift_pub.publish(Int8(data=self.lift_pending[1]))
            self.lift_pending = None
        if self.scan_due is not None and now >= self.scan_due:
            if self.state == 'WAIT_START' and not self.nav_ready:
                return
            if self.scenario == 'missing_qr':
                return
            payload = {'WAIT_START': 'START', 'VERIFY_DESTINATION': 'END',
                       'WAIT_RACK_QR': f'RACK{"ABCD"[min(self.index, 3)]}_DEMO'}.get(self.state)
            if payload is None:
                return
            if self.scenario == 'wrong_qr' and self.state != 'WAIT_START':
                payload = 'RACKA_WRONG'
            self.qr_pub.publish(String(data=payload))
            self.event('mock_qr', payload=payload)
            self.scan_due = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockPeers()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            if node.action:
                node.action.destroy()
            node.destroy_node()
        rclpy.try_shutdown()
