"""One terminal for manual chassis motion and the existing full-stroke lift."""
import curses
import json
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .lift_keyboard import LiftKeyboard


DRIVE_KEYS = {
    ord('i'): (.4, 0.), ord(','): (-.4, 0.),
    ord('j'): (0., 2.), ord('l'): (0., -2.),
}
KEY_TIMEOUT = .18


class RobotKeyboard(LiftKeyboard):
    def __init__(self):
        super().__init__('robot_keyboard')
        self.bridge = {}
        self.bridge_at = self.odom_at = self.key_at = -math.inf
        self.stationary_count = 0
        self.velocity = None
        self.command = Twist()
        self.armed = False
        self.arm_key_down = False
        self.arm_request = None
        self.arm_at = self.last_activity = 0.
        self.stopped_at = None
        self.stop_requests = []
        self.drive_notice = '按 A 手动使能'
        self.arm_service = self.create_client(SetBool, '/ec_bridge/set_arm')
        self.stop_service = self.create_client(Trigger, '/ec_bridge/stop')
        self.create_subscription(String, '/ec_bridge/status', self.on_bridge, 1)
        self.create_subscription(Odometry, '/ec/wheel/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(.05, self.drive_tick)

    def on_bridge(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            value = None
        self.bridge = value if isinstance(value, dict) else {}
        self.bridge_at = time.monotonic()
        if self.velocity is None or self.stopped_at is not None:
            return
        if not self.bridge.get('ec_armed'):
            if self.armed:
                self.halt('底盘已解除使能')
        elif not self.armed:
            self.key_at = -math.inf
            self.armed = True

    def on_odom(self, msg):
        t = msg.twist.twist
        x, y, yaw = t.linear.x, t.linear.y, t.angular.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if not all(math.isfinite(v) for v in (x, y, yaw)) or stamp <= 0 or not -.1 <= age <= .5:
            self.odom_at = -math.inf
            self.stationary_count = 0
            return
        self.odom_at = time.monotonic()
        speed = math.hypot(x, y)
        self.stationary_count = self.stationary_count + 1 if speed <= .01 and abs(yaw) <= .03 else 0
        if speed > .6 or abs(yaw) > 2.5:
            self.halt('里程计速度超过遥控上限')

    def drive_issue(self):
        now = time.monotonic()
        if self.received_at is None or now - self.received_at > 2 or now - self.bridge_at > 2:
            return '等待 EC 通信'
        if not self.status.get('motion_enabled'):
            return 'EC 适配器未开放行驶'
        if (not self.bridge.get('link_ok') or self.bridge.get('ec_estop') is not False
                or self.bridge.get('ec_fault') != 0):
            return 'EC 连接、急停或故障状态异常'
        if not self.status.get('telemetry_ready') or now - self.odom_at > .5:
            return '等待新鲜里程计和传感器反馈'
        if (self.count_publishers('/cmd_vel') != int(self.velocity is not None)
                or self.count_publishers('/ec/cmd_vel') != 1):
            return '速度话题发布者数量异常'
        return ''

    def arm_drive(self):
        if self.velocity is not None:
            return
        issue = self.drive_issue()
        if issue or not self.arm_service.service_is_ready() or not self.stop_service.service_is_ready():
            self.drive_notice = issue or '等待 EC 使能和停车服务'
            return
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 1)
        self.arm_at = self.last_activity = time.monotonic()
        self.arm_request = self.arm_service.call_async(SetBool.Request(data=True))
        self.drive_notice = '等待控制板确认使能'

    def handle_key(self, key):
        if key in (ord('a'), ord('A')):
            if not self.arm_key_down:
                self.arm_drive()
            self.arm_key_down = True
            return
        self.arm_key_down = False
        if key in DRIVE_KEYS:
            if self.armed and self.stopped_at is None:
                self.command = Twist()
                self.command.linear.x, self.command.angular.z = DRIVE_KEYS[key]
                self.key_at = self.last_activity = time.monotonic()
            return
        if key == curses.KEY_RESIZE:
            return
        self.key_at = -math.inf
        if self.velocity is not None and self.stopped_at is None:
            self.velocity.publish(Twist())
        super().handle_key(key)

    def drive_tick(self):
        if self.velocity is None or self.stopped_at is not None:
            return
        now = time.monotonic()
        issue = self.drive_issue()
        if self.arm_request.done() and not self.arm_request.result().success:
            issue = '控制板拒绝使能'
        elif not self.armed and now - self.arm_at > 3:
            issue = '使能反馈超时'
        elif now - self.last_activity > 300:
            issue = '行驶键已闲置五分钟'
        if issue:
            self.halt(issue)
            return
        self.velocity.publish(self.command if self.armed and now - self.key_at < KEY_TIMEOUT else Twist())
        if self.armed:
            self.drive_notice = '已使能'

    def halt(self, reason):
        if self.velocity is None or self.stopped_at is not None:
            return
        self.armed = False
        self.stopped_at = time.monotonic()
        self.stationary_count = 0
        self.velocity.publish(Twist())
        for client, request in ((self.stop_service, Trigger.Request()),
                                (self.arm_service, SetBool.Request(data=False))):
            if client.service_is_ready():
                self.stop_requests.append(client.call_async(request))
        self.drive_notice = reason + '；已请求停车、解除使能。重新打开遥控可恢复。'

    def finish_drive(self):
        if self.velocity is None:
            return
        self.halt('退出遥控')
        end = time.monotonic() + 2.
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
            if (self.bridge_at >= self.stopped_at and self.bridge.get('link_ok')
                    and self.bridge.get('ec_armed') is False and self.odom_at >= self.stopped_at
                    and self.stationary_count >= 2 and all(f.done() for f in self.stop_requests)):
                print('EC 已解除使能，里程计报告静止。')
                return
        print('已请求停车；未收到完整的解除使能和静止反馈。')

    def screen_lines(self):
        return (
            '底盘与托板遥控',
            'A 底盘使能    i 前进    , 后退    j 左转    l 右转    k 或空格停车',
            '底盘：' + (self.drive_issue() or self.drive_notice),
            '',
        ) + super().screen_lines()


def main(args=None):
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        sys.exit('请在交互终端运行。通过 SSH 运行可使用 ssh -t。')
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotKeyboard()

    def interrupt(*_):
        raise KeyboardInterrupt

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    try:
        curses.wrapper(node.run_terminal)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.finish_drive()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
