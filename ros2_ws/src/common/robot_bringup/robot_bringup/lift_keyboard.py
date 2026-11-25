"""Terminal controls for the NAV lift adapter. Requires an interactive TTY."""
import curses
import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int8, String


TARGETS = {ord('9'): 1, ord('0'): 0}
ENDPOINTS = {1: '上位', 0: '下位'}


class LiftKeyboard(Node):
    def __init__(self):
        super().__init__('lift_keyboard')
        self.commands = self.create_publisher(Int8, '/lift_cmd', 10)
        # Only the latest adapter snapshot is relevant when an operator presses a key.
        self.create_subscription(String, '/ec_adapter/status', self.on_status, 1)
        self.status = {}
        self.received_at = None
        self.last_key = None
        self.notice = '等待按键'

    def on_status(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            status = None
        self.status = status if isinstance(status, dict) else {}
        self.received_at = time.monotonic()

    def availability(self):
        if self.received_at is None or time.monotonic() - self.received_at > 2:
            return '等待 EC 适配器状态'
        if self.status.get('lift_enabled') is not True:
            return '升降未启用'
        if self.status.get('pending_lift_id') is not None:
            return '正在执行升降，请等待反馈'
        if not self.commands.get_subscription_count():
            return '等待升降指令接收端'
        return ''

    def handle_key(self, key):
        if key == ord(' '):
            self.last_key = None
            self.notice = '可重新按 9 或 0'
        if key not in TARGETS or key == self.last_key:
            return
        # Terminals report key repeats but no release. Latch a direction until
        # the other direction or an explicit retry, even across a reconnect.
        self.last_key = key
        reason = self.availability()
        if reason:
            self.notice = '未发送：' + reason
            return
        target = TARGETS[key]
        self.commands.publish(Int8(data=target))
        self.notice = '已请求' + ('升起' if target else '降下')

    def run_terminal(self, screen):
        screen.keypad(True)
        screen.timeout(100)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            lines = (
                '托板升降    9 升起    0 降下    Q 退出',
                '同一方向需要重试时，先按空格，再按 9 或 0。',
                '连接：' + (self.availability() or '就绪'),
                '操作：' + self.notice,
                'EC 上次报告的位置：' + ENDPOINTS.get(self.status.get('reported_lift_target'), '未知'),
                '适配器反馈：' + (self.status.get('lift_result') or '暂无'),
            )
            screen.erase()
            try:
                screen.addstr(0, 0, '\n'.join(lines))
            except curses.error:
                # A small terminal can fill its bottom-right cell while drawing.
                pass
            screen.refresh()
            key = screen.getch()
            if key in (ord('q'), ord('Q'), 3):
                return
            if key in TARGETS:
                rclpy.spin_once(self, timeout_sec=0)
            self.handle_key(key)


def main(args=None):
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        sys.exit('升降键盘需要交互终端。通过 SSH 运行时请分配 TTY（ssh -t）。')
    rclpy.init(args=args)
    node = LiftKeyboard()
    try:
        curses.wrapper(node.run_terminal)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
