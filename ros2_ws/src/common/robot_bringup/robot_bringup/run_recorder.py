"""Persist received mission states and confirmations; never publish control commands."""
from datetime import datetime
import json
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class RunRecorder(Node):
    def __init__(self):
        super().__init__('run_recorder')
        self.declare_parameter('output_dir', '~/.ros/nav_vi_runs/record-'+uuid.uuid4().hex[:12])
        self.declare_parameter('source', 'unspecified')
        self.path = Path(self.get_parameter('output_dir').value).expanduser()
        self.path.mkdir(parents=True, exist_ok=True)
        self.source = self.get_parameter('source').value
        self.events = self.path.joinpath('events.jsonl').open('x', encoding='utf-8')
        self.status, self.codes, self.started, self.finished = None, [], None, None
        self.create_subscription(String, '/mission/status', self.on_state, 10)
        self.create_subscription(String, '/qr/data', self.on_qr, 10)
        self.create_subscription(String, '/ec_adapter/status', self.on_ec, 10)
        self.create_timer(2., self.summary)
        self.record('recorder_started', source=self.source)

    def record(self, kind, **data):
        self.events.write(json.dumps({'event': kind, 'received_at': datetime.now().astimezone().isoformat(),
                                     **data}, ensure_ascii=False)+'\n')
        self.events.flush()

    def on_qr(self, msg):
        self.codes.append(msg.data)
        if msg.data == 'START' and self.started is None:
            self.started = time.monotonic()
        self.record('qr_confirmed', payload=msg.data)
        self.summary()

    def on_state(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            self.record('invalid_status', raw=msg.data)
            return
        if not isinstance(status, dict):
            self.record('invalid_status', raw=msg.data)
            return
        if status != self.status:
            self.status = status
            if status.get('state') in ('COMPLETE', 'ERROR') and self.finished is None:
                self.finished = time.monotonic()
            self.record('mission_state', status=status)
            self.summary()

    def on_ec(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            self.record('invalid_ec_status', raw=msg.data)
            return
        self.record('ec_adapter_status', status=status)

    def summary(self):
        data = {'source': self.source, 'status': self.status, 'confirmed_codes': self.codes,
                'elapsed_after_start_s': round((self.finished or time.monotonic())-self.started, 3)
                if self.started else None,
                'timing_basis': 'observer receipt time, not official competition timing',
                'physical_acceptance': False}
        temporary = self.path / 'summary.json.tmp'
        temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')
        temporary.replace(self.path / 'summary.json')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RunRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.record('recorder_stopped')
            node.summary()
            node.events.close()
            node.destroy_node()
        rclpy.try_shutdown()
