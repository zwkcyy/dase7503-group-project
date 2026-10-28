"""Small ROS boundary fakes for offline tests; these do not simulate DDS."""

import importlib.util
from concurrent.futures import Future
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch


SRC = Path(__file__).resolve().parents[2]


class Message:
    def __init__(self, data=None):
        self.data = data


class Image:
    def __init__(self, frame=None, stamp_ns=0):
        self.frame = frame
        self.header = SimpleNamespace(
            frame_id="camera_optical_frame",
            stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000,
                                  nanosec=stamp_ns % 1_000_000_000),
        )


class PoseStamped:
    def __init__(self):
        self.header = SimpleNamespace(frame_id="", stamp=None)
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )


class Trigger:
    class Request:
        pass

    class Response:
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message


class Publisher:
    def __init__(self):
        self.messages = []
        self.subscribers = 1

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscribers


class Client:
    def __init__(self, *args):
        self.ready = True
        self.calls = []
        self.removed = []
        self.error = None

    def service_is_ready(self):
        return self.ready

    server_is_ready = service_is_ready

    def call_async(self, request):
        if self.error is not None:
            raise self.error
        future = Future()
        self.calls.append((request, future))
        return future

    send_goal_async = call_async

    def remove_pending_request(self, future):
        self.removed.append(future)


class GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result = Future()
        self.cancel = Client()
        self.result_error = None

    def get_result_async(self):
        if self.result_error is not None:
            raise self.result_error
        return self.result

    def cancel_goal_async(self):
        return self.cancel.call_async(None)


class Node:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.declare_parameter('use_sim_time', False)
        self.publishers = {}
        self.logger = Mock()
        self.now_ns = 10_000_000_000

    def declare_parameter(self, name, default):
        value = self.overrides.get(name, default)
        parameter = SimpleNamespace(value=value)
        parameter.get_parameter_value = lambda: SimpleNamespace(
            string_value=value, double_value=value, integer_value=value)
        self.parameters[name] = parameter

    def get_parameter(self, name):
        return self.parameters[name]

    def create_client(self, *args):
        return Client()

    def create_publisher(self, msg_type, topic, qos):
        publisher = Publisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        return SimpleNamespace(callback=callback, qos=qos)

    def create_service(self, srv_type, topic, callback):
        return callback

    def create_timer(self, interval, callback):
        return callback

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=self.now_ns,
            to_msg=lambda: Image(stamp_ns=self.now_ns).header.stamp))


class Bridge:
    def imgmsg_to_cv2(self, message, desired_encoding):
        return message.frame

    def cv2_to_imgmsg(self, frame, encoding):
        return Image(frame)


class ExternalShutdownException(Exception):
    pass


def load_module(name, relative_path, with_ros=False):
    """Import production source while replacing only ROS transport boundaries."""
    modules = {}

    def add(module_name, **attributes):
        module = ModuleType(module_name)
        module.__dict__.update(attributes)
        modules[module_name] = module

    if with_ros:
        add("rclpy")
        add("rclpy.node", Node=Node)
        add("rclpy.action", ActionClient=Client)
        add("rclpy.signals", SignalHandlerOptions=SimpleNamespace(NO=0))
        add("rclpy.executors", ExternalShutdownException=ExternalShutdownException)
        add("rclpy.qos", QoSProfile=SimpleNamespace,
            ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=2))
        for package in ("std_msgs", "sensor_msgs", "std_srvs", "nav2_msgs",
                        "geometry_msgs", "action_msgs"):
            add(package)
        add("std_msgs.msg", String=Message, Int8=Message, Bool=Message)
        add("sensor_msgs.msg", Image=Image)
        add("std_srvs.srv", Trigger=Trigger, SetBool=Trigger)
        add("geometry_msgs.msg", PoseStamped=PoseStamped)
        add("nav2_msgs.action", NavigateToPose=SimpleNamespace(Goal=SimpleNamespace))
        add("action_msgs.msg", GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4))
        add("cv_bridge", CvBridge=Bridge)

    spec = importlib.util.spec_from_file_location(name, SRC / relative_path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", modules):
        spec.loader.exec_module(module)
    return module
