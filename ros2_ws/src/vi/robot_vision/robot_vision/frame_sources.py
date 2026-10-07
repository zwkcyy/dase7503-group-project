"""Frame sources: ROS images from Camera Module 3, local cameras, and images."""

import time

import cv2


def run_source(args, stop, on_frame):
    """Read frames in a worker thread; on_frame must not access Tk widgets."""
    if args.source == "ros":
        run_ros(args, stop, on_frame)
    elif args.source == "opencv":
        run_opencv(args, stop, on_frame)
    else:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise RuntimeError(f"无法读取图片: {args.image}")
        while not stop.is_set():
            on_frame(frame.copy())
            stop.wait(1 / args.fps)


def run_opencv(args, stop, on_frame):
    # Use this mode for USB or built-in cameras; use camera_ros for CSI cameras.
    capture = cv2.VideoCapture(args.camera)
    try:
        if not capture.isOpened():
            raise RuntimeError(f"无法打开摄像头 {args.camera}, 请检查权限和设备编号")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        while not stop.is_set():
            started = time.monotonic()
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("摄像头读取失败, 请检查连接并重启程序")
            on_frame(frame)
            stop.wait(max(0, 1 / args.fps - (time.monotonic() - started)))
    finally:
        capture.release()


def run_ros(args, stop, on_frame):
    # Import lazily so image and USB camera modes can run without ROS 2.
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise RuntimeError("ROS 模式需要 rclpy, cv_bridge 和 sensor_msgs; 请先 source ROS 环境") from exc

    # A separate context lets shutdown release only this application's ROS resources.
    context = Context()
    node = executor = None
    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node("qr_popup", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        bridge = CvBridge()
        last_frame_at = 0.0

        def receive(msg):
            nonlocal last_frame_at
            now = time.monotonic()
            if now - last_frame_at < 1 / args.fps:
                return
            last_frame_at = now
            # Request BGR explicitly instead of assuming the camera's pixel encoding.
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            on_frame(frame)

        # Keep only the latest frame; BEST_EFFORT supports common camera QoS settings.
        subscription = node.create_subscription(
            Image, args.topic, receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        while not stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
