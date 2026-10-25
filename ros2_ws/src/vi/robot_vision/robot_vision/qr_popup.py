"""Display confirmed scanner results in a persistent, optional desktop window.

The ROS worker only subscribes. Closing this process does not stop the scanner
or mission. Adapted from vision/qr-popup (60a40e3).
"""

import math
import signal
import sys
import threading
import time

from .qr_result_state import ViewerState


def run_ros_viewer(state, stop, args=None):
    """Receive data in a worker thread; never access Tk widgets here."""
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from cv_bridge import CvBridge
        from PIL import Image as PILImage
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
    except ImportError as exc:
        state.set_status(
            "无法连接：请安装 ROS、cv_bridge、Pillow 并加载 ROS 环境。 / "
            f"Missing viewer dependency: {exc}"
        )
        return

    context = Context()
    node = executor = None
    try:
        rclpy.init(args=args, context=context,
                   signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("qr_popup", context=context)
        node.declare_parameter("image_topic", "/camera/image_raw")
        node.declare_parameter("debug_image_topic", "/qr/debug_image")
        node.declare_parameter("result_topic", "/qr/data")
        node.declare_parameter("preview_hz", 10.0)
        preview_hz = float(node.get_parameter("preview_hz").value)
        if not math.isfinite(preview_hz) or not 0.0 < preview_hz <= 30.0:
            raise ValueError("preview_hz must be finite and in (0, 30]")

        bridge = CvBridge()
        last_frame = {False: -math.inf, True: -math.inf}
        last_error_log = -math.inf

        def receive_image(message, annotated=False):
            nonlocal last_error_log
            now = time.monotonic()
            if now - last_frame[annotated] < 1.0 / preview_hz:
                return
            last_frame[annotated] = now
            try:
                rgb = bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
                image = PILImage.fromarray(rgb)
                # Bound memory and GUI work even if the camera publishes large images.
                image.thumbnail((960, 480))
                state.receive_frame(image, annotated=annotated)
            except Exception as exc:
                # A broken preview must not prevent /qr/data callbacks from running.
                state.set_preview_error("画面无法显示，扫码结果仍会保留 / Preview unavailable")
                if now - last_error_log >= 2.0:
                    node.get_logger().warning(f"Preview conversion failed: {exc}")
                    last_error_log = now

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(
            Image, node.get_parameter("image_topic").value,
            receive_image, image_qos,
        )
        node.create_subscription(
            Image, node.get_parameter("debug_image_topic").value,
            lambda message: receive_image(message, annotated=True), image_qos,
        )
        node.create_subscription(
            String, node.get_parameter("result_topic").value,
            lambda message: state.receive_result(message.data), 10,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        state.set_status("")
        node.get_logger().info("QR result window connected; waiting for confirmed results")
        while not stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=0.1)
        if not stop.is_set():
            state.set_status("连接已停止，保留上次结果 / Connection stopped; keeping result")
    except ExternalShutdownException:
        if not stop.is_set():
            state.set_status("连接已停止，保留上次结果 / Connection stopped; keeping result")
    except Exception as exc:
        state.set_status(f"连接失败，保留上次结果 / Connection failed: {exc}")
    finally:
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                rclpy.try_shutdown(context=context)


class QRPopup:
    """Render on the main thread while ROS runs in a separate worker."""

    def __init__(self, root, tk, ttk, image_tk, stop, args=None):
        self.root, self.tk, self.image_tk = root, tk, image_tk
        self.stop = stop
        self.state = ViewerState()
        self.shown_result = None
        self.shown_frame = None
        self.preview_size = None
        self.display_error = ""
        self.photo = None
        self.closed = False
        self.after_id = None

        root.title("Robot Vision | QR Results")
        root.geometry("860x780")
        root.minsize(680, 680)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda event: self.close())
        panel = ttk.Frame(root, padding=20)
        panel.pack(fill="both", expand=True)
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(2, weight=1)
        ttk.Label(panel, text="扫码结果 / QR Results",
                  font=("TkDefaultFont", 20, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(
            panel,
            text="结果保留到下一个不同的二维码被确认 / Results stay until a different code is confirmed",
            wraplength=640,
        ).grid(row=1, column=0, sticky="w", pady=(6, 12))
        self.video = ttk.Label(panel, anchor="center", text="等待相机画面 / Waiting for camera")
        self.video.grid(row=2, column=0, sticky="nsew")
        self.preview_status = tk.StringVar(value="")
        ttk.Label(panel, textvariable=self.preview_status, wraplength=640).grid(
            row=3, column=0, sticky="w")
        self.meaning = tk.StringVar(value="等待第一个确认结果 / Waiting for the first result")
        ttk.Label(panel, textvariable=self.meaning, font=("TkDefaultFont", 14, "bold"),
                  wraplength=640).grid(row=4, column=0, sticky="w", pady=(14, 8))

        text_panel = ttk.Frame(panel)
        text_panel.grid(row=5, column=0, sticky="ew")
        self.payload = tk.Text(text_panel, height=4, wrap="word", font=("TkDefaultFont", 16))
        scroll = ttk.Scrollbar(text_panel, orient="vertical", command=self.payload.yview)
        self.payload.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.payload.pack(side="left", fill="both", expand=True)
        self.set_payload("尚未收到扫码结果 / No confirmed result received")
        self.timestamp = tk.StringVar(value="")
        ttk.Label(panel, textvariable=self.timestamp).grid(
            row=6, column=0, sticky="w", pady=(8, 4))
        self.status = tk.StringVar(value="正在连接 / Connecting...")
        ttk.Label(panel, textvariable=self.status, wraplength=640).grid(
            row=7, column=0, sticky="w")
        buttons = ttk.Frame(panel)
        buttons.grid(row=8, column=0, sticky="ew", pady=(12, 0))
        self.copy_button = ttk.Button(buttons, text="复制结果 / Copy", command=self.copy_result,
                                      state="disabled")
        self.copy_button.pack(side="left")
        ttk.Button(buttons, text="关闭窗口 / Close window", command=self.close).pack(side="right")

        self.worker = threading.Thread(
            target=run_ros_viewer, args=(self.state, stop, args),
            name="qr-result-viewer", daemon=True,
        )
        self.worker.start()
        self.poll()

    def set_payload(self, text):
        self.payload.configure(state="normal")
        self.payload.delete("1.0", "end")
        self.payload.insert("1.0", text)
        self.payload.configure(state="disabled")

    def poll(self):
        if self.closed:
            return
        if self.stop.is_set():
            self.close()
            return
        snapshot = self.state.snapshot()
        if snapshot.result is not None and snapshot.result != self.shown_result:
            self.shown_result = snapshot.result
            self.set_payload(snapshot.result.payload)
            self.meaning.set(snapshot.result.description)
            # /qr/data has no timestamp; do not present receipt time as capture time.
            self.timestamp.set(f"收到时间 / Received at: {snapshot.result.received_at}")
            self.copy_button.configure(state="normal")

        frame = snapshot.frame
        size = (self.video.winfo_width(), self.video.winfo_height())
        if (frame is not None and min(size) > 1
                and (frame.sequence != self.shown_frame or size != self.preview_size)):
            self.shown_frame = frame.sequence
            self.preview_size = size
            try:
                image = frame.image.copy()
                image.thumbnail(size)
                photo = self.image_tk.PhotoImage(image, master=self.root)
                self.video.configure(image=photo, text="")
                self.photo = photo
                self.display_error = ""
            except Exception:
                self.display_error = "画面无法显示，扫码结果仍会保留 / Preview unavailable"
        preview_error = snapshot.preview_error or self.display_error
        if preview_error:
            self.preview_status.set(preview_error)
        elif frame is None:
            self.preview_status.set("等待相机画面 / Waiting for camera")
        elif time.monotonic() - frame.received_at > 3.0:
            self.preview_status.set("画面已暂停，保留上次结果 / Image stream paused; keeping result")
        else:
            self.preview_status.set("相机画面 / Camera preview")
        self.status.set(snapshot.status or (
            "确认结果已保留，等待下一次扫描 / Showing last result; waiting for the next scan"
            if self.shown_result else "等待扫码确认 / Waiting for a confirmed QR code"
        ))
        self.after_id = self.root.after(50, self.poll)

    def copy_result(self):
        if self.shown_result is not None:
            try:
                self.root.clipboard_clear()
                self.root.clipboard_append(self.shown_result.payload)
            except self.tk.TclError:
                self.status.set("无法访问剪贴板，请直接选中文字复制 / Clipboard unavailable")

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        self.root.withdraw()
        self.worker.join(timeout=1.5)
        self.root.destroy()


def main(args=None):
    # Keep desktop dependencies isolated from the headless scanner executable.
    try:
        import tkinter as tk
        from tkinter import ttk
        from PIL import ImageTk
    except ImportError as exc:
        raise SystemExit(
            "扫码窗口需要 python3-tk、python3-pil 和 python3-pil.imagetk。 / "
            "Install the desktop viewer dependencies."
        ) from exc

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit(
            "无法创建扫码窗口，请在图形桌面或远程桌面中运行。 / "
            "A graphical desktop is required for the result window."
        ) from exc

    stop = threading.Event()
    previous_handlers = {}
    app = None
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda *_: stop.set())
        app = QRPopup(root, tk, ttk, ImageTk, stop, args)
        if not app.closed:
            root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if app is not None:
            app.close()
        else:
            root.destroy()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main(sys.argv)
