"""Scan QR codes and retain the latest result in a single non-blocking window."""

import argparse
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk

from .frame_sources import run_source
from .qr_core import QRDecoder, ResultLatch


@dataclass
class FrameUpdate:
    frame: object
    result: object
    detected: bool


def replace_latest(queue, value):
    """Keep only the latest frame when the GUI cannot keep up."""
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        queue.put_nowait(value)


class QRPopup:
    def __init__(self, root, args, source_runner=run_source):
        self.root, self.args = root, args
        self.stop = threading.Event()
        self.frames = Queue(maxsize=1)
        self.errors = Queue(maxsize=1)
        self.shown_result = None
        self.last_frame_at = time.monotonic()
        self.photo = None
        self.closed = False
        self.after_id = None

        root.title("Robot Vision | QR Scanner")
        root.geometry("860x780")
        root.minsize(680, 680)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda event: self.close())
        panel = ttk.Frame(root, padding=20)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="二维码扫描 / QR Scanner",
                  font=("TkDefaultFont", 20, "bold")).pack(anchor="w")
        ttk.Label(panel, text="识别结果保留到下一个二维码出现 / Last result stays until a new code is scanned").pack(anchor="w", pady=(6, 12))
        self.video = ttk.Label(panel, anchor="center", text="等待图像 / Waiting for camera image")
        self.video.pack(fill="both", expand=True)
        self.meaning = tk.StringVar(value="等待第一个二维码 / Waiting for the first QR code")
        ttk.Label(panel, textvariable=self.meaning, font=("TkDefaultFont", 14, "bold"),
                  wraplength=620).pack(anchor="w", pady=(14, 8))
        # Text supports Unicode, multiline payloads, scrolling, and copying.
        # A persistent widget lets scanning continue without a modal message box.
        text_panel = ttk.Frame(panel)
        text_panel.pack(fill="x")
        self.payload = tk.Text(text_panel, height=4, wrap="word", font=("TkDefaultFont", 16))
        scroll = ttk.Scrollbar(text_panel, orient="vertical", command=self.payload.yview)
        self.payload.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.payload.pack(side="left", fill="both", expand=True)
        self.set_payload("尚未扫描 / No QR code scanned")
        self.timestamp = tk.StringVar(value="")
        ttk.Label(panel, textvariable=self.timestamp).pack(anchor="w", pady=(8, 4))
        self.status = tk.StringVar(value=f"输入 / Source: {args.topic if args.source == 'ros' else args.source}")
        ttk.Label(panel, textvariable=self.status, wraplength=620).pack(anchor="w")
        buttons = ttk.Frame(panel)
        buttons.pack(fill="x", pady=(12, 0))
        ttk.Button(buttons, text="复制结果 / Copy", command=self.copy_result).pack(side="left")
        ttk.Button(buttons, text="退出 / Exit", command=self.close).pack(side="right")

        def worker():
            try:
                decoder = QRDecoder()
                latch = ResultLatch()

                def process(frame):
                    payload, corners = decoder.decode(frame)
                    latch.update(payload)
                    if corners is not None:
                        cv2.polylines(frame, [corners.astype("int32")], True, (0, 200, 0), 2)
                    # Include the retained result in every frame update so GUI frame
                    # dropping cannot discard the latest decoded payload.
                    replace_latest(self.frames, FrameUpdate(frame, latch.current, bool(payload)))

                source_runner(args, self.stop, process)
            except Exception as exc:
                replace_latest(self.errors, f"{type(exc).__name__}: {exc}")

        self.worker = threading.Thread(target=worker, name="qr-camera", daemon=True)
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
        try:
            update = self.frames.get_nowait()
        except Empty:
            update = None
        if update:
            self.last_frame_at = time.monotonic()
            if update.result and update.result != self.shown_result:
                self.shown_result = update.result
                self.set_payload(update.result.payload)
                self.meaning.set(update.result.description)
                self.timestamp.set(f"扫描时间 / Scanned at: {update.result.scanned_at}")
            self.status.set("已识别 / QR detected" if update.detected else
                            "未检测到二维码, 保留上次结果 / No code visible; keeping last result")
            rgb = cv2.cvtColor(update.frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            image.thumbnail((max(300, self.video.winfo_width()), 390))
            self.photo = ImageTk.PhotoImage(image)
            self.video.configure(image=self.photo, text="")
        elif time.monotonic() - self.last_frame_at > 3 and self.worker.is_alive():
            self.status.set("未收到图像, 保留上次结果. 请检查相机或 ROS 话题 / Waiting for image")
        try:
            error = self.errors.get_nowait()
        except Empty:
            error = None
        if error:
            self.status.set(f"扫描已停止, 保留上次结果 / Scanner stopped: {error}")
        self.after_id = self.root.after(50, self.poll)

    def copy_result(self):
        if self.shown_result:
            self.root.clipboard_clear()
            self.root.clipboard_append(self.shown_result.payload)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        self.worker.join(timeout=1.5)
        self.root.destroy()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("ros", "opencv", "image"), default="ros")
    parser.add_argument("--topic", default="/camera/image_raw", help="sensor_msgs/msg/Image 话题")
    parser.add_argument("--camera", type=int, default=0, help="本地 / USB 摄像头编号")
    parser.add_argument("--image", type=Path, help="图片测试模式的输入文件")
    parser.add_argument("--fps", type=float, default=10, help="最大处理帧率")
    parser.add_argument("--width", type=int, default=1280, help="本地摄像头请求宽度")
    parser.add_argument("--height", type=int, default=720, help="本地摄像头请求高度")
    args = parser.parse_args(argv)
    if not 0 < args.fps <= 60:
        parser.error("--fps 必须大于 0 且不超过 60")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width 和 --height 必须大于 0")
    if args.source == "image" and (args.image is None or not args.image.is_file()):
        parser.error("图片模式需要 --image 指向存在的文件")
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit("无法创建窗口. 请在桌面会话中运行, 或使用带显示转发的远程桌面.") from exc
    app = QRPopup(root, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.close()


if __name__ == "__main__":
    main()
