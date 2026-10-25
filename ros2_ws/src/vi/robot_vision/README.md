# Robot Vision - DASE7503

ROS 2 on-demand QR code scanner for Raspberry Pi 5, with an optional result window.

## Platform

- Raspberry Pi 5
- Raspberry Pi Camera Module 3 (IMX708)
- Ubuntu 24.04
- ROS 2 Jazzy

## QR Recognition

- Primary: OpenCV WeChatQRCode (CNN + Super Resolution)
- Fallback: OpenCV QRCodeDetector (also works when WeChat models are absent)
- Two-hit QR confirmation

The WeChat detector is enabled when all four downloaded model files are present
and the installed OpenCV build includes WeChatQRCode. Otherwise the node logs a
warning and continues with the OpenCV fallback.

Supported QR codes: START, END, RACKA_XXXX, RACKB_XXXX, RACKC_XXXX, RACKD_XXXX.

## ROS Interface

Package: `robot_vision`

Executable: `qr_detector_node`

Node: `/qr_detector`

Camera input:
- `/camera/image_raw` (sensor_msgs/msg/Image)

Start scanning:
- `/qr/start_scan` (std_srvs/srv/Trigger)

Stop scanning:
- `/qr/stop_scan` (std_srvs/srv/Trigger)

Scan result:
- `/qr/data` (std_msgs/msg/String)

Detection status:
- `/qr/detected` (std_msgs/msg/Bool)

Debug image:
- `/qr/debug_image` (sensor_msgs/msg/Image)

## Scanning Behaviour

1. Scanner starts in IDLE.
2. Navigation reaches the target position.
3. Navigation calls `/qr/start_scan`.
4. Vision starts QR detection.
5. A valid QR code is confirmed.
6. Vision publishes `/qr/data` exactly once.
7. Scanner automatically returns to IDLE.

No timeout is implemented in the vision module.
Navigation or mission management handles timeout and retry.

Repeated start requests succeed without resetting an active scan. Stop requests
succeed even when already idle and clear confirmation history. A new scan starts
with fresh history. Confirmation needs the same text on separate frames within
`confirm_window` (default 0.8 s); the default is two hits at up to 10 Hz.

The subscriber keeps only the latest image. Frames timestamped before the scan,
and repeated or out-of-order nonzero timestamps, are ignored. The camera and
scanner must use the same ROS clock; set `use_sim_time` consistently for replay.
Sources using a zero timestamp remain supported, but cannot benefit from the
timestamp freshness check. Time windows and throttling use a monotonic wall clock.

CSV startup/write errors are logged without stopping QR delivery. Results are
published before writing to disk. Debug images are drawn only when a subscriber
is connected, and a debug conversion failure cannot suppress a confirmed result.

## Install Models

Run:

    bash scripts/download_qr_models.sh

Models are stored in:

    ~/.local/share/robot_vision/wechat_qrcode/

## Build

    source /opt/ros/jazzy/setup.bash
    source ~/camera_ws/install/setup.bash
    cd ~/dase7503-group-project/ros2_ws
    colcon build --symlink-install --packages-select robot_vision
    source install/setup.bash

## Run

Start the camera and QR scanner together:

    ros2 launch robot_vision vision_bringup.launch.py

If `camera_ros` is already running, leave it up and launch only the scanner:

    ros2 run robot_vision qr_detector_node

The launch file accepts `start_camera:=false` and `image_topic:=/your/image/topic`
for an already running camera or a camera driver with a different image topic.
It also accepts `use_sim_time:=true` and
`camera_frame:=camera_optical_frame`. The default frame matches the optical frame
in `robot_description`; the camera driver's private image topic is remapped to
`image_topic`.

## Persistent result window

The result window from `vision/qr-popup` is integrated as the optional `qr_popup`
executable. It displays confirmed `/qr/data` results from the current scanner,
with the full original text, its meaning, local receipt time, a copy button and
a camera preview. The payload keeps its original case. A repeated payload keeps
its displayed time; a different payload replaces it. Removing the QR, finishing
a scan or losing the camera stream leaves the last result visible.

Install the desktop dependencies on Ubuntu 24.04, then rebuild `robot_vision`
using the build commands above:

```bash
sudo apt install python3-tk python3-pil python3-pil.imagetk
```

The package declares the corresponding rosdep keys, including
`python3-pil-imagetk` (Ubuntu package `python3-pil.imagetk`). See the
[official rosdep dependency mapping](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml).

Start the camera, scanner and window together in a graphical desktop session:

```bash
ros2 launch robot_vision vision_bringup.launch.py show_results:=true
```

If the scanner and camera are already running, open only the window:

```bash
ros2 run robot_vision qr_popup
# For an existing camera with a different topic:
ros2 run robot_vision qr_popup --ros-args -p image_topic:=/your/image/topic
```

The launch defaults to `show_results:=false`, suitable for headless operation.
Use a monitor or remote desktop for the window; a plain SSH shell without a
graphical display cannot show it. Closing the window only stops the viewer.

The viewer subscribes to `/qr/data`, `/camera/image_raw` and `/qr/debug_image`.
During scanning it briefly prefers the detector's annotated images, then returns
to the live camera feed. It does not decode images, publish commands or invoke
scan services. The mission continues to control `/qr/start_scan` and
`/qr/stop_scan`. For a standalone scan with the mission stopped, use the service
command in the next section. Opening a window alone does not start a scan.

Viewer parameters are `image_topic`, `debug_image_topic`, `result_topic` and
`preview_hz` (default 10.0, valid range greater than 0 and at most 30). The launch
also accepts `preview_hz:=5.0` to reduce preview work. Receipt time is the window's
local wall-clock time: the existing String message does not carry capture time.

Open the viewer before scanning. `/qr/data` carries new confirmations without
replaying past messages, so opening or reopening a window waits for the next
confirmation. Retention lasts while the window is open; the detector's CSV log
continues to provide the recorded history separately.

## Navigation Integration

Call the service after reaching the target:

    ros2 service call /qr/start_scan std_srvs/srv/Trigger "{}"

Subscribe to the result:

    ros2 topic echo /qr/data

Example result:

    data: RACKA_4X6M

Each successful scan publishes one result and returns to IDLE.

## Validation

The QR flow and expected topic/service sequence are documented here. Confirm the
camera, model availability, and decoding reliability on the team's Raspberry Pi
and camera before the competition.
The result window and replay-to-scanner ROS path have been exercised on WSL 2
with generated RACKA_DEMO and END images. Target-camera rendering, clipboard access
and optical performance on the team's Raspberry Pi remain hardware checks.

Expected sequence:

    IDLE -> START SCAN -> QR CONFIRMED -> DATA PUBLISHED -> IDLE

The [offline regression suite](../../common/tests/README.md) decodes generated
START, END and all four rack codes using real OpenCV, and checks the scanner
lifecycle with ROS transport fakes. Synthetic images do not establish camera
performance under blur, glare, changing distance or rack occlusion. The optional
WeChat models and the real cv_bridge/DDS path also require target-platform checks.

## Confirmation artifacts and bounded image work

Set `evidence_dir` to save the exact input frame that triggered each confirmation,
along with the full payload, image timestamp, decoder and `evidence_source` label.
It defaults to disabled. Files use unique names independent of the QR payload;
write failures are logged after result publication and cannot suppress the result.
The integrated demo enables this feature with the `synthetic_task_images` source.
These input images are not screenshots of the result window or physical acceptance.

OpenCV uses one worker thread by default to share CPU with Nav2/EKF. Set
`opencv_threads` from 1 to 4 only after measuring target performance. If decoding
fails on an image smaller than 240 pixels on its longest side, a single bounded
linear-upscale retry runs and its corner coordinates are converted back to the
original image. Strong motion blur still needs better capture conditions.

Use `bash scripts/nav.sh check robustness` from the repository root for the
production decoder's synthetic distortion report; optional WeChat models are
deliberately excluded from that fallback benchmark.

## Dependencies

Requires ROS 2 Jazzy, camera_ros, OpenCV QRCodeDetector, cv_bridge and NumPy.
WeChatQRCode support and the four downloaded models are optional. Keep the Pi's
ROS/OpenCV installation managed by apt/rosdep so cv_bridge uses compatible
libraries; the isolated pip environment described in the offline suite is for
host-side testing only.
