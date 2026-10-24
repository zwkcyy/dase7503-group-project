# Robot Vision - DASE7503

ROS 2 on-demand QR code scanner for Raspberry Pi 5.

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

Expected sequence:

    IDLE -> START SCAN -> QR CONFIRMED -> DATA PUBLISHED -> IDLE

Successfully decoded the coursework END QR code.

## Dependencies

Requires ROS 2 Jazzy, camera_ros, OpenCV with WeChatQRCode,
cv_bridge, NumPy and the downloaded QR models.
