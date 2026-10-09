# Robot Vision - DASE7503

ROS 2 on-demand QR code scanner for Raspberry Pi 5.

## Platform

- Raspberry Pi 5
- Raspberry Pi Camera Module 3 (IMX708)
- Ubuntu 24.04
- ROS 2 Jazzy

## QR Recognition

- Primary: OpenCV WeChatQRCode (CNN + Super Resolution)
- Fallback: OpenCV QRCodeDetector
- Two-hit QR confirmation

Supported QR codes: START, END, RACKA_XXXX, RACKB_XXXX, RACKC_XXXX, RACKD_XXXX.

## ROS Interface

Package: `robot_vision`

Executable: `qr_detector_node`

Node: `/qr_detector`

Camera input:
- `/camera/image_raw` (sensor_msgs/msg/Image)

Start scanning:
- `/qr/start_scan` (std_srvs/srv/Trigger)

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

Start camera_ros first:

    ros2 run camera_ros camera_node

Then start vision:

    ros2 run robot_vision qr_detector_node

## Navigation Integration

Call the service after reaching the target:

    ros2 service call /qr/start_scan std_srvs/srv/Trigger "{}"

Subscribe to the result:

    ros2 topic echo /qr/data

Example result:

    data: RACKA_4X6M

Each successful scan publishes one result and returns to IDLE.

## Validation

Tested with Raspberry Pi 5 and Camera Module 3.

Verified sequence:

    IDLE -> START SCAN -> QR CONFIRMED -> DATA PUBLISHED -> IDLE

Successfully decoded the coursework END QR code.

## Dependencies

Requires ROS 2 Jazzy, camera_ros, OpenCV with WeChatQRCode,
cv_bridge, NumPy and the downloaded QR models.
