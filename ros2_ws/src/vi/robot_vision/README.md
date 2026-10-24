# Robot vision

This package subscribes to the Raspberry Pi camera image stream, detects the
game's QR labels, and publishes valid decoded values as `std_msgs/msg/String`.

## Topics

- Input: `/camera/image_raw` (`sensor_msgs/msg/Image`)
- Output: `/vision/qr_code` (`std_msgs/msg/String`)
- Debug output: `/vision/annotated_image` (`sensor_msgs/msg/Image`)

The topic names are ROS parameters and can be changed for the camera driver in
`config/qr_detector.yaml`.

## Run

After sourcing ROS 2 Jazzy and building the workspace:

```bash
ros2 launch robot_vision qr_detector.launch.py
```

Only `START`, `END`, and `RACK[A-D]_<four alphanumeric characters>` are
published. The rack suffix is team-specific; the detector accepts any suffix
matching the course format. It does not infer a rack's field location or robot
pose. Those belong to the mission/navigation layer.
