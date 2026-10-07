# Robot Vision: QR Scanner

Scan QR codes with a camera and display the original payload, its meaning, and the scan time in a persistent window.

## Behavior

- Display the first successfully decoded QR code.
- Keep the payload and scan time unchanged while the same code remains visible.
- Retain the previous result when the code leaves the frame, decoding fails, or the image stream stops.
- Replace the content of the same window when a different payload is decoded. `A -> B -> A` displays all three results in sequence.
- When multiple decodable codes are visible, select the one closest to the image center. Move the target code toward the center to select it.
- Preserve any decoded payload, with descriptions for the course formats: `START`, `END`, and `RACKA_XXXX` through `RACKD_XXXX`.
- Outline the selected code in green in the camera preview. Long payloads can be scrolled and copied.
- Click Exit, press Esc, or close the window to stop scanning.

The window remains open during scanning and does not require an OK button between results. Two physical QR codes with identical payloads are treated as the same result. The interface currently displays bilingual Chinese and English labels.

## Camera and input sources

The course document `3. Group Projects Materials_02.pdf` lists the **Raspberry Pi Camera Module 3**, connected to a **Raspberry Pi 5**, on page 4. Page 15 recommends `camera_ros` and `libcamera` and specifies building both from source. Follow the course or team's installation instructions for these components. This package does not include the camera driver.

The default ROS input is `/camera/image_raw`, with message type `sensor_msgs/msg/Image`. `cv_bridge` converts frames to BGR before OpenCV processes them. The subscriber accepts BEST_EFFORT camera publishers and keeps only the latest queued frame. Processing is limited to 10 FPS by default; adjust it with `--fps`. This setting does not change the camera driver's capture rate.

Use ROS mode for the CSI Camera Module 3. Use `--source opencv` for a local built-in or USB camera, or `--source image` for a static image test.

This Python program runs on the Raspberry Pi; it does not require flashing firmware to the ESP32.

### Raspberry Pi with ROS 2 Jazzy

Run these commands in an Ubuntu 24.04 desktop environment with ROS 2 Jazzy installed. Use the system Python that matches the apt-installed `cv_bridge`.

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-pil python3-pil.imagetk python3-tk ros-jazzy-cv-bridge

# From the repository root:
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_vision
source install/setup.bash
```

In terminal 1, start the camera driver built according to the course instructions. If it is in a separate workspace, source that workspace's `install/setup.bash` first.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run camera_ros camera_node
```

In terminal 2, work from this repository's `ros2_ws` directory, check the actual image topic, and start the scanner.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list -t
# Select a sensor_msgs/msg/Image topic, rather than a compressed image topic.
ros2 run robot_vision qr_popup --source ros --topic /camera/image_raw
```

Replace `--topic` if your camera publishes under a different name. During initial deployment, check the camera driver's exposure, focus, and image-size parameters. Test at the actual scanning distance and lighting conditions, with enough pixels for the QR code to remain sharp.

The Tk window requires a graphical desktop, available through a connected display or remote desktop. A plain SSH session without display forwarding cannot show the window.

This application reads camera images and displays QR results. It does not control the chassis or publish navigation or mission commands. It does not change other groups' interfaces.

### Local testing without ROS

Run from this package directory, `ros2_ws/src/vi/robot_vision`, using Python 3.10 or newer:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt

# Scan with a built-in or USB camera. Try another device index if necessary.
python -m robot_vision.qr_popup --source opencv --camera 0

# Test with a static PNG or JPG image.
python -m robot_vision.qr_popup --source image --image /path/to/qr.png
```

On Linux, also install Tk with `sudo apt install python3-tk`. On macOS, use a Python installation with Tk support and grant camera access to the terminal running Python. Image mode works without a camera.

## Validation

```bash
python -m unittest discover -s tests -v
```

The tests generate actual QR images with OpenCV and verify `START`, `END`, all four rack formats, and selection among multiple codes. They also check retention and switching for `A -> blank -> A -> B -> A`. No camera or GUI is required for these tests.

Before deployment, open the scanner, scan START, remove it to confirm that the result stays visible, scan a rack code to confirm replacement, and finally scan END. Save screenshots of each decoded result as required by the course QR specification. Check that the last result remains visible after disconnecting the camera and that the camera can be reopened after closing the application.

Validation completed on macOS covers offline QR decoding and GUI result retention, switching, copying, and shutdown. The six QR codes in the course appendix were also decoded from rendered PDF pages. Camera Module 3 integration, `camera_ros`, ROS builds, and real-device performance still require validation on the Raspberry Pi.
