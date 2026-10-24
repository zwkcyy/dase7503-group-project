"""Start camera_ros and the on-demand QR scanner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    image_topic = LaunchConfiguration("image_topic")
    model_dir = LaunchConfiguration("model_dir")
    log_file = LaunchConfiguration("log_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description="Start camera_ros with the QR scanner",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="Camera image topic subscribed to by the QR scanner",
            ),
            DeclareLaunchArgument(
                "model_dir",
                default_value="~/.local/share/robot_vision/wechat_qrcode",
                description="Directory containing WeChatQRCode model files",
            ),
            DeclareLaunchArgument(
                "log_file",
                default_value="~/.ros/robot_vision/qr_scan_log.csv",
                description="Path used to save confirmed QR results",
            ),
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                condition=IfCondition(start_camera),
                remappings=[("/camera/image_raw", image_topic)],
            ),
            Node(
                package="robot_vision",
                executable="qr_detector_node",
                name="qr_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "model_dir": model_dir,
                        "log_file": log_file,
                    }
                ],
            ),
        ]
    )
