"""Start camera_ros and the on-demand QR scanner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    image_topic = LaunchConfiguration("image_topic")
    model_dir = LaunchConfiguration("model_dir")
    log_file = LaunchConfiguration("log_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_frame = LaunchConfiguration("camera_frame")
    show_results = LaunchConfiguration("show_results")
    preview_hz = LaunchConfiguration("preview_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument('evidence_dir', default_value=''),
            DeclareLaunchArgument('evidence_source', default_value='unspecified'),
            DeclareLaunchArgument(
                "show_results",
                default_value="false",
                description="Show the persistent QR result window (requires a desktop)",
            ),
            DeclareLaunchArgument(
                "preview_hz",
                default_value="10.0",
                description="Maximum window preview updates per second for each image topic",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the same ROS clock for the camera and scanner",
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="camera_optical_frame",
                description="Optical frame defined by the robot description",
            ),
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
                parameters=[{
                    "frame_id": ParameterValue(camera_frame, value_type=str),
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                }],
                remappings=[("~/image_raw", image_topic)],
            ),
            Node(
                package="robot_vision",
                executable="qr_detector_node",
                name="qr_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": ParameterValue(image_topic, value_type=str),
                        "model_dir": ParameterValue(model_dir, value_type=str),
                        "log_file": ParameterValue(log_file, value_type=str),
                        "evidence_dir": ParameterValue(LaunchConfiguration('evidence_dir'), value_type=str),
                        "evidence_source": ParameterValue(LaunchConfiguration('evidence_source'), value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            Node(
                package="robot_vision",
                executable="qr_popup",
                name="qr_popup",
                output="screen",
                condition=IfCondition(show_results),
                parameters=[{
                    "image_topic": ParameterValue(image_topic, value_type=str),
                    "preview_hz": ParameterValue(preview_hz, value_type=float),
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                }],
            ),
        ]
    )
