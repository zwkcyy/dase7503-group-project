"""Launch the QR detector with the team camera topic configuration."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("robot_vision"), "config", "qr_detector.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="robot_vision",
                executable="qr_detector",
                name="qr_detector",
                output="screen",
                parameters=[params],
            )
        ]
    )
