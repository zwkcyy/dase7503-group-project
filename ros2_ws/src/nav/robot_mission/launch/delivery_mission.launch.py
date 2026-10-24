"""Launch the rack delivery state machine."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("robot_mission"), "config", "mission_params.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="robot_mission",
                executable="delivery_mission",
                name="delivery_mission",
                output="screen",
                parameters=[params],
            )
        ]
    )
