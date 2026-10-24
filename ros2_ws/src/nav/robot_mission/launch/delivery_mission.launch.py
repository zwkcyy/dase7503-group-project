"""Launch the rack delivery state machine."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("robot_mission"), "config", "mission_params.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_mission",
                executable="delivery_mission",
                name="delivery_mission",
                output="screen",
                parameters=[LaunchConfiguration("params_file"), {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                }],
            )
        ]
    )
