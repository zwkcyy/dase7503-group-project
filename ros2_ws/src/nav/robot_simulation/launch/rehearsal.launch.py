"""Exercise the production mission against ROS action/service/topic mocks."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from robot_simulation.launch_support import validate_domain
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robot_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='75'),
        DeclareLaunchArgument('scenario', default_value='success'),
        DeclareLaunchArgument('competition_time_s', default_value='180.0'),
        DeclareLaunchArgument('navigation_timeout_s', default_value='5.0'),
        OpaqueFunction(function=validate_domain),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(package='robot_simulation', executable='mock_peers', output='screen',
             parameters=[{'scenario': LaunchConfiguration('scenario')}]),
        Node(package='robot_mission', executable='delivery_mission', output='screen',
             parameters=[str(share / 'config/demo_mission.yaml'), {
                 'competition_time_s': ParameterValue(LaunchConfiguration('competition_time_s'), value_type=float),
                 'navigation_timeout_s': ParameterValue(LaunchConfiguration('navigation_timeout_s'), value_type=float)}]),
    ])
