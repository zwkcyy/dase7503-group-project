"""Exercise real QR decoding and its window without a physical camera."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from robot_simulation.launch_support import validate_domain
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='75'),
        DeclareLaunchArgument('input_file', default_value=''),
        DeclareLaunchArgument('payload', default_value='START'),
        DeclareLaunchArgument('show_results', default_value='true'),
        DeclareLaunchArgument('evidence_dir', default_value=''),
        OpaqueFunction(function=validate_domain),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(package='robot_simulation', executable='image_replay', output='screen',
             parameters=[{'input_file': LaunchConfiguration('input_file'), 'payload': LaunchConfiguration('payload')}]),
        Node(package='robot_vision', executable='qr_detector_node', output='screen', parameters=[{
            'evidence_dir': LaunchConfiguration('evidence_dir'), 'evidence_source': 'image_replay'}]),
        Node(package='robot_vision', executable='qr_popup', output='screen',
             condition=IfCondition(LaunchConfiguration('show_results'))),
    ])
