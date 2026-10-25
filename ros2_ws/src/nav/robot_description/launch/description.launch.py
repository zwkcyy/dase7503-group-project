"""Publish the robot model and its static TFs with robot_state_publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    geometry_file = LaunchConfiguration('geometry_file')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro "', xacro_file, '" geometry_file:="', geometry_file, '"']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'geometry_file', default_value=PathJoinSubstitution([
                FindPackageShare('robot_description'), 'config', 'mechanical.yaml']),
            description='Single CAD geometry file; millimetres and radians'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
