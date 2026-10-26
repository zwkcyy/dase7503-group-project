"""Publish the robot model and its static TFs with robot_state_publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    geometry_file = LaunchConfiguration('geometry_file')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro "', xacro_file, '" geometry_file:="', geometry_file, '"']), value_type=str)

    def publish(context):
        with open(geometry_file.perform(context), encoding='utf-8') as stream:
            profile = yaml.safe_load(stream)
        selected = profile.get('model_source', 'primitives')
        value = robot_description
        if selected == 'cad_20261025':
            from robot_geometry.cad_model import description
            value = ParameterValue(description(profile), value_type=str)
        elif selected != 'primitives':
            raise ValueError(f'Unknown model_source: {selected}')
        return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                     name='robot_state_publisher', output='screen', parameters=[{
                         'robot_description': value,
                         'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}])]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'geometry_file', default_value=PathJoinSubstitution([
                FindPackageShare('robot_description'), 'config', 'mechanical.yaml']),
            description='Single CAD geometry file; millimetres and radians'),

        OpaqueFunction(function=publish),
    ])
