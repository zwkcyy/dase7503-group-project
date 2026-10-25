"""Run real EKF, AMCL, MPPI and Nav2 on synthetic sensors; optional mock mission."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, OpaqueFunction
from robot_simulation.launch_support import validate_domain
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('robot_simulation'))
    nav = Path(get_package_share_directory('robot_navigation'))
    geometry = str(share / 'config/demo_geometry.yaml')
    world_file = str(share / 'config/demo_world.yaml')
    with open(world_file) as stream:
        initial = yaml.safe_load(stream)['initial_pose']
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='75'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mission', default_value='false'),
        OpaqueFunction(function=validate_domain),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(package='robot_simulation', executable='planar_base', output='screen',
             parameters=[{'geometry_file': geometry, 'world_file': world_file}]),
        IncludeLaunchDescription(str(nav / 'launch/bringup.launch.py'), launch_arguments={
            'geometry_file': geometry, 'allow_unconfirmed_geometry': 'true', 'initial_loaded': 'false',
            'rviz': LaunchConfiguration('rviz'), 'initial_x': str(initial[0]),
            'initial_y': str(initial[1]), 'initial_yaw': str(initial[2])}.items()),
        TimerAction(period=8., actions=[
            Node(package='robot_simulation', executable='mock_peers', output='screen',
                 condition=IfCondition(LaunchConfiguration('mission')),
                 parameters=[{'mock_navigation': False, 'mock_envelope': False}]),
            Node(package='robot_mission', executable='delivery_mission', output='screen',
                 condition=IfCondition(LaunchConfiguration('mission')),
                 parameters=[str(share / 'config/demo_mission.yaml')]),
        ]),
    ])
