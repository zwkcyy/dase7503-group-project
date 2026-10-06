"""Full NAV stack: localisation + navigation, optionally RViz.

Example:
  ros2 launch robot_navigation bringup.launch.py initial_x:=0.3 initial_y:=0.3 rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_navigation')
    launch_dir = PathJoinSubstitution([pkg_share, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    rviz = LaunchConfiguration('rviz')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml']),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure and activate the lifecycle nodes'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='Log level'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='Initial x of base_link in map [m]'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='Initial y of base_link in map [m]'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='Initial yaw of base_link in map [rad]'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz (keep false on the robot)'),
    ]

    common_args = {
        'use_sim_time': use_sim_time,
        'params_file': params_file,
        'autostart': autostart,
        'log_level': log_level,
    }

    localization = IncludeLaunchDescription(
        PathJoinSubstitution([launch_dir, 'localization.launch.py']),
        launch_arguments={
            **common_args,
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([launch_dir, 'navigation.launch.py']),
        launch_arguments=common_args.items(),
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'nav.rviz'])],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription(declare_args + [localization, navigation, rviz_node])
