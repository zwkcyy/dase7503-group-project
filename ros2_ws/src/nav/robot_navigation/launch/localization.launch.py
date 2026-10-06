"""Localisation: robot model, EKF, laser filter, map_server and AMCL.

TF published here:
  odom -> base_link   ekf_filter_node (robot_localization)
  map  -> odom        amcl
  base_link -> sensors robot_state_publisher
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    nav2_params = ParameterFile(params_file, allow_substs=True)
    ekf_params = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    laser_filter_params = PathJoinSubstitution([pkg_share, 'config', 'laser_filter.yaml'])

    lifecycle_nodes = ['map_server', 'amcl']
    ros_args = ['--ros-args', '--log-level', log_level]

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
    ]

    description = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('robot_description'), 'launch', 'description.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nodes = GroupAction([
        SetParameter('use_sim_time', use_sim_time),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            arguments=ros_args,
        ),
        # No name= here: a __node remap would also rename the box filter's
        # internal lifecycle node and create two nodes with the same name.
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            output='screen',
            parameters=[laser_filter_params],
            arguments=ros_args,
            remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params],
            arguments=ros_args,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                nav2_params,
                {
                    'initial_pose.x': ParameterValue(initial_x, value_type=float),
                    'initial_pose.y': ParameterValue(initial_y, value_type=float),
                    'initial_pose.yaw': ParameterValue(initial_yaw, value_type=float),
                },
            ],
            arguments=ros_args,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': lifecycle_nodes,
            }],
            arguments=ros_args,
        ),
    ])

    return LaunchDescription(
        [SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')]
        + declare_args
        + [description, nodes]
    )
