"""Navigation: Nav2 controller, planner, behaviors, BT navigator and velocity smoother.

Command velocity chain:
  controller_server / behavior_server -> /cmd_vel_nav
  -> velocity_smoother -> /cmd_vel (geometry_msgs/Twist)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
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

    nav2_params = ParameterFile(params_file, allow_substs=True)

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
    ]
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
    ]

    nodes = GroupAction([
        SetParameter('use_sim_time', use_sim_time),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
            arguments=ros_args,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
            arguments=ros_args,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
            arguments=ros_args,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            arguments=ros_args,
            # Each name is remapped once, so the output does not chain into the input
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
            arguments=ros_args,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
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
        + [nodes]
    )
