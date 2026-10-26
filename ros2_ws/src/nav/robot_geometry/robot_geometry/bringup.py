"""Create one consistent NAV launch from the selected mechanical profile."""

from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .profile import load_profile, nav_parameters


def generate_launch_description():
    nav = Path(get_package_share_directory('robot_navigation'))
    description = Path(get_package_share_directory('robot_description'))
    defaults = {'geometry_file': str(description / 'config/mechanical.yaml'),
                'allow_unconfirmed_geometry': 'false', 'initial_loaded': 'true',
                'empty_only': 'false',
                'ekf_params_file': str(nav / 'config/ekf.yaml'),
                'params_file': str(nav / 'config/nav2_params.yaml'),
                'use_sim_time': 'false', 'autostart': 'true', 'log_level': 'info',
                'initial_x': '0.0', 'initial_y': '0.0', 'initial_yaw': '0.0', 'rviz': 'false'}

    def prepare(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        boolean = lambda key: args[key].lower() in ('true', '1')
        if boolean('empty_only') and boolean('initial_loaded'):
            raise ValueError('Empty-only commissioning cannot start loaded')
        profile = load_profile(args['geometry_file'], boolean('allow_unconfirmed_geometry'), boolean('empty_only'))
        with open(args['params_file'], encoding='utf-8') as stream:
            params = nav_parameters(yaml.safe_load(stream), profile, boolean('initial_loaded'))
        directory = tempfile.TemporaryDirectory(prefix='dase7503-nav-')
        generated = Path(directory.name) / 'nav2.yaml'
        generated.write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf-8')
        common = {key: args[key] for key in ('use_sim_time', 'autostart', 'log_level')}
        common['params_file'] = str(generated)

        def cleanup(context):
            directory.cleanup()
            return []

        actions = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]
        actions.append(IncludeLaunchDescription(str(nav / 'launch/localization.launch.py'),
            launch_arguments={**common, 'geometry_file': args['geometry_file'],
                              'external_scan_filter': 'true',
                              'ekf_params_file': args['ekf_params_file'],
                              **{key: args[key] for key in ('initial_x', 'initial_y', 'initial_yaw')}}.items()))
        actions.append(IncludeLaunchDescription(str(nav / 'launch/navigation.launch.py'),
                                               launch_arguments=common.items()))
        actions.append(Node(package='robot_geometry', executable='load_envelope', output='screen',
            parameters=[{'geometry_file': args['geometry_file'],
                         'allow_unconfirmed_geometry': boolean('allow_unconfirmed_geometry'),
                         'empty_only': boolean('empty_only'),
                         'initial_loaded': boolean('initial_loaded'), 'use_sim_time': boolean('use_sim_time')}]))
        if boolean('rviz'):
            actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', str(nav / 'rviz/nav.rviz')],
                parameters=[{'use_sim_time': boolean('use_sim_time')}]))
        return actions

    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=prepare)])
