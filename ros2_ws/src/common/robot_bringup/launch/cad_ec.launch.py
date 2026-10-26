"""Supplied CAD and EC contract commissioning; no autonomous delivery mission."""
from datetime import datetime
from pathlib import Path
import math
import uuid

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_bringup.launch_support import prepare_run_directory


def generate_launch_description():
    share = Path(get_package_share_directory('robot_bringup'))
    model = Path(get_package_share_directory('robot_description'))
    nav = Path(get_package_share_directory('robot_navigation'))
    vision = Path(get_package_share_directory('robot_vision'))
    defaults = {
        'geometry_file': str(model/'config/mechanical_cad_20261025.yaml'),
        'adapter_file': str(share/'config/ec_20261025.yaml'),
        'navigation': 'false', 'rviz': 'true', 'vision': 'true', 'show_results': 'false',
        'start_camera': 'false', 'image_topic': '/camera/image_raw',
        'start_bridge': 'false', 'bridge_root': '', 'port': '/dev/ttyUSB0',
        'enable_motion': 'false', 'enable_lift': 'false',
        'initial_x': '', 'initial_y': '', 'initial_yaw': '',
        'use_course_start': 'false',
        'field_config_file': str(nav/'config/field_config.yaml'),
        'output_dir': str(Path.home()/'.ros/nav_vi_runs'/(
            'cad-ec-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]))}

    def prepare(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        enabled = lambda key: args[key].lower() == 'true'
        if enabled('navigation'):
            if enabled('use_course_start'):
                if any(args[key] for key in ('initial_x', 'initial_y', 'initial_yaw')):
                    raise ValueError('Choose use_course_start or explicit initial_x/y/yaw, not both')
                with open(args['field_config_file'], encoding='utf-8') as stream:
                    start = yaml.safe_load(stream)['nav_start']
                args.update(initial_x=str(float(start['x_mm'])/1000.),
                            initial_y=str(float(start['y_mm'])/1000.),
                            initial_yaw=str(float(start['yaw_rad'])))
            for key in ('initial_x', 'initial_y', 'initial_yaw'):
                if not args[key] or not math.isfinite(float(args[key])):
                    raise ValueError('Empty navigation requires explicit finite initial_x/y/yaw (m/rad)')
        path = prepare_run_directory(args['output_dir'])
        actions = [LogInfo(msg='CAD/EC commissioning: unloaded geometry only; no delivery mission is started.'),
            IncludeLaunchDescription(str(share/'launch/ec_bridge.launch.py'), launch_arguments={
                key: args[key] for key in ('adapter_file', 'start_bridge', 'bridge_root', 'port',
                                           'enable_motion', 'enable_lift')}.items()),
            Node(package='robot_bringup', executable='run_recorder', output='screen',
                 parameters=[{'output_dir': str(path), 'source': 'cad_ec_commissioning_unverified'}])]
        if enabled('navigation'):
            actions.append(IncludeLaunchDescription(str(nav/'launch/bringup.launch.py'), launch_arguments={
                **{key: args[key] for key in ('geometry_file', 'rviz', 'initial_x', 'initial_y', 'initial_yaw')},
                'allow_unconfirmed_geometry': 'false', 'empty_only': 'true', 'initial_loaded': 'false',
                'ekf_params_file': str(share/'config/ekf_ec_10hz.yaml')}.items()))
        else:
            actions.append(IncludeLaunchDescription(str(share/'launch/cad_view.launch.py'), launch_arguments={
                key: args[key] for key in ('geometry_file', 'rviz')}.items()))
        if enabled('vision'):
            actions.append(IncludeLaunchDescription(str(vision/'launch/vision_bringup.launch.py'), launch_arguments={
                **{key: args[key] for key in ('start_camera', 'image_topic', 'show_results')},
                'evidence_dir': str(path/'qr'), 'evidence_source': 'external_camera_unverified',
                'log_file': str(path/'qr_scan.csv')}.items()))
        return actions

    return LaunchDescription([*[DeclareLaunchArgument(key, default_value=value)
                               for key, value in defaults.items()], OpaqueFunction(function=prepare)])
