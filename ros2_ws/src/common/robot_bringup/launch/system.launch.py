"""Production composition; measured files and live drivers are prerequisites."""
from datetime import datetime
from pathlib import Path
import math
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_geometry.profile import load_profile
from robot_bringup.configuration import mission_parameters
from robot_bringup.launch_support import after_preflight, prepare_run_directory


def generate_launch_description():
    nav = Path(get_package_share_directory('robot_navigation'))
    vision = Path(get_package_share_directory('robot_vision'))
    defaults = {'geometry_file': str(Path(get_package_share_directory('robot_description'))/'config/mechanical.yaml'),
                'mission_file': str(Path(get_package_share_directory('robot_mission'))/'config/mission_params.yaml'),
                'start_camera': 'false', 'show_results': 'false', 'rviz': 'false',
                'image_topic': '/camera/image_raw', 'initial_loaded': 'true',
                'output_dir': str(Path.home()/'.ros/nav_vi_runs'/('robot-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]))}

    def prepare(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in [*defaults, 'initial_x', 'initial_y', 'initial_yaw']}
        load_profile(args['geometry_file'])
        params = mission_parameters(args['mission_file'])
        if any(not math.isfinite(float(args[key])) for key in ('initial_x', 'initial_y', 'initial_yaw')):
            raise ValueError('Initial pose must be finite, in the map frame (m/rad)')
        for key, value in {'qr_topic': '/qr/data', 'qr_start_service': '/qr/start_scan', 'qr_stop_service': '/qr/stop_scan'}.items():
            if params.get(key, value) != value:
                raise ValueError(f'Unified launch expects {key}={value}; use component launches for custom interfaces')
        path = prepare_run_directory(args['output_dir'])
        check = ExecuteProcess(cmd=['ros2', 'run', 'robot_bringup', 'preflight', '--mode', 'full',
            '--mission-file', args['mission_file'], '--geometry-file', args['geometry_file'],
            '--image-topic', args['image_topic'], '--output', str(path/'preflight.json')], output='screen')
        mission = Node(package='robot_mission', executable='delivery_mission', output='screen', parameters=[args['mission_file']])
        return [
            IncludeLaunchDescription(str(nav/'launch/bringup.launch.py'), launch_arguments={key: args[key] for key in
                ('geometry_file', 'initial_loaded', 'initial_x', 'initial_y', 'initial_yaw', 'rviz')}.items()),
            IncludeLaunchDescription(str(vision/'launch/vision_bringup.launch.py'), launch_arguments={
                **{key: args[key] for key in ('start_camera', 'show_results', 'image_topic')},
                'evidence_dir': str(path/'qr'), 'evidence_source': 'external_camera_unverified',
                'log_file': str(path/'qr_scan.csv')}.items()),
            Node(package='robot_bringup', executable='run_recorder', output='screen', parameters=[{
                'output_dir': str(path), 'source': 'robot_session_pending_physical_acceptance'}]),
            after_preflight(check, mission), check]

    return LaunchDescription([*[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        DeclareLaunchArgument('initial_x', description='Measured initial map x in metres'),
        DeclareLaunchArgument('initial_y', description='Measured initial map y in metres'),
        DeclareLaunchArgument('initial_yaw', description='Measured initial yaw in radians'),
        OpaqueFunction(function=prepare)])
