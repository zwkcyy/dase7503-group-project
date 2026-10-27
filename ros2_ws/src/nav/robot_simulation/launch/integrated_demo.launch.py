"""Real navigation + real image decoding + mission, with synthetic sensors/lift."""
from datetime import datetime
from pathlib import Path
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_bringup.launch_support import after_preflight, prepare_run_directory
from robot_simulation.launch_support import validate_domain


def generate_launch_description():
    share = Path(get_package_share_directory('robot_simulation'))

    def prepare(context):
        mission_file = LaunchConfiguration('mission_file').perform(context)
        path = prepare_run_directory(LaunchConfiguration('output_dir').perform(context))
        check = ExecuteProcess(cmd=['ros2', 'run', 'robot_bringup', 'preflight', '--mode', 'full',
            '--mission-file', mission_file, '--timeout', '45', '--output', str(path/'preflight.json')], output='screen')
        mission = Node(package='robot_mission', executable='delivery_mission', output='screen', parameters=[mission_file])
        return [
            IncludeLaunchDescription(str(share/'launch/navigation_demo.launch.py'), launch_arguments={
                'domain_id': LaunchConfiguration('domain_id'), 'mission': 'false',
                'geometry_file': LaunchConfiguration('geometry_file'),
                'world_file': LaunchConfiguration('world_file'),
                'map_file': LaunchConfiguration('map_file'),
                'params_file': LaunchConfiguration('params_file'),
                'rviz': LaunchConfiguration('rviz')}.items()),
            Node(package='robot_simulation', executable='mock_peers', output='screen', parameters=[{
                'mock_navigation': False, 'mock_envelope': False, 'mock_qr': False}],
                condition=IfCondition(LaunchConfiguration('mock_lift'))),
            Node(package='robot_mission', executable='docking_controller', output='screen',
                 condition=IfCondition(LaunchConfiguration('docking')),
                 parameters=[{'geometry_file': LaunchConfiguration('geometry_file')}]),
            Node(package='robot_simulation', executable='mission_camera', output='screen', parameters=[{
                'mission_file': mission_file, 'scenario': LaunchConfiguration('image_scenario')}]),
            Node(package='robot_vision', executable='qr_detector_node', output='screen', parameters=[{
                'evidence_dir': str(path/'qr'), 'evidence_source': 'synthetic_task_images',
                'log_file': str(path/'qr_scan.csv')}]),
            Node(package='robot_vision', executable='qr_popup', output='screen',
                 condition=IfCondition(LaunchConfiguration('show_results'))),
            Node(package='robot_bringup', executable='run_recorder', output='screen', parameters=[{
                'output_dir': str(path), 'source': LaunchConfiguration('record_source')}]),
            after_preflight(check, mission), check]

    output = str(Path.home()/'.ros/nav_vi_runs'/('integrated-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]))
    return LaunchDescription([
        DeclareLaunchArgument('mission_file', default_value=str(share/'config/demo_mission.yaml')),
        DeclareLaunchArgument('geometry_file', default_value=str(share/'config/demo_geometry.yaml')),
        DeclareLaunchArgument('world_file', default_value=str(share/'config/demo_world.yaml')),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('params_file', default_value=str(Path(get_package_share_directory('robot_navigation'))/'config/nav2_params.yaml')),
        DeclareLaunchArgument('mock_lift', default_value='true'),
        DeclareLaunchArgument('docking', default_value='false'),
        DeclareLaunchArgument('record_source', default_value='planar_simulation_with_real_qr_decoder'),
        DeclareLaunchArgument('domain_id', default_value='75'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('show_results', default_value='true'),
        DeclareLaunchArgument('image_scenario', default_value='success'),
        DeclareLaunchArgument('output_dir', default_value=output),
        OpaqueFunction(function=validate_domain),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        OpaqueFunction(function=prepare)])
