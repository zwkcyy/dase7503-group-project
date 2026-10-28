"""Gazebo physics, rendered sensors, real Nav2 and real QR decoding."""
from datetime import datetime
from pathlib import Path
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable,
                            ExecuteProcess, IncludeLaunchDescription, TimerAction,
                            RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_bringup.launch_support import after_preflight
from robot_simulation.launch_support import validate_domain
from robot_gazebo.world import generate


def bridge_config():
    result = []
    def add(gz, ros, ros_type, gz_type, direction='GZ_TO_ROS'):
        result.append(dict(gz_topic_name=gz, ros_topic_name=ros, ros_type_name=ros_type,
                           gz_type_name='gz.msgs.'+gz_type, direction=direction))
    add('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'Clock')
    add('/gazebo/scan', '/scan', 'sensor_msgs/msg/LaserScan', 'LaserScan')
    add('/gazebo/camera', '/camera/image_raw', 'sensor_msgs/msg/Image', 'Image')
    add('/gazebo/imu', '/gazebo/imu', 'sensor_msgs/msg/Imu', 'IMU')
    add('/gazebo/wheel_odom', '/gazebo/wheel_odom', 'nav_msgs/msg/Odometry', 'Odometry')
    add('/gazebo/joint_states', '/gazebo/joint_states', 'sensor_msgs/msg/JointState', 'Model')
    add('/gazebo/cmd_vel', '/gazebo/cmd_vel', 'geometry_msgs/msg/Twist', 'Twist', 'ROS_TO_GZ')
    add('/gazebo/lift_target', '/gazebo/lift_target', 'std_msgs/msg/Float64', 'Double', 'ROS_TO_GZ')
    return result


def generate_launch_description():
    own = Path(get_package_share_directory('robot_gazebo'))
    sim = Path(get_package_share_directory('robot_simulation'))
    nav = Path(get_package_share_directory('robot_navigation'))
    run_id = datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]
    defaults = dict(domain_id='75', gui='true', rviz='false', mission='true', navigation='true',
                    show_results='false', headless_rendering='false',
                    output_dir=str(Path.home()/'.ros/nav_vi_runs'/('gazebo-'+run_id)),
                    scene_file=str(sim/'config/course_layout.yaml'), physics_file=str(own/'config/physics.yaml'))

    def prepare(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        enabled = lambda key: args[key].lower() == 'true'
        if enabled('mission') and not enabled('navigation'):
            raise ValueError('Mission requires navigation')
        path = generate(args['output_dir'], args['scene_file'], args['physics_file'])
        (path/'bridge.yaml').write_text(yaml.safe_dump(bridge_config()), encoding='utf-8')
        (path/'launch_arguments.yaml').write_text(yaml.safe_dump({**args, 'gz_partition': context.environment['GZ_PARTITION']}), encoding='utf-8')
        command = ['gz', 'sim', '-r', '-s', '-v', '3', str(path/'course.sdf')]
        if enabled('headless_rendering'): command.append('--headless-rendering')
        server = ExecuteProcess(cmd=command, output='screen')
        adapter = Node(package='robot_gazebo', executable='gazebo_io', parameters=[{
            'output_dir': str(path), 'use_sim_time': True}], output='screen')
        bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge',
                      parameters=[{'config_file': str(path/'bridge.yaml'), 'use_sim_time': True}], output='screen')
        actions = [server,
            RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
                EmitEvent(event=Shutdown(reason='Gazebo server stopped'))])),
            bridge,
            RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[
                EmitEvent(event=Shutdown(reason='Gazebo transport bridge stopped'))])),
            adapter,
            RegisterEventHandler(OnProcessExit(target_action=adapter, on_exit=[
                EmitEvent(event=Shutdown(reason='Gazebo actuator watchdog stopped'))])),
            Node(package='robot_vision', executable='qr_detector_node', output='screen', parameters=[{
                'use_sim_time': True, 'evidence_dir': str(path/'qr'),
                'evidence_source': 'gazebo_ogre2_rendered_camera', 'log_file': str(path/'qr_scan.csv')}])]
        if enabled('gui'):
            actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g', '--gui-config', str(own/'config/gui.config')], output='screen'))
        if enabled('show_results'):
            actions.append(Node(package='robot_vision', executable='qr_popup', parameters=[{'use_sim_time': True}]))
        if enabled('navigation'):
            scene = yaml.safe_load((path/'scene.yaml').read_text())
            x, y, yaw = scene['initial_pose_mm_rad']
            dx, dy, da = scene.get('localization_offset_mm_rad', [0., 0., 0.])
            actions += [IncludeLaunchDescription(str(nav/'launch/bringup.launch.py'), launch_arguments={
                'geometry_file': str(path/'geometry.yaml'), 'allow_unconfirmed_geometry': 'true',
                'initial_loaded': 'false', 'use_sim_time': 'true', 'rviz': args['rviz'],
                'map_file': str(path/'course_map.yaml'), 'params_file': str(path/'nav2.yaml'),
                'initial_x': str((x+dx)/1000), 'initial_y': str((y+dy)/1000), 'initial_yaw': str(yaw+da)}.items()),
                Node(package='robot_mission', executable='docking_controller', output='screen',
                     parameters=[{'geometry_file': str(path/'geometry.yaml'), 'use_sim_time': True}])]
        if enabled('mission'):
            check = ExecuteProcess(cmd=['ros2', 'run', 'robot_bringup', 'preflight', '--mode', 'full',
                '--mission-file', str(path/'mission.yaml'), '--use-sim-time', '--timeout', '90',
                '--output', str(path/'preflight.json')], output='screen')
            mission = Node(package='robot_mission', executable='delivery_mission',
                           parameters=[str(path/'mission.yaml')], output='screen')
            actions += [after_preflight(check, mission), TimerAction(period=5., actions=[check]),
                Node(package='robot_bringup', executable='run_recorder', output='screen', parameters=[{
                    'use_sim_time': True, 'output_dir': str(path), 'source': 'gazebo_harmonic_physics_rendered_camera'}])]
        return actions

    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]+[
        OpaqueFunction(function=validate_domain),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('GZ_PARTITION', 'dase7503-'+run_id),
        OpaqueFunction(function=prepare)])
