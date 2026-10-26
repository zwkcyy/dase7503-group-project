"""Run an existing EC bridge with isolated raw topics, plus the NAV adapter."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('robot_bringup'))
    defaults = {'start_bridge': 'false', 'bridge_root': '', 'port': '/dev/ttyUSB0',
                'adapter_file': str(share/'config/ec_20261025.yaml'),
                'enable_motion': 'false', 'enable_lift': 'false'}

    def prepare(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        actions = []
        if args['start_bridge'].lower() == 'true':
            path = Path(args['bridge_root']).expanduser().resolve()
            script = path/'tools/ros_serial_bridge.py'
            if not args['bridge_root'] or not script.is_file():
                raise ValueError('bridge_root must contain the supplied EC tools/ros_serial_bridge.py')
            # The EC source remains EC-owned. No guessed ASCII telemetry parser.
            remaps = {'/cmd_vel': '/ec/cmd_vel', '/wheel/odom': '/ec/wheel/odom',
                      '/imu/data_raw': '/ec/imu/data_raw', '/lift_cmd': '/ec/lift_cmd',
                      '/lift_state': '/ec/lift_state'}
            command = ['python3', str(script), '--ros-args', '-p', 'port:='+args['port']]
            for source, target in remaps.items():
                command.extend(['-r', source+':='+target])
            actions.append(ExecuteProcess(cmd=command, cwd=str(path), output='screen'))
        actions.append(Node(package='robot_bringup', executable='ec_adapter', output='screen',
            parameters=[args['adapter_file'], {
                key: args[key].lower() == 'true' for key in ('enable_motion', 'enable_lift')}]))
        return actions

    return LaunchDescription([*[DeclareLaunchArgument(key, default_value=value)
                               for key, value in defaults.items()], OpaqueFunction(function=prepare)])
