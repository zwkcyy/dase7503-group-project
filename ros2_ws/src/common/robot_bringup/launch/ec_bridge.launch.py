"""Run the EC binary UART bridge (ec_serial_bridge) plus the NAV adapter.

The bridge used to live in the EC repository as tools/ros_serial_bridge.py and
was started through ``bridge_root``. That file was removed upstream (EC
6985c8a), so ``start_bridge:=true`` now starts the ec_serial_bridge package and
``bridge_root`` is kept only so existing includes and command lines do not
break.

Every launch argument that cad_ec.launch.py forwards is still declared here;
removing one would make that include fail with unused-argument errors.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('robot_bringup'))
    defaults = {'start_bridge': 'false', 'bridge_root': '', 'port': '/dev/ttyUSB0',
                'bridge_params_file': '', 'baudrate': '115200',
                'lift_cmd_mode': 'text', 'lift_state_mode': 'passthrough',
                'adapter_file': str(share/'config/ec_20261025.yaml'),
                'enable_motion': 'false', 'enable_lift': 'false'}

    def prepare(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        actions = []
        if args['start_bridge'].lower() == 'true':
            if args['bridge_root']:
                actions.append(LogInfo(msg='ec_bridge: bridge_root is ignored; the bridge now '
                                           'ships as the ec_serial_bridge package.'))
            # Empty bridge_params_file means "use the package default yaml".
            parameters = [args['bridge_params_file']] if args['bridge_params_file'] else []
            parameters.append({'port': args['port'], 'baudrate': args['baudrate'],
                               'lift_cmd_mode': args['lift_cmd_mode'],
                               'lift_state_mode': args['lift_state_mode']})
            actions.append(Node(package='ec_serial_bridge', executable='ec_serial_bridge',
                                output='screen', parameters=parameters))
        actions.append(Node(package='robot_bringup', executable='ec_adapter', output='screen',
                            parameters=[args['adapter_file'], {
                                key: args[key].lower() == 'true'
                                for key in ('enable_motion', 'enable_lift')}]))
        return actions

    return LaunchDescription([*[DeclareLaunchArgument(key, default_value=value)
                               for key, value in defaults.items()],
                              OpaqueFunction(function=prepare)])
