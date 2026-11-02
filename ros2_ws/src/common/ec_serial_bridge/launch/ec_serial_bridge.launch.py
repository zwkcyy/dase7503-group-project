"""Run the EC binary UART bridge alone; the NAV adapter is started separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ec_serial_bridge'))
    defaults = {
        'params_file': str(share/'config/ec_bridge_20261102.yaml'),
        'port': '/dev/ttyACM0',
        'lift_cmd_mode': 'text',
        'lift_state_mode': 'passthrough',
        'baudrate': '115200',
    }
    arguments = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    node = Node(package='ec_serial_bridge', executable='ec_serial_bridge', output='screen',
                parameters=[LaunchConfiguration('params_file'), {
                    'port': LaunchConfiguration('port'),
                    'baudrate': LaunchConfiguration('baudrate'),
                    'lift_cmd_mode': LaunchConfiguration('lift_cmd_mode'),
                    'lift_state_mode': LaunchConfiguration('lift_state_mode')}])
    return LaunchDescription([*arguments, node])
