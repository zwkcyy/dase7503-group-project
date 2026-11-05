"""Run the EC binary UART bridge alone; the NAV adapter is started separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
                    'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                    'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                    'lift_cmd_mode': ParameterValue(LaunchConfiguration('lift_cmd_mode'), value_type=str),
                    'lift_state_mode': ParameterValue(LaunchConfiguration('lift_state_mode'), value_type=str)}])
    return LaunchDescription([*arguments, node])
