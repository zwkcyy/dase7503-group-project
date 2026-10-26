from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    package = Path(get_package_share_directory('logistics_robot_description'))
    filename = LaunchConfiguration('model').perform(context)
    allowed = {'logistics_robot.urdf', 'model_snapshot_fixed.urdf'}
    if filename not in allowed:
        raise ValueError('model must be one of: ' + ', '.join(sorted(allowed)))
    description = (package / 'urdf' / filename).read_text(encoding='utf-8')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(package / 'rviz' / 'robot.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='logistics_robot.urdf'),
        OpaqueFunction(function=launch_setup),
    ])
