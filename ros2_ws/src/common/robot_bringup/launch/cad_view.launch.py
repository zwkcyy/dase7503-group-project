"""View the supplied CAD through the same NAV frame adaptation used on hardware."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = Path(get_package_share_directory('robot_description'))
    bringup = Path(get_package_share_directory('robot_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file', default_value=str(model/'config/mechanical_cad_20261025.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(str(model/'launch/description.launch.py'), launch_arguments={
            'geometry_file': LaunchConfiguration('geometry_file')}.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(bringup/'rviz/cad_navigation.rviz')])])
