"""Course-shaped presentation using the existing real navigation/vision pipeline."""
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robot_simulation.course_scene import load_scene, world_config, mission_config, map_config
from robot_simulation.launch_support import validate_domain
from robot_geometry.profile import load_profile


def generate_launch_description():
    share = Path(get_package_share_directory('robot_simulation'))
    nav = Path(get_package_share_directory('robot_navigation'))
    output = str(Path.home()/'.ros/nav_vi_runs'/('course-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]))

    def prepare(context):
        scene = load_scene(LaunchConfiguration('scene_file').perform(context))
        profile = load_profile(LaunchConfiguration('geometry_file').perform(context), allow_unconfirmed=True)
        path = Path(LaunchConfiguration('output_dir').perform(context)).expanduser().resolve()
        path.mkdir(parents=True, exist_ok=False)
        for name, value in [('scene.yaml', scene), ('world.yaml', world_config(scene)),
                            ('mission.yaml', mission_config(scene)), ('geometry.yaml', profile),
                            ('field.yaml', map_config(scene))]:
            (path/name).write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
        subprocess.run([sys.executable, str(nav/'scripts/generate_map.py'),
                        '--config', str(path/'field.yaml'), '--out-dir', str(path)], check=True)
        return [
            IncludeLaunchDescription(str(share/'launch/integrated_demo.launch.py'), launch_arguments={
                'domain_id': LaunchConfiguration('domain_id'), 'rviz': LaunchConfiguration('rviz'),
                'show_results': 'false', 'output_dir': str(path),
                'mission_file': str(path/'mission.yaml'), 'world_file': str(path/'world.yaml'),
                'map_file': str(path/'course_map.yaml'), 'geometry_file': str(path/'geometry.yaml'),
                'record_source': 'course_schematic_simulation_with_real_nav2_and_qr',
                'image_scenario': LaunchConfiguration('image_scenario')}.items()),
            Node(package='robot_simulation', executable='field_viewer', output='screen',
                 condition=IfCondition(LaunchConfiguration('show_field')),
                 parameters=[{'scene_file': str(path/'scene.yaml'),
                              'geometry_file': str(path/'geometry.yaml'),
                              'fullscreen': ParameterValue(LaunchConfiguration('fullscreen'), value_type=bool)}])]

    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='75'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('show_field', default_value='true'),
        DeclareLaunchArgument('fullscreen', default_value='false'),
        DeclareLaunchArgument('image_scenario', default_value='success'),
        DeclareLaunchArgument('scene_file', default_value=str(share/'config/course_layout.yaml')),
        DeclareLaunchArgument('geometry_file', default_value=str(share/'config/demo_geometry.yaml')),
        DeclareLaunchArgument('output_dir', default_value=output),
        OpaqueFunction(function=validate_domain),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        OpaqueFunction(function=prepare)])
