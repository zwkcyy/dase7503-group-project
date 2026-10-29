from glob import glob
from setuptools import setup

setup(name='robot_gazebo', version='0.1.0', packages=['robot_gazebo'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_gazebo']),
                  ('share/robot_gazebo', ['package.xml', 'README.md']),
                  ('share/robot_gazebo/config', glob('config/*')),
                  ('share/robot_gazebo/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], entry_points={'console_scripts': [
          'gazebo_io = robot_gazebo.io:main',
          'gazebo_audit = robot_gazebo.audit:main',
          'verify_gazebo = robot_gazebo.verify:main',
          'verify_gazebo_faults = robot_gazebo.faults:main',
          'generate_world = robot_gazebo.world:main']})
