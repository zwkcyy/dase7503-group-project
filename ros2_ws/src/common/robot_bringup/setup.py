from glob import glob
from setuptools import setup

setup(name='robot_bringup', version='0.1.0', packages=['robot_bringup'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_bringup']),
                  ('share/robot_bringup', ['package.xml']),
                  ('share/robot_bringup/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], entry_points={'console_scripts': [
          'preflight = robot_bringup.preflight:main',
          'run_recorder = robot_bringup.run_recorder:main']})
