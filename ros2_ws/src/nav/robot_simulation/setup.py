from glob import glob
from setuptools import setup

setup(name='robot_simulation', version='0.1.0', packages=['robot_simulation'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_simulation']),
                  ('share/robot_simulation', ['package.xml', 'README.md']),
                  ('share/robot_simulation/config', glob('config/*.yaml')),
                  ('share/robot_simulation/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], entry_points={'console_scripts': [
          'planar_base = robot_simulation.planar_base:main',
          'mock_peers = robot_simulation.mock_peers:main',
          'image_replay = robot_simulation.image_replay:main']})
