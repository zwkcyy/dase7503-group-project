from glob import glob
from setuptools import setup

setup(name='ec_serial_bridge', version='0.1.0', packages=['ec_serial_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/ec_serial_bridge']),
                  ('share/ec_serial_bridge', ['package.xml']),
                  ('share/ec_serial_bridge/launch', glob('launch/*.launch.py')),
                  ('share/ec_serial_bridge/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], entry_points={'console_scripts': [
          'ec_serial_bridge = ec_serial_bridge.node:main']})
