from setuptools import setup

setup(name='robot_geometry', version='0.1.0', packages=['robot_geometry'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robot_geometry']),
                  ('share/robot_geometry', ['package.xml'])],
      install_requires=['setuptools'],
      entry_points={'console_scripts': ['load_envelope = robot_geometry.load_envelope:main']})
