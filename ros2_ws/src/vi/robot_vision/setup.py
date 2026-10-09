from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'scripts'),
            glob('scripts/*'),
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Group 5 VI',

    description='On-demand QR vision module for DASE7503',

    license='TODO',

    entry_points={
        'console_scripts': [
            'qr_detector_node = robot_vision.qr_detector_node:main',
        ],
    },
)
