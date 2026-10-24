from glob import glob
from setuptools import setup

package_name = "robot_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DASE7503 VI Group",
    maintainer_email="vi@todo.invalid",
    description="Camera image processing and game QR-code recognition.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["qr_detector = robot_vision.qr_detector:main"],
    },
)
