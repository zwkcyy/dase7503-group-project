from glob import glob
from setuptools import setup

package_name = "robot_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/maps", glob("maps/*.yaml") + glob("maps/*.pgm")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DASE7503 NAV Group",
    maintainer_email="nav@todo.invalid",
    description="Game field map and Nav2 configuration for the logistics robot.",
    license="Apache-2.0",
    entry_points={"console_scripts": []},
)
