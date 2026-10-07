from setuptools import find_packages, setup

setup(
    name="robot_vision",
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    python_requires=">=3.10",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robot_vision"]),
        ("share/robot_vision", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wuyimingai2004-cloud",
    maintainer_email="wuyimingai2004-cloud@users.noreply.github.com",
    description="Camera QR decoding with a persistent result window",
    license="License not specified",
    entry_points={"console_scripts": ["qr_popup = robot_vision.qr_popup:main"]},
)
