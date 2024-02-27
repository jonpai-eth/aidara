"""Setup.py file of the camera_calibration ROS package."""

from setuptools import find_packages, setup

package_name = "camera_calibration"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Artur von Ruffer",
    maintainer_email="avonruffer@ethz.ch",
    description="Publishes transformation from camera to checkerboard frame",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibration_node = camera_calibration.calibration_node:main",
        ],
    },
)
