"""Setup for the Stäubli controller."""
from setuptools import find_packages, setup

package_name = "staubli_controller"

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
    maintainer="pavi",
    maintainer_email="pramanathan@student.ethz.ch",
    description="Controller to communicate with the tx-60 robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "controller = staubli_controller.staubli_controller:main",
            "gripper = staubli_controller.staubli_gripper:main",
        ],
    },
)
