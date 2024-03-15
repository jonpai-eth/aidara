"""Setup for the hand_position package."""

from setuptools import find_packages, setup

package_name = "hand_position"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aidara",
    maintainer_email="luedkej@ethz.ch",
    entry_points={
        "console_scripts": [
            "hand_position = hand_position.hand_position:main",
        ],
    },
)
