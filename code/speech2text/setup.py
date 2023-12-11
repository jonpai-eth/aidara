"""Setup for the speech2text package."""
from setuptools import find_packages, setup

package_name = "speech2text"

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
    maintainer="Joel Kaufmann",
    maintainer_email="jkaufmann@ethz.ch",
    description="Speech2Text ROS 2 Node.",
    license="MIT",
    entry_points={
        "console_scripts": ["speech2text = speech2text.speech2text:main"],
    },
)
