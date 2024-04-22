"""Setup for tf2_service package."""

from setuptools import find_packages, setup

package_name = "tf2_service"

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
    maintainer_email="rbesenfel@student.ethz.ch",
    description="Service interface for tf.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tf2_service = tf2_service.tf2_server:main",
        ],
    },
)
