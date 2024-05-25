"""Setup for the rerun_manager package."""

from setuptools import find_packages, setup

package_name = "rerun_manager"

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
    maintainer_email="jonpai@ethz.ch",
    description="rerun manager.",
    license="MIT",
    entry_points={
        "console_scripts": ["rerun_manager = rerun_manager.rerun_manager:main"],
    },
)
