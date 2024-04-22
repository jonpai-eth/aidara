"""Setup for the geometric_grasp package."""

from setuptools import find_packages, setup

package_name = "geometric_grasp"

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
    maintainer="felix",
    maintainer_email="fehegg@student.ethz.ch",
    description="Geometric grasp generation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "geometric_grasp_server = geometric_grasp.geometric_grasp_server:main",
            "dummy_client = geometric_grasp.dummy_client:main",
        ],
    },
)
