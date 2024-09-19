"""Setup for the gripper controller for the Franka robot."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "gripper_controller"

package_dir = Path(__file__).resolve().parent
share_dir = Path("share")
resource_dir = Path("resource")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            str(share_dir / "ament_index" / "resource_index" / "packages"),
            [str(resource_dir / package_name)],
        ),
        (str(share_dir / package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aidara",
    maintainer_email="pramanathan@ethz.ch",
    description="Gripper controller for the Franka research 3.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gripper = gripper_controller.gripper:main",
        ],
    },
)
