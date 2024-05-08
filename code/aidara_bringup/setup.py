"""Setup for the aidara_bringup package."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "aidara_bringup"
package_dir = Path(__file__).resolve().parent
static_transforms_dir = Path("static_transforms")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            str(Path("share") / package_name / "launch"),
            [
                str(file.relative_to(package_dir))
                for file in (package_dir / "launch").glob("*launch.py")
            ],
        ),
        (
            str(Path("share") / package_name / "static_transforms"),
            [
                str(static_transforms_dir / "franka_transforms.yaml"),
                str(static_transforms_dir / "staubli_transforms.yaml"),
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aidara",
    maintainer_email="pramanathan@student.ethz.ch",
    description="Package containing all launch files of the aidara project.",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
