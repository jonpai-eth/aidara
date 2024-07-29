"""Setup for the cuRobo based planner."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "trajectory_planning"

package_dir = Path(__file__).resolve().parent
share_dir = Path("share")
resource_dir = Path("resource")
config_dir = Path("config")


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
        (
            str(share_dir / package_name / "config"),
            [
                str(config.relative_to(package_dir))
                for config in (package_dir / "config").iterdir()
                if config.is_file()
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pavi",
    maintainer_email="pramanathan@ethz.ch",
    description="Find an optimized path for a goal pose.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "planner = trajectory_planning.planner:main",
            "debug_cli = trajectory_planning.debug_cli:main",
        ],
    },
)
