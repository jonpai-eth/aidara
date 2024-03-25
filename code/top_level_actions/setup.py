"""Setup for the top_level_actions package."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "top_level_actions"

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
    maintainer="aidara",
    maintainer_email="jonpai@ethz.ch",
    description="Top-level actions for the LLM planner.",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
