"""Setup for the llm_planning package."""

from pathlib import Path

from setuptools import find_packages, setup

package_name = "llm_planning"

package_dir = Path(__file__).resolve().parent
share_dir = Path("share")
resource_dir = Path("resource")
example_prompts_dir = Path("example_prompts")

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
            str(share_dir / package_name / "example_prompts"),
            [str(example_prompts_dir / "examples.yaml")],
        ),
        (
            str(share_dir / package_name / "example_prompts" / "images"),
            [
                str(img.relative_to(package_dir))
                for img in (package_dir / example_prompts_dir / "images").glob("*.jpeg")
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aidara",
    maintainer_email="jonpai@ethz.ch",
    description="LLM planner interfaces.",
    license="MIT",
    entry_points={
        "console_scripts": ["llm_planner = llm_planning.llm_planner:main"],
    },
)
