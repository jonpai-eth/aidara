"""Setup for the text_to_speech package."""

from setuptools import find_packages, setup

package_name = "text_to_speech"

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
    maintainer="Timm Grigat",
    maintainer_email="tgrigat@ethz.ch",
    description="This package converts a text input into sound.",
    license="MIT",
    entry_points={
        "console_scripts": ["text_to_speech = text_to_speech.text_to_speech:main"],
    },
)
