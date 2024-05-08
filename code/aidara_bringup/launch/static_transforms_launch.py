"""Creates all the needed static transforms of the aidara project."""

import pathlib

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Launch all static transforms."""
    robot_args = DeclareLaunchArgument(
        "robot",
        choices=["franka", "staubli"],
    )

    def declare_static_transforms(context: LaunchContext) -> list[ExecuteProcess]:
        share_dir = get_package_share_directory("aidara_bringup")
        static_transforms_dir = pathlib.Path(share_dir) / "static_transforms"
        robot = LaunchConfiguration("robot").perform(context)
        with (
            static_transforms_dir
            / f"{robot}_transforms.yaml"
        ).open() as f:
            transform_collection = yaml.safe_load(f)
        return [
            ExecuteProcess(
                prefix=(
                    "ros2 run tf2_ros static_transform_publisher"
                    f" --x {t['x']}"
                    f" --y {t['y']}"
                    f" --z {t['z']}"
                    f" --qx {t['qx']}"
                    f" --qy {t['qy']}"
                    f" --qz {t['qz']}"
                    f" --qw {t['qw']}"
                    f" --frame-id {t['parent']}"
                    f" --child-frame-id {t['child']}"
                ),
                cmd="",
                emulate_tty=True,
            )
            for t in transform_collection
        ]

    return LaunchDescription(
        [
            robot_args,
            OpaqueFunction(function=declare_static_transforms),
        ],
    )
