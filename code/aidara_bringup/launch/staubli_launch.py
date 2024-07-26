"""Launch file for staubli trajectory planner, speech_to_text, static transforms."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch controller and planner."""
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("aidara_bringup"),
                            "launch",
                            "static_transforms_launch.py",
                        ],
                    ),
                ),
                launch_arguments={
                    ("robot", "staubli"),
                },
            ),
            Node(
                package="trajectory_planning",
                executable="planner_staubli",
                name="planner_staubli",
                emulate_tty=True,
            ),
            Node(
                package="speech_to_text",
                executable="speech_to_text",
            ),
            Node(
            package="text_to_speech",
            executable="text_to_speech",
        ),
        ],
    )
