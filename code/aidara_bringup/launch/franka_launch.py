"""Launch file for the franka trajectory planner, speech_to_text, static transforms."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch planner, speech_to_text and static transforms."""
    return LaunchDescription(
        [
            Node(
                package="trajectory_planning",
                executable="planner",
                emulate_tty=True,
                arguments=["--robot", "franka"],
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
