"""Launch file for a variety of aidara_packages.

llm_planner, vision_launch, hand_position, tf2_service, text_to_speech
and geometric_grasp.
"""

import uuid

import rerun
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from llm_planning.llm_interfaces import VisionMode


def generate_launch_description() -> LaunchDescription:
    """
    Launch a variety of aidara packages.

    Launches llm_planner, vision_launch, hand_position,
    tf2_service, text_to_speech, geometric_grasp.
    """
    rerun.spawn()
    llm = DeclareLaunchArgument(
        "llm",
        default_value="gpt-4",
        description="The LLM the planner uses.",
        choices=["gemini", "gpt-4"],
    )
    dry_run = DeclareLaunchArgument(
        "is_dry_run",
        default_value="False",
        choices=["True", "False"],
        description="If true, do not execute the generated code.",
    )
    examples_vision_mode = DeclareLaunchArgument(
        "examples_vision_mode",
        default_value=str(VisionMode.NONE),
        choices=VisionMode.get_valid_options(),
        description="The vision mode for examples.",
    )
    request_vision_mode = DeclareLaunchArgument(
        "request_vision_mode",
        default_value=str(VisionMode.TOP),
        choices=VisionMode.get_valid_options(),
        description="The vision mode for examples.",
    )
    robot = DeclareLaunchArgument(
        "robot",
        default_value="franka",
        choices=["franka", "staubli"],
        description="The vision mode for examples.",
    )
    hand_tracking_camera = DeclareLaunchArgument(
        "hand_tracking_camera_name",
        default_value="zed",
        description="The name of the ZED camera.",
    )
    chessboard_width_arg = DeclareLaunchArgument(
        "chessboard_width",
        description=(
            "number of inner corners (one less than the number of squares)"
            " in the long dimension."
        ),
    )
    chessboard_height_arg = DeclareLaunchArgument(
        "chessboard_height",
        description=(
            "number of inner corners (one less than the number of squares)"
            " in the short dimension."
        ),
    )
    square_size_arg = DeclareLaunchArgument(
        "square_size",
        description="side length of a square in m",
    )

    def launch_llm_planner(ctx: LaunchContext) -> list[Node]:
        return [
            Node(
                package="llm_planning",
                executable="llm_planner",
                arguments=[
                    *(
                        ["--dry-run"]
                        if LaunchConfiguration("is_dry_run").perform(ctx) == "True"
                        else []
                    ),
                    "--llm",
                    LaunchConfiguration("llm"),
                    "--examples-vision-mode",
                    LaunchConfiguration("examples_vision_mode"),
                    "--request-vision-mode",
                    LaunchConfiguration("request_vision_mode"),
                    "--robot",
                    LaunchConfiguration("robot"),
                ],
            ),
        ]

    description = [
        llm,
        dry_run,
        examples_vision_mode,
        request_vision_mode,
        robot,
        hand_tracking_camera,
        chessboard_width_arg,
        chessboard_height_arg,
        square_size_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("aidara_bringup"),
                        "launch",
                        "vision_launch.py",
                    ],
                ),
            ),
            launch_arguments=[
                ("chessboard_width", LaunchConfiguration("chessboard_width")),
                ("chessboard_height", LaunchConfiguration("chessboard_height")),
                ("square_size", LaunchConfiguration("square_size")),
            ],
        ),
        Node(
            package="text_to_speech",
            executable="text_to_speech",
        ),
        Node(
            package="tf2_service",
            executable="tf2_service",
        ),
        Node(
            package="geometric_grasp",
            executable="geometric_grasp_server",
        ),
        Node(
            package="hand_position",
            executable="hand_position",
            arguments=[
                "--zed-camera-name",
                LaunchConfiguration("hand_tracking_camera_name"),
            ],
        ),
        OpaqueFunction(function=launch_llm_planner),
        SetParameter(name="/rr_recording_id", value=str(uuid.uuid4())),
    ]
    return LaunchDescription(description)
