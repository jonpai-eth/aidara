"""Launch file for a variety of aidara_packages.

llm_planner, vision_launch, hand_position, text_to_speech
and geometric_grasp.
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from llm_planning.llm_interfaces import VisionMode


def generate_launch_description() -> LaunchDescription:
    """
    Launch a variety of aidara packages.

    Launches llm_planner, vision_launch, hand_position,
    tf2_service, text_to_speech, geometric_grasp.
    """
    llm = DeclareLaunchArgument(
        "llm",
        default_value="gpt-4",
        description="The LLM the planner uses.",
        choices=["gemini", "gpt-4"],
    )
    prompt_version = DeclareLaunchArgument(
        "prompt_version",
        default_value="playground",
        description="The prompt version passed to the llm.",
        choices=["playground", "minigame"],
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
                    "--prompt-version",
                    LaunchConfiguration("prompt_version"),
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
        prompt_version,
        dry_run,
        examples_vision_mode,
        request_vision_mode,
        robot,
        hand_tracking_camera,
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
                ("robot", LaunchConfiguration("robot")),
            },
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
    ]
    return LaunchDescription(description)
