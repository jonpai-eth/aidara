"""Launch a camera by its serial number and calibrate it."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch a zed camera by serial number and calibrate it."""
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="zed",
        description="The name of the camera.",
    )
    serial_number_arg = DeclareLaunchArgument(
        "serial_number",
        description="The serial number of the camera.",
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

    camera_name = LaunchConfiguration("camera_name")
    chessboard_width = LaunchConfiguration("chessboard_width")
    chessboard_height = LaunchConfiguration("chessboard_height")
    square_size = LaunchConfiguration("square_size")

    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("zed_wrapper"),
                        "launch",
                        "zed_camera.launch.py",
                    ],
                ),
            ],
        ),
        launch_arguments={
            ("camera_model", "zed2i"),
            ("camera_name", camera_name),
            ("serial_number", LaunchConfiguration("serial_number")),
            ("publish_tf", "false"),
            ("publish_map_tf", "false"),
        },
    )

    def calibrate_camera(context: LaunchContext) -> list[ExecuteProcess]:
        return [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/chessboard_calibration/calibrate_camera",
                    "aidara_msgs/srv/CalibrateCamera",
                    f"{{camera_name: {camera_name.perform(context)}"
                    f", chessboard_width: {chessboard_width.perform(context)}"
                    f", chessboard_height: {chessboard_height.perform(context)}"
                    f" , square_size: {square_size.perform(context)}"
                    "}",
                ],
                output="screen",
            ),
        ]

    return LaunchDescription(
        [
            chessboard_width_arg,
            chessboard_height_arg,
            square_size_arg,
            camera_name_arg,
            serial_number_arg,
            launch_camera,
            OpaqueFunction(function=calibrate_camera),
        ],
    )
