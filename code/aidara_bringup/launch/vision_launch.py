"""Launch file for calibration node and camera launch files."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    NotEqualsSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MISSING = "MISSING"


def generate_launch_description() -> LaunchDescription:
    """Launch cameras and calibration node."""
    zed_top_sn_arg = DeclareLaunchArgument(
        "zed_top_sn",
        default_value=MISSING,
        description="The serial number of the top camera.",
    )
    zed_right_sn_arg = DeclareLaunchArgument(
        "zed_right_sn",
        default_value=MISSING,
        description="The serial number of the right camera.",
    )
    zed_left_sn_arg = DeclareLaunchArgument(
        "zed_left_sn",
        default_value=MISSING,
        description="The serial number of the left camera.",
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

    calibration_node = Node(
        package="camera_calibration",
        executable="chessboard_calibration",
        name="chessboard_calibration",
        output="screen",
    )

    tf2_service = Node(
        package="tf2_service",
        executable="tf2_service",
    )

    launch_cameras = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("aidara_bringup"),
                            "launch",
                            "camera_launch.py",
                        ],
                    ),
                ],
            ),
            launch_arguments=[
                ("camera_name", camera_name),
                ("serial_number", serial_number),
                ("chessboard_width", LaunchConfiguration("chessboard_width")),
                ("chessboard_height", LaunchConfiguration("chessboard_height")),
                ("square_size", LaunchConfiguration("square_size")),
            ],
            condition=IfCondition(
                NotEqualsSubstitution(serial_number, MISSING),
            ),
        )
        for camera_name, serial_number in [
            ("zed_top", LaunchConfiguration("zed_top_sn")),
            ("zed_right", LaunchConfiguration("zed_right_sn")),
            ("zed_left", LaunchConfiguration("zed_left_sn")),
        ]
    ]

    return LaunchDescription(
        [
            zed_top_sn_arg,
            zed_right_sn_arg,
            zed_left_sn_arg,
            chessboard_width_arg,
            chessboard_height_arg,
            square_size_arg,
            Node(
                package="rerun_manager",
                executable="rerun_manager",
            ),
            calibration_node,
            tf2_service,
            *launch_cameras,
        ],
    )
