"""Manager for the rerun visualization."""

import uuid

import rclpy
import rerun as rr
from std_msgs.msg import String


def main(args: list[str] | None = None) -> None:
    """Spawn rerun and declare the recording_id parameter."""
    rclpy.init(args=args)

    rerun_manager = rclpy.create_node("rerun_manager")

    recording_id = str(uuid.uuid4())
    rerun_manager.declare_parameter("recording_id", recording_id)

    rr.init(
        "aidara",
        recording_id=recording_id,
        default_enabled=True,
        spawn=True,
    )

    rerun_manager.create_subscription(
        String,
        "/speech_to_text_raw",
        lambda msg: rr.log("/user_request", rr.TextDocument(msg.data)),
        1,
    )

    rclpy.spin(rerun_manager)
