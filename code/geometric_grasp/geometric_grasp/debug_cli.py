"""Dummy client that calls the geometric_grasp_server to get a grasp position."""

import argparse
from typing import cast

import rclpy
from rclpy.node import Node

from aidara_common.async_utils import AsyncServiceCall
from aidara_msgs.srv import GeometricGrasp


class DummyClient(Node):
    """Dummy client that calls the GeometricGraspServer to get a grasp position."""

    def __init__(self) -> None:
        """Initialize client."""
        super().__init__("geometric_grasp_cli_client")
        self._client = self.create_client(
            GeometricGrasp,
            "/geometric_grasp",
        )

    def get_target_pose(self, object_name: str) -> None:
        """Send request to geometric_grasp_server."""
        request = GeometricGrasp.Request()
        request.object_description.data = object_name

        result = AsyncServiceCall.create(
            self,
            self._client,
            request,
        ).resolve_with_eh(n_retries=0)
        if not result.success:
            self.get_logger().error("The object could not be detected.")
            return

        self.get_logger().info(
            f"target_pose: {cast(GeometricGrasp.Result, result).pose}",
        )


def main(args: list[str] | None = None) -> None:
    """Run client to get a grasp position."""
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        prog="ros2 run geometric_grasp debug_cli",
        description="Geometric Grasp CLI",
    )
    parser.add_argument(
        "--object",
        type=str,
        help="What object to grasp.",
        default="bottle",
    )
    cli_args, _ = parser.parse_known_args()

    client = DummyClient()
    client.get_target_pose(cli_args.object)
    rclpy.shutdown()
