"""Node providing tf functionality via service interfaces."""

import rclpy
from aidara_msgs.srv import Tf2DoTransform, Tf2GetTransform
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration, Time
from tf2_geometry_msgs import (
    PoseStamped,  # noqa: F401. Required for buffer.transform.
)
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Tf2Server(Node):
    """Node providing tf functionality via service interfaces."""

    def __init__(self) -> None:
        """Initialize Tf2Server."""
        super().__init__("tf2_server")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._get_transform_srv = self.create_service(
            Tf2GetTransform,
            "~/get_transform",
            self._get_transform_cb,
        )

        self._do_transform_srv = self.create_service(
            Tf2DoTransform,
            "~/do_transform",
            self._do_transform_cb,
        )

    def _get_transform_cb(
        self,
        request: Tf2GetTransform.Request,
        response: Tf2GetTransform.Response,
    ) -> Tf2GetTransform.Response:
        """Get transformation from source_frame to target_frame."""
        response.success = False

        target_frame = request.target_frame
        source_frame = request.source_frame

        try:
            res = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        except TransformException:
            return response

        response.tf = res
        response.success = True
        return response

    def _do_transform_cb(
        self,
        request: Tf2DoTransform.Request,
        response: Tf2DoTransform.Response,
    ) -> Tf2DoTransform.Response:
        """Transform pose from pose frame to target frame."""
        response.success = False

        target_frame = request.target_frame
        source_pose = request.source_pose

        try:
            res = self._tf_buffer.transform(
                source_pose,
                target_frame,
                timeout=Duration(seconds=1),
            )
        except TransformException:
            return response

        response.pose = res
        response.success = True
        return response


def main(args: list[str] | None = None) -> None:
    """Initialize tf2_server."""
    rclpy.init(args=args)

    try:
        node = Tf2Server()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
