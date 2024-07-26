"""Node providing tf functionality via service interfaces."""

from typing import TypeVar

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration, Time
from tf2_geometry_msgs import (
    PointStamped,  # noqa: F401. Required for buffer.transform.
    PoseStamped,  # noqa: F401. Required for buffer.transform.
)
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from aidara_msgs.srv import Tf2GetTransform, Tf2TransformPoint, Tf2TransformPose

TransformRequest = TypeVar(
    "TransformRequest",
    Tf2TransformPoint.Request,
    Tf2TransformPose.Request,
)
TransformResponse = TypeVar(
    "TransformResponse",
    Tf2TransformPoint.Response,
    Tf2TransformPose.Response,
)


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
            callback_group=ReentrantCallbackGroup(),
        )

        self._transform_pose_srv = self.create_service(
            Tf2TransformPose,
            "~/transform_pose",
            self._do_transform_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self._transform_point_srv = self.create_service(
            Tf2TransformPoint,
            "~/transform_point",
            self._do_transform_cb,
            callback_group=ReentrantCallbackGroup(),
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

        for _ in range(3):
            try:
                res = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    Duration(seconds=3),
                )
            except TransformException:
                continue

            response.tf = res
            response.success = True
            return response

        return response

    def _do_transform_cb(
        self,
        request: TransformRequest,
        response: TransformResponse,
    ) -> TransformResponse:
        """Transform transformable from original frame to target frame."""
        response.success = False

        target_frame = request.target_frame
        source = request.source
        source.header.stamp = self.get_clock().now().to_msg()

        for _ in range(3):
            try:
                res = self._tf_buffer.transform(
                    source,
                    target_frame,
                    timeout=Duration(seconds=3),
                )
            except TransformException as e:
                msg = f"Error in do_transform_cb: {e}"
                self.get_logger().error(msg)
                continue

            response.result = res
            response.success = True
            return response

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
