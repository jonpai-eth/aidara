"""Utilities for using and converting geometric objects."""

import functools

from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    TransformStamped,
    Vector3,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from aidara_msgs.srv import (
    Tf2GetTransform,
    Tf2TransformPoint,
    Tf2TransformPose,
)


class TfMixin:
    """Mixin class for getting transforms and transforming poses."""

    def __init__(self) -> None:
        """Initialize TfMixin."""
        if not isinstance(self, Node):
            msg = "TfMixin can only be used for classes that are Nodes."
            raise TypeError(msg)

        self._tf_client_get_transform = self.create_client(
            Tf2GetTransform,
            "/tf2_server/get_transform",
            callback_group=ReentrantCallbackGroup(),
        )
        self._tf_client_make_transform = self.create_client(
            Tf2TransformPose,
            "/tf2_server/transform_pose",
            callback_group=ReentrantCallbackGroup(),
        )

    def _get_tf(self, source_frame: str, target_frame: str) -> TransformStamped:
        """Get the transform from source to target frame."""
        request = Tf2GetTransform.Request()
        request.target_frame = target_frame
        request.source_frame = source_frame
        result = self._tf_client_get_transform.call(request)

        if not result.success:
            msg = (
                f"Error while getting the transform between '{source_frame}' and"
                f" '{target_frame}'. Consider checking if the transform exists."
            )
            raise RuntimeError(msg)

        return result.tf

    def _do_tf(
        self,
        source: PointStamped | PoseStamped,
        target_frame: str,
    ) -> TransformStamped:
        """Transform a point or pose to the target frame."""
        if isinstance(source, PointStamped):
            request = Tf2TransformPoint.Request()
        else:
            request = Tf2TransformPose.Request()

        request.target_frame = target_frame
        request.source = source

        result = self._tf_client_get_transform.call(request)

        if not result.success:
            msg = (
                f"Error while transforming from '{source.header.frame_id}' to"
                f" '{target_frame}'. Consider checking if the target frame exists."
            )
            raise RuntimeError(msg)

        return result.tf


@functools.singledispatch
def convert(source: object) -> object:
    """Convert between geometric types."""
    msg = f"No conversion defined for '{source}'."
    raise NotImplementedError(msg)


@convert.register
def _(source: TransformStamped) -> PoseStamped:
    """Convert TransformStamped to PoseStamped."""
    res = PoseStamped()

    res.header = source.header
    res.pose.position = convert(source.transform.translation)
    res.pose.orientation = source.transform.rotation

    return res


@convert.register
def _(vector: Vector3) -> Point:
    """Convert Vector3 to Point."""
    res = Point()

    res.x = vector.x
    res.y = vector.y
    res.z = vector.z

    return res
