"""Utilities for using and converting geometric objects."""

import collections
import functools
from typing import TypeVar, cast

from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    TransformStamped,
    Vector3,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from aidara_common.async_utils import AsyncServiceCall
from aidara_msgs.srv import (
    Tf2GetTransform,
    Tf2TransformPoint,
    Tf2TransformPose,
)

Transformable = TypeVar("Transformable", PointStamped, PoseStamped)


class TfMixin:
    """Mixin class for getting transforms and transforming poses."""

    def __init__(self) -> None:
        """Initialize TfMixin."""
        if not isinstance(self, Node):
            msg = "TfMixin can only be used for classes that are Nodes."
            raise TypeError(msg)

        self._get_transform_client = self.create_client(
            Tf2GetTransform,
            "/tf2_server/get_transform",
            callback_group=ReentrantCallbackGroup(),
        )
        self._transform_point_client = self.create_client(
            Tf2TransformPoint,
            "/tf2_server/transform_point",
            callback_group=ReentrantCallbackGroup(),
        )
        self._transform_pose_client = self.create_client(
            Tf2TransformPose,
            "/tf2_server/transform_pose",
            callback_group=ReentrantCallbackGroup(),
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcasters: dict[str, StaticTransformBroadcaster] = (
            collections.defaultdict(lambda: StaticTransformBroadcaster(self))
        )

    def get_tf(self, source_frame: str, target_frame: str) -> TransformStamped:
        """Get the transform from source to target frame."""
        request = Tf2GetTransform.Request()
        request.target_frame = target_frame
        request.source_frame = source_frame

        result = AsyncServiceCall.create(
            cast(Node, self),
            self._get_transform_client,
            request,
        ).resolve_with_eh(n_retries=3)

        if not result.success:
            msg = (
                f"Error while getting the transform between '{source_frame}' and"
                f" '{target_frame}'. Consider checking if the transform exists."
            )
            raise RuntimeError(msg)

        return result.tf

    def do_tf(self, source: Transformable, target_frame: str) -> Transformable:
        """Transform a point or pose to the target frame."""
        if isinstance(source, PointStamped):
            request = Tf2TransformPoint.Request()
            client = self._transform_point_client
        elif isinstance(source, PoseStamped):
            request = Tf2TransformPose.Request()
            client = self._transform_pose_client
        else:
            msg = "Unhandled object type in transform call."
            raise TypeError(msg)

        request.source = source
        request.target_frame = target_frame

        result = AsyncServiceCall.create(
            cast(Node, self),
            client,
            request,
        ).resolve_with_eh(n_retries=3)

        if not result.success:
            msg = (
                f"Error while transforming from '{source.header.frame_id}' to"
                f" '{target_frame}'. Consider checking if the target frame exists."
            )
            raise RuntimeError(msg)

        return result.result

    def publish_transform(
        self,
        transform: TransformStamped | list[TransformStamped],
    ) -> None:
        """Publish the transform on /tf."""
        self._tf_broadcaster.sendTransform(transform)

    def publish_static_transform(self, transform: TransformStamped) -> None:
        """Publish a static transform on /tf_static.

        This works for both publishing multiple different transforms and overwriting a
        previous static transform defining the same frame.
        """
        self._static_tf_broadcasters[transform.child_frame_id].sendTransform(transform)


@functools.singledispatch
def convert(source: object, *__args: object) -> object:
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
def _(source: PoseStamped, child_frame_id: str) -> TransformStamped:
    """Convert PoseStamped to TransformStamped."""
    res = TransformStamped()

    res.header = source.header
    res.child_frame_id = child_frame_id
    res.transform.translation = convert(source.pose.position)
    res.transform.rotation = source.pose.orientation

    return res


@convert.register
def _(point: Point) -> Vector3:
    """Convert Vector3 to Point."""
    res = Vector3()

    res.x = point.x
    res.y = point.y
    res.z = point.z

    return res


@convert.register
def _(vector: Vector3) -> Point:
    """Convert Vector3 to Point."""
    res = Point()

    res.x = vector.x
    res.y = vector.y
    res.z = vector.z

    return res
