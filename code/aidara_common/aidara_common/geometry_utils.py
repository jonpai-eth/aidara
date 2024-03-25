"""Utility functions for geometric things."""

import functools

from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Vector3


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
