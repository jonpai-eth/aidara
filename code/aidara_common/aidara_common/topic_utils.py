"""Utility functions for working with ROS 2 topics."""

from typing import TypeVar

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image

AnyMessage = TypeVar("AnyMessage")


def get_latest_msg_from_topic(
    node: Node,
    topic: str,
    msg_type: type[AnyMessage],
    callback_group: ReentrantCallbackGroup,
    timeout: Duration | None = None,
) -> AnyMessage:
    """
    Return latest message from specified topic.

    Args:
        node: ROS2 node.
        topic: Topic that the desired message is published to.
        msg_type: Type of the desired message.
        callback_group: Reentrant callback group.

    Returns:
        Raw message.
    """
    timeout = timeout or Duration(seconds=5)
    msg = None

    def update_msg(incoming_msg: Image) -> None:
        nonlocal msg
        msg = incoming_msg
        node.destroy_subscription(subs)

    subs = node.create_subscription(
        msg_type,
        topic,
        update_msg,
        qos_profile=1,
        callback_group=callback_group,
    )

    node.get_logger().info(f"Waiting for message on '{topic}'.")

    # Sleep until msg can be retrieved.
    rate = node.create_rate(10, node.get_clock())
    for _ in range(10 * int(timeout.nanoseconds) // S_TO_NS):
        rate.sleep()
        if msg is not None:
            break
    else:
        msg = (
            f"Failed to receive any message on '{topic}' after"
            f" {timeout.nanoseconds / S_TO_NS} seconds."
        )
        raise RuntimeError(msg)

    node.get_logger().info(f"Received message on '{topic}'.")

    return msg
