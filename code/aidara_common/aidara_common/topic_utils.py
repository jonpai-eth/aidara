"""Utility functions for working with ROS 2 topics."""

from typing import TypeVar

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image

AnyMessage = TypeVar("AnyMessage")


def get_latest_msg_from_topic(
    node: Node,
    topic: str,
    msg_type: type[AnyMessage],
    callback_group: ReentrantCallbackGroup,
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
    while msg is None:
        rate.sleep()

    node.get_logger().info(f"Received message on '{topic}'.")

    return msg
