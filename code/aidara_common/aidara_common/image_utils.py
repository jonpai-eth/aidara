"""Utility functions for working with ZED cameras and its images."""
from typing import TypeVar

import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def get_img_from_zed(node: Node, callback_group: ReentrantCallbackGroup) -> Image:
    """
    Get image of current scene from ZED camera.

    From topic /zed/zed_node/left/image_rect_color.

    Args:
        node: ROS2 node.
        callback_group: Reentrant callback group for the subscriber.

    Returns:
        Latest image from ZED camera.
    """
    node.logger.info("Waiting for image...")
    img = get_latest_msg_from_topic(
        node,
        "/zed/zed_node/left/image_rect_color",
        Image,
        callback_group,
    )
    node.logger.info("Received image.")
    return img


def get_zed_intrinsics(
    node: Node,
    callback_group: ReentrantCallbackGroup,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Get camera intrinsics from ZED camera.

    Args:
        node: ROS2 node.
        callback_group: Reentrant callback group for the subscriber.

    Returns:
        Tuple:
            3x3 ndarray cam_mtx:   [[fx, 0, cx],
                                    [0, fy, cy],
                                    [0, 0, 1]]
            1x5 ndarray dist_coeffs: [k1, k2, p1, p2, k3]
    """
    # Get CameraInfo message from topic
    node.logger.info("Waiting for intrinsics...")
    cam_info = get_latest_msg_from_topic(
        node,
        "/zed/zed_node/left/camera_info",
        CameraInfo,
        callback_group,
    )
    node.logger.info("Received intrinsics.")

    # Extract intrinsic parameters from CameraInfo message
    fx, _, cx, _, fy, cy, *_ = iter(cam_info.k)
    k1, k2, p1, p2, k3, *_ = iter(cam_info.d)

    # Create the camera matrix mtx
    mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    # Create a vector containing the known distortion coefficients
    dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

    return mtx, dist_coeffs


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

    # Sleep until image can be retrieved to ensure image is there
    rate = node.create_rate(10, node.get_clock())
    while msg is None:
        rate.sleep()

    return msg
