"""Utility functions for working with ZED cameras and its images."""
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


def get_img_from_zed(node: Node, callback_group: ReentrantCallbackGroup) -> Image:
    """
    Get image of current scene from ZED camera.

    From topic /zed/zed_node/left/image_rect_color.

    Args:
        node: ROS2 node.
        callback_group: Reentrant callback group for the subscriber.

    Returns:
        Latest image from ZED camera
    """
    video_qos = QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    img = None

    def update_img(msg: Image) -> None:
        nonlocal img
        img = msg
        node.logger.info("Received image.")
        node.destroy_subscription(subs)

    subs = node.create_subscription(
        Image,
        "/zed/zed_node/left/image_rect_color",
        update_img,
        qos_profile=video_qos,
        callback_group=callback_group,
    )

    # Sleep until image can be retrieved to ensure image is there
    node.logger.info("Waiting for image...")
    rate = node.create_rate(10, node.get_clock())
    while img is None:
        rate.sleep()

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
    intrinsics_qos = QoSProfile(
        depth=5,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    dist_coeffs = None
    mtx = None

    def intrinsics_callback(msg: CameraInfo) -> None:
        nonlocal dist_coeffs, mtx

        # Extract intrinsic parameters from CameraInfo message
        fx, _, cx, _, fy, cy, *_ = iter(msg.k)
        k1, k2, p1, p2, k3, *_ = iter(msg.d)

        # Create a vector containing the known distortion coefficients
        dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

        # Create the camera matrix mtx
        mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

        node.logger.info("Received intrinsics.")
        node.destroy_subscription(subs)

    subs = node.create_subscription(
        CameraInfo,
        "/zed/zed_node/left/camera_info",
        intrinsics_callback,
        callback_group=callback_group,
        qos_profile=intrinsics_qos,
    )

    # Sleep until intrinsics can be retrieved
    node.logger.info("Waiting for intrinsics...")
    rate = node.create_rate(10, node.get_clock())
    while mtx is None:
        rate.sleep()

    return mtx, dist_coeffs
