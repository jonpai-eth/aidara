"""Utility functions for working with ZED cameras and its images."""

import base64
import pathlib
from collections.abc import Iterable, Iterator

import cv2
import numpy as np
import numpy.typing as npt
import PIL.Image
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from aidara_common.singleton import Singleton
from aidara_common.topic_utils import get_latest_msg_from_topic


class CvBridgeSingleton(CvBridge, metaclass=Singleton):
    """Singleton Cv Bridge."""


def get_img_from_zed(
    node: Node,
    callback_group: ReentrantCallbackGroup,
    camera_name: str = "zed",
    timeout: Duration | None = None,
) -> Image:
    """
    Get image of current scene from ZED camera.

    Listens to `/<camera_name>/zed_node/rgb/image_rect_color`.

    Args:
        node: ROS2 node.
        callback_group: Reentrant callback group for the subscriber.

    Returns:
        Latest image from ZED camera.
    """
    timeout = timeout or Duration(seconds=5)

    return get_latest_msg_from_topic(
        node,
        f"/{camera_name}/zed_node/rgb/image_rect_color",
        Image,
        callback_group,
        timeout,
    )


def get_all_images(
    node: Node,
    callback_group: ReentrantCallbackGroup,
    camera_names: Iterable[str] = ["zed_left", "zed_top", "zed_right"],
) -> Iterator[Image]:
    """Return a tuple of all available images."""
    yield from (
        get_latest_msg_from_topic(
            node,
            f"/{camera_name}/zed_node/rgb/image_rect_color",
            Image,
            callback_group,
        )
        for camera_name in camera_names
    )


def get_zed_intrinsics(
    node: Node,
    callback_group: ReentrantCallbackGroup,
    camera_name: str = "zed",
) -> tuple[np.ndarray, np.ndarray]:
    """
    Get camera intrinsics from ZED camera.

    Args:
        node: ROS2 node.
        callback_group: Reentrant callback group for the subscriber.
        camera_name: Name of the camera as specified during startup

    Returns:
        Tuple:
            3x3 ndarray cam_mtx:   [[fx, 0, cx],
                                    [0, fy, cy],
                                    [0, 0, 1]]
            1x5 ndarray dist_coeffs: [k1, k2, p1, p2, k3]
    """
    # Get CameraInfo message from topic
    cam_info = get_latest_msg_from_topic(
        node,
        f"/{camera_name}/zed_node/rgb/camera_info",
        CameraInfo,
        callback_group,
    )

    # Extract intrinsic parameters from CameraInfo message
    fx, _, cx, _, fy, cy, *_ = iter(cam_info.k)
    k1, k2, p1, p2, k3, *_ = iter(cam_info.d)

    # Create the camera matrix mtx
    mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    # Create a vector containing the known distortion coefficients
    dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

    return mtx, dist_coeffs


def imgmsg_to_base64(image: Image) -> str:
    """Convert Image msg to b64 encoding."""
    cv2_img = CvBridgeSingleton().imgmsg_to_cv2(image, desired_encoding="bgr8")
    _, jpg_img = cv2.imencode(".jpg", cv2_img)
    return base64.b64encode(jpg_img).decode("utf-8")


def imgmsg_to_grayscale(image: Image) -> npt.NDArray:
    """Convert Image msg to grayscale."""
    return CvBridgeSingleton().imgmsg_to_cv2(image, "mono8")


def imgmsg_to_pil(image: Image) -> PIL.Image.Image:
    """Convert Image msg to PIL."""
    cv2_img = CvBridgeSingleton().imgmsg_to_cv2(image, desired_encoding="bgr8")
    cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
    return PIL.Image.fromarray(cv2_img)


def imgmsg_to_rgb(image: Image) -> npt.NDArray:
    """Convert Image msg to rgb."""
    return CvBridgeSingleton().imgmsg_to_cv2(image, "rgb8")


def jpg_to_base64(image: pathlib.Path) -> str:
    """Read a .jpg file and return the b64 encoding."""
    with image.open("rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def mono8_to_imgmsg(np_img: np.ndarray) -> Image:
    """Convert a mono8 2D np array to an Image msg."""
    np_img = np_img.astype(np.uint8)
    return CvBridgeSingleton().cv2_to_imgmsg(np_img, encoding="mono8")
