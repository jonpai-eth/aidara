"""ROS service that opens ZED camera, grabs frame and returns transformation."""

import traceback

import cv2
import numpy as np
import rclpy
import ros2_numpy as rnp
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from aidara_common.image_utils import (
    get_img_from_zed,
    get_zed_intrinsics,
    imgmsg_to_grayscale,
)
from aidara_common.tf_utils import TfMixin
from aidara_msgs.srv import CalibrateCamera


class ChessBoardCornersNotFoundError(Exception):
    """Raise when chessboard corners are not found."""


def _get_chessboard_corners(
    image_gray: np.ndarray,
    chessboard_width: int,
    chessboard_height: int,
    square_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate chessboard world coordinates."""
    objp = np.zeros((chessboard_width * chessboard_height, 3), np.float32)
    objp[:, :2] = (
        np.mgrid[0:chessboard_width, 0:chessboard_height].T.reshape(-1, 2) * square_size
    )

    ret, corners = cv2.findChessboardCorners(
        image_gray,
        (chessboard_width, chessboard_height),
        None,
    )

    if not ret:
        raise ChessBoardCornersNotFoundError

    # https://theailearner.com/tag/cv2-cornersubpix/
    corners_refined = cv2.cornerSubPix(
        image_gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    return corners_refined, objp


def _get_static_transform(
    corners: np.ndarray,
    object_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Calculate the homogeneous transform from camera to chessboard frame."""
    _, rvecs, tvecs = cv2.solvePnP(
        object_points,
        corners,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    # Construct the transformation matrix
    rot_mat, _ = cv2.Rodrigues(rvecs)

    return tvecs.squeeze(), rot_mat


def _make_zed_to_opencv_transform(cam_frame_name: str) -> TransformStamped:
    """Create TransformStamped msg from zed to opencv camera frames."""
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = cam_frame_name
    tf_msg.child_frame_id = cam_frame_name + "_opencv"

    # Rotate 90 degrees around z-axis
    rotation_msg = rnp.msgify(Quaternion, np.array([0.0, -0.70710678, 0.70710678, 0.0]))
    tf_msg.transform.rotation = rotation_msg

    return tf_msg


class ChessboardCalibration(Node, TfMixin):
    """Node that calculates the homogeneous transform from cam to chessboard frame."""

    def __init__(self) -> None:
        """Create ChessboardCalibration node."""
        Node.__init__(self, "chessboard_calibration")
        TfMixin.__init__(self)

        self._calib_cb_group = ReentrantCallbackGroup()
        self._calibrate_camera_srv = self.create_service(
            CalibrateCamera,
            "~/calibrate_camera",
            self._calibrate_camera_cb,
            callback_group=self._calib_cb_group,
        )

        self._cb_group = ReentrantCallbackGroup()

    def _calibrate_camera_cb(
        self,
        request: CalibrateCamera.Request,
        response: CalibrateCamera.Response,
    ) -> CalibrateCamera.Response:
        """Publish cam to chessboard and ZED to OpenCV camera frame transforms.

        Names of frames:
            ZED frame: As passed into service with camera_name argument
            OpenCV frame: camera_name + "_opencv"
        """
        response.success = False

        camera_name = request.camera_name

        img_msg = get_img_from_zed(self, self._cb_group, camera_name)
        camera_zed_frame = img_msg.header.frame_id

        cv_image_gray = imgmsg_to_grayscale(img_msg)

        camera_matrix, dist_coeffs = get_zed_intrinsics(
            self,
            self._cb_group,
            camera_name,
        )

        try:
            corners, object_points = _get_chessboard_corners(
                cv_image_gray,
                request.chessboard_width,
                request.chessboard_height,
                request.square_size,
            )
        except ChessBoardCornersNotFoundError:
            msg = (
                "Chessboard corners not found.\n"
                "Make sure chessboard is visible to camera and chessboard "
                "parameters are correct. Error:"
            )
            # Want to use ROS logger which does not provide exception logging.
            # Log the traceback manually instead.
            self.get_logger().error(msg)
            self.get_logger().info(traceback.format_exc())

            return response

        translation, rotation = _get_static_transform(
            corners,
            object_points,
            camera_matrix,
            dist_coeffs,
        )

        camera_chessboard_frame = camera_name + "_chessboard"

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = camera_zed_frame + "_opencv"
        transform_stamped.child_frame_id = camera_chessboard_frame

        transform_stamped.transform.translation = rnp.msgify(
            Vector3,
            translation,
        )
        transform_stamped.transform.rotation = rnp.msgify(
            Quaternion,
            Rotation.from_matrix(rotation).as_quat(canonical=False),
        )

        self.publish_static_transform(_make_zed_to_opencv_transform(camera_zed_frame))
        self.publish_static_transform(transform_stamped)

        cam_link_frame_name = f"{camera_name}_camera_link"
        tf_cam_chessboard_to_cam_link = self.get_tf(
            cam_link_frame_name,
            camera_chessboard_frame,
        )
        tf_cam_chessboard_to_cam_link.header.frame_id = "chessboard"
        tf_cam_chessboard_to_cam_link.child_frame_id = cam_link_frame_name

        self.publish_static_transform(tf_cam_chessboard_to_cam_link)

        self.get_logger().info("Calibrated camera successfully.")

        response.success = True
        return response


def main(args: list[str] | None = None) -> None:
    """Spin the ChessboardCalibration node."""
    rclpy.init(args=args)

    try:
        node = ChessboardCalibration()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
