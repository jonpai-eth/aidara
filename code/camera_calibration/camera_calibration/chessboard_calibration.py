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
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from aidara_common.image_utils import (
    get_img_from_zed,
    get_zed_intrinsics,
    imgmsg_to_grayscale,
)
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

    return tvecs, rot_mat


class ChessboardCalibration(Node):
    """Node that calculates the homogeneous transform from cam to chessboard frame."""

    def __init__(self) -> None:
        """Create ChessboardCalibration node."""
        super().__init__("chessboard_calibration")
        self._calibrate_camera_srv = self.create_service(
            CalibrateCamera,
            "~/calibrate_camera",
            self._calibrate_camera_cb,
        )
        self._br = StaticTransformBroadcaster(self)
        self._logger = self.get_logger()
        self._cb_group = ReentrantCallbackGroup()

    def _calibrate_camera_cb(
        self,
        request: CalibrateCamera.Request,
        response: CalibrateCamera.Response,
    ) -> Trigger.Response:
        """Capture frame, calculate transform."""
        response.success = False

        img_msg = get_img_from_zed(self, self._cb_group)

        cv_image_gray = imgmsg_to_grayscale(img_msg)

        camera_matrix, dist_coeffs = get_zed_intrinsics(self, self._cb_group)

        try:
            corners, object_points = _get_chessboard_corners(
                cv_image_gray,
                request.chessboard_width,
                request.chessboard_height,
                request.square_size,
            )
        except ChessBoardCornersNotFoundError as e:
            msg = (
                "Chessboard corners not found."
                f" Make sure chessboard is visible to camera. Error: {e}"
            )
            # Want to use ROS logger which does not provide exception logging.
            # Log the traceback manually instead.
            self._logger.error(msg)  # noqa: TRY400
            self._logger.debug(traceback.format_exc())

            return response

        translation, rotation = _get_static_transform(
            corners,
            object_points,
            camera_matrix,
            dist_coeffs,
        )

        # Calculate the inverse rotation matrix
        inverse_rotation = np.linalg.inv(rotation)

        # Calculate the inverse translation vector
        inverse_translation = -np.dot(
            inverse_rotation,
            translation,
        ).squeeze()

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "chessboard_frame"
        transform_stamped.child_frame_id = "zed_camera_frame"

        transform_stamped.transform.translation = rnp.msgify(
            Vector3,
            inverse_translation,
        )
        transform_stamped.transform.rotation = rnp.msgify(
            Quaternion,
            Rotation.from_matrix(inverse_rotation).as_quat(),
        )

        # Broadcast static transform
        self._br.sendTransform(transform_stamped)
        self._logger.info("Calibrated camera successfully.")

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
