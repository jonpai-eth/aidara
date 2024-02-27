"""ROS service that opens ZED camera, grabs frame and returns transformation."""
import traceback

import cv2
import numpy as np
import rclpy
import ros2_numpy as rnp
from aidara_common.image_utils import get_img_from_zed, get_zed_intrinsics
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from numpy.typing import NDArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

# Define the chessboard parameters
CHESSBOARD_SIZE = (8, 13)  # Number of inner corners on the chessboard
SQUARE_SIZE = 0.02  # Size of each square in meters


class ChessBoardCornersNotFoundError(Exception):
    """Raise when chessboard corners are not found."""


def get_chessboard_corners(image_gray: NDArray) -> tuple[NDArray, NDArray]:
    """Generate chessboard world coordinates."""
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = (
        np.mgrid[0 : CHESSBOARD_SIZE[0], 0 : CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
        * SQUARE_SIZE
    )

    ret, corners = cv2.findChessboardCorners(image_gray, CHESSBOARD_SIZE, None)

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


def get_static_transform(
    image_gray: NDArray,
    camera_matrix: NDArray,
    dist_coeffs: NDArray,
) -> NDArray:
    """Calculate the homogeneous transform from camera to chessboard frame."""
    corners2, objp = get_chessboard_corners(image_gray)

    # Solve the PnP problem
    _, rvecs, tvecs = cv2.solvePnP(
        objp,
        corners2,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    # Construct the transformation matrix
    rot_mat, _ = cv2.Rodrigues(rvecs)

    return tvecs, rot_mat


class CameraChessboardNode(Node):
    """Node that calculates the homogeneous transform from cam to chessboard frame."""

    def __init__(self) -> None:
        """Create CameraChessboardNode."""
        super().__init__("camera_chessboard_node")
        self.srv = self.create_service(
            Trigger,
            "calibrate_camera",
            self.calibrate_camera_cb,
        )
        self.br = StaticTransformBroadcaster(self)
        self.logger = self.get_logger()
        self.cb_group = ReentrantCallbackGroup()

    def calibrate_camera_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Capture frame, calculate transform."""
        # Get frame from ZED camera
        img_msg = get_img_from_zed(self, self.cb_group)

        # Convert to OpenCV format and then grayscale
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Get intrinsics from ROS topic instead
        mtx, dist_coeffs = get_zed_intrinsics(self, self.cb_group)

        try:
            translation, rotation = get_static_transform(
                cv_image_gray,
                mtx,
                dist_coeffs,
            )
        except ChessBoardCornersNotFoundError as e:
            msg = (
                f"Chessboard corners not found."
                f" Make sure chessboard is visible to camera. Error: {e}"
            )
            # Want to use ROS logger which does not provide exception logging.
            # Log the traceback manually instead.
            self.logger.error(msg)  # noqa: TRY400
            self.logger.debug(traceback.format_exc())

            response.success = False
            return response

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
            Quaternion, Rotation.from_matrix(inverse_rotation).as_quat(),
        )

        # Broadcast static transform
        self.br.sendTransform(transform_stamped)
        self.logger.info("Calibrated camera successfully.")
        response.success = True

        return response


def main(args: list[str] | None = None) -> None:
    """Spin the CameraChessboardNode."""
    rclpy.init(args=args)

    try:
        node = CameraChessboardNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
