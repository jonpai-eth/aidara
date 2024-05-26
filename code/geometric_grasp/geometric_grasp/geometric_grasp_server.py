"""Server to execute table top grasp from segmentation mask."""

import math

import numpy as np
import rclpy
import rerun as rr
import ros2_numpy as rnp
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from lang_sam import LangSAM
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.constants import S_TO_NS
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from sklearn.decomposition import PCA

from aidara_common.image_utils import (
    get_img_from_zed,
    get_zed_intrinsics,
    imgmsg_to_pil,
    mono8_to_imgmsg,
)
from aidara_common.node_utils import NodeMixin
from aidara_common.tf_utils import TfMixin
from aidara_msgs.srv import GeometricGrasp

MAX_INTENSITY = 255


class GeometricGraspServer(Node, TfMixin, NodeMixin):
    """Server to execute table top grasp from segmentation mask."""

    def __init__(self) -> None:
        """Initialize service, load models and create client for the tf2_server."""
        Node.__init__(self, "geometric_grasp_server")
        TfMixin.__init__(self)
        NodeMixin.__init__(self)

        self._grasping_cb_group = ReentrantCallbackGroup()

        self._srv = self.create_service(
            GeometricGrasp,
            "/geometric_grasp",
            self._grasp_callback,
            callback_group=self._grasping_cb_group,
        )

        self._seg_pub = self.create_publisher(Image, "segmentation", 10)

        self._model = LangSAM()

    def _grasp_callback(
        self,
        request: GeometricGrasp.Request,
        response: GeometricGrasp.Response,
    ) -> GeometricGrasp.Response:
        """Issue all function calls to finally get a grasp position."""
        response.success = False

        self.init_rerun()

        # extract requested class
        object_description = request.object_description.data

        start_time = self.get_clock().now()
        img = get_img_from_zed(self, self._grasping_cb_group, camera_name="zed_top")

        received_image_time = self.get_clock().now()

        mask = self._get_segmentation(img, object_description)
        if mask is None:
            return response  # Return success False if object could not be detected

        received_segmap_time = self.get_clock().now()
        self.get_logger().debug(
            "Received segmap after "
            f"{(received_segmap_time - received_image_time).nanoseconds / S_TO_NS}",
        )

        self._publish_segmap(mask.copy(), object_description)
        intrinsics, _dist_coeffs = get_zed_intrinsics(
            self,
            self._grasping_cb_group,
            camera_name="zed_top",
        )

        points = self._compute_points(mask, intrinsics)
        grasp_point = self._get_grasp_point(points)
        self.get_logger().debug(f"grasp_point: {grasp_point}")
        pose_stamped = self._get_pose(grasp_point[0], grasp_point[1], grasp_point[2])
        self.get_logger().debug(f"pose_stamped: {pose_stamped.pose}")

        received_pose_time = self.get_clock().now()
        response.pose = pose_stamped

        self.get_logger().debug(
            "Finished whole callback after "
            f"{((received_pose_time - start_time).nanoseconds / S_TO_NS)}",
        )

        response.success = True
        return response

    def _get_segmentation(self, ros_img: Image, seg_prompt: str) -> np.ndarray | None:
        """
        Get segmentation mask corresponding to text prompt.

        Args:
            ros_img: ROS2 sensor_msgs/Image message
            seg_prompt: natural language prompt for object to be segmented in the scene.
                        examples: "green pen on the table" or "rightmost hammer"
        Returns:
            np.ndarray of shape (H, W)
            or None if desired object could not be detected

        """
        image_pil = imgmsg_to_pil(ros_img)

        masks, _boxes, _phrases, _logits = self._model.predict(image_pil, seg_prompt)

        if masks is None or len(masks) == 0:
            self.get_logger().error(f"'{seg_prompt}' could not be detected.")
            return None

        first_mask = masks[0]

        first_mask = first_mask.mul(
            255,
        ).byte()
        return first_mask.cpu().numpy()

    def _compute_points(
        self,
        mask: np.ndarray,
        cam_intrinsics: np.ndarray,
    ) -> np.ndarray:
        """
        Compute 3D coordinates of the image pixels in chessboard frame.

        Args:
            mask: np.ndarray of shape (H, W)
            cam_intrinsics: np.ndarray of shape(3, 3)

        Returns:
            np.ndarray of shape (2, N)
        """
        t = self.get_clock().now()
        chess_to_cam_tf = self.get_tf(
            "chessboard",
            "zed_top_left_camera_optical_frame_opencv",
        )
        self.get_logger().debug(f"Chess to cam tf: {chess_to_cam_tf.transform}")
        self.get_logger().debug(
            f"Time for get_tf: {(self.get_clock().now()-t).nanoseconds / S_TO_NS}",
        )

        tf_rot = chess_to_cam_tf.transform.rotation
        tf_trans = chess_to_cam_tf.transform.translation

        chess_to_cam_tf = np.vstack(
            (
                np.hstack(
                    (
                        Rotation.from_quat(
                            [tf_rot.x, tf_rot.y, tf_rot.z, tf_rot.w],
                        ).as_matrix(),
                        np.array([[tf_trans.x], [tf_trans.y], [tf_trans.z]]),
                    ),
                ),
                np.array([0, 0, 0, 1]),
            ),
        )
        cam_to_chess_tf = np.linalg.inv(chess_to_cam_tf)

        # get z value of table in camera frame
        z = chess_to_cam_tf[2, 3]

        indices = np.where(mask == MAX_INTENSITY)

        obj_coordinates = np.ones((4, indices[0].size))

        obj_coordinates[0] = (indices[1] - cam_intrinsics[0, 2]) * (
            z / cam_intrinsics[0, 0]
        )

        obj_coordinates[1] = (indices[0] - cam_intrinsics[1, 2]) * (
            z / cam_intrinsics[1, 1]
        )

        obj_coordinates[2] = z

        pixel_positions = cam_to_chess_tf @ obj_coordinates

        # Return labeled points in 2D chessboard frame
        return pixel_positions[[0, 1], :]

    def _get_grasp_point(self, points: np.ndarray) -> tuple[float, float, float]:
        """
        Sample a grasp position via PCA.

        Args:
            mask: 2D points in table with shape (2, N)

        Returns:
            x and y coordinate and angle in radians (wrt. y axis) of the grasp point
        """
        # get centroid
        x = np.mean(points[0, :])
        y = np.mean(points[1, :])

        # get angle of shortest length
        pca = PCA(n_components=2)
        pca.fit(points.T)
        best_angle = (
            np.arctan2(pca.components_[1, 1], pca.components_[1, 0]) % math.pi
        ) + (math.pi / 2)  # Rotate gripper 90 degrees around z-axis

        return x.item(), y.item(), best_angle.item()

    def _get_pose(self, x: float, y: float, angle: float) -> PoseStamped:
        """
        Get transformation from the grasp point as ros msg.

        Args:
            grasp_point: Tuple of x, y and angle (wrt y axis)

        Returns:
            PoseStamped object that represents 6d transformation
        """
        # extract rotation and translation
        translation = Point()
        translation.x = x
        translation.y = y

        r = Rotation.from_euler("z", angle, degrees=False).as_quat(canonical=False)
        quaternion = rnp.msgify(Quaternion, r)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = "chessboard"
        pose_stamped_msg.pose.position = translation
        pose_stamped_msg.pose.orientation = quaternion

        return pose_stamped_msg

    def _publish_segmap(self, segmap: np.ndarray, description: str) -> None:
        """
        Publish segmentation map as img to ROS topic for visualization with RViz.

        Args:
            segmap: NumPy NDArray of shape H, W
        """
        segmap[np.where(segmap == 1)] = 255

        rr.log(
            "/",
            rr.AnnotationContext(
                [rr.AnnotationInfo(id=255, label=description, color=(255, 0, 0))],
            ),
            static=True,
        )
        rr.log(
            "world/zed_top/image/segmentation",
            rr.SegmentationImage(segmap),
            static=True,
        )
        ros_segmap = mono8_to_imgmsg(segmap)
        self._seg_pub.publish(ros_segmap)


def main(args: None = None) -> None:
    """Run service that computes a grasp position for a given object class."""
    rclpy.init(args=args)

    try:
        node = GeometricGraspServer()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
