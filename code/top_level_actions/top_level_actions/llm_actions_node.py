"""Node carrying out top level actions for the LLM planner."""

import pathlib

import yaml
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from std_srvs.srv import Trigger

from aidara_common.singleton import Singleton
from aidara_common.tf_utils import TfMixin
from aidara_common.topic_utils import get_latest_msg_from_topic
from aidara_msgs.srv import GeometricGrasp, TargetPose


class LLMActions(Node, TfMixin, metaclass=Singleton):
    """Singleton for the performing LLM actions."""

    def __init__(self, config_file: str) -> None:
        """Initialize node for performing LLM actions."""
        Node.__init__(self, "llm_actions")
        TfMixin.__init__(self)

        self._read_config(config_file)

        self._user_feedback_pub = self.create_publisher(String, "/user_feedback", 1)

        self._open_gripper_client = self.create_client(Trigger, "/open_gripper")
        self._close_gripper_client = self.create_client(Trigger, "/close_gripper")

        self._move_eef_client = self.create_client(TargetPose, "/move_eef")

        self._geometric_grasp_client = self.create_client(
            GeometricGrasp,
            "/geometric_grasp",
        )

    def _read_config(self, config_file: str) -> None:
        share_dir = get_package_share_directory("top_level_actions")
        with (pathlib.Path(share_dir) / "config" / config_file).open() as f:
            config = yaml.safe_load(f)

        home_pose_config = config["home_pose"]
        home_pose = PoseStamped()
        home_pose.header.frame_id = home_pose_config["frame_id"]
        home_pose.pose.position.x = home_pose_config["position"]["x"]
        home_pose.pose.position.y = home_pose_config["position"]["y"]
        home_pose.pose.position.z = home_pose_config["position"]["z"]
        home_pose.pose.orientation.w = home_pose_config["orientation"]["w"]
        home_pose.pose.orientation.x = home_pose_config["orientation"]["x"]
        home_pose.pose.orientation.y = home_pose_config["orientation"]["y"]
        home_pose.pose.orientation.z = home_pose_config["orientation"]["z"]
        self._home_pose = home_pose

    @property
    def home_pose(self) -> PoseStamped:
        """The home configuration."""
        return self._home_pose

    @property
    def user_feedback_pub(self) -> Publisher:
        """Publisher to the '/user_feedback' topic."""
        return self._user_feedback_pub

    @property
    def open_gripper_client(self) -> Client:
        """Client for the '/open_gripper' service."""
        return self._open_gripper_client

    @property
    def close_gripper_client(self) -> Client:
        """Client for the '/close_gripper' service."""
        return self._close_gripper_client

    @property
    def move_eef_client(self) -> Client:
        """Client for the '/move_eef_to' service."""
        return self._move_eef_client

    @property
    def geometric_grasp_client(self) -> Client:
        """Client for the '/geometric_grasp' service."""
        return self._geometric_grasp_client

    def sleep(self, duration: Duration) -> None:
        """Sleep for the given duration."""
        rate = self.create_rate(
            S_TO_NS / duration.nanoseconds,
            self.get_clock(),
        )

        rate.sleep()

        rate.destroy()

    def get_handover_pose(self) -> PoseStamped:
        """Return the pose 10cm above the right hand of the user."""
        hand_position = get_latest_msg_from_topic(
            self,
            "/hand_position",
            PointStamped,
            ReentrantCallbackGroup(),
        )
        hand_position = self.do_tf(hand_position, "world")
        hand_position.point.z += 0.1

        res = PoseStamped()
        res.header = hand_position.header
        res.pose.position = hand_position.point
        res.pose.orientation.w = 1.0

        return res
