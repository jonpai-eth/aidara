"""Node carrying out top level actions for the LLM planner."""

import pathlib

import numpy as np
import rclpy
import rclpy.duration
import ros2_numpy as rnp
import yaml
from ament_index_python import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from aidara_common.async_utils import FutureResolver
from aidara_common.node_utils import NodeMixin
from aidara_common.singleton import Singleton
from aidara_common.tf_utils import TfMixin
from aidara_msgs.srv import (
    GeometricGrasp,
    TargetPose,
)


class LLMActions(Node, TfMixin, NodeMixin, metaclass=Singleton):
    """Singleton for the performing LLM actions."""

    def __init__(self, config_file: str) -> None:
        """Initialize node for performing LLM actions."""
        Node.__init__(self, "llm_actions")
        TfMixin.__init__(self)
        NodeMixin.__init__(self)

        self._read_config(config_file)

        self._user_feedback_pub = self.create_publisher(String, "/user_feedback", 1)

        self._open_gripper_client = self.create_client(Trigger, "/open_gripper")
        self._close_gripper_client = self.create_client(Trigger, "/close_gripper")

        self._relax_client = self.create_client(
            Trigger,
            "/compliant_trajectory_controller/relax",
        )
        self._tighten_up_client = self.create_client(
            Trigger,
            "/compliant_trajectory_controller/tighten_up",
        )

        self._move_eef_client = self.create_client(TargetPose, "/move_eef")

        self._geometric_grasp_client = self.create_client(
            GeometricGrasp,
            "/geometric_grasp",
        )

        self._joint_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self.controller}/follow_joint_trajectory",
            callback_group=ReentrantCallbackGroup(),
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
        self._joint_names = config["joint_names"]
        self._home_joint_pos = config["home_joint_positions"]
        self._controller = config["controller_name"]

    def build_table_pose(self, pose: list[float]) -> PoseStamped:
        """Build a PoseStamped message in table frame."""
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "table"
        target_pose.pose.position = rnp.msgify(Point, np.array(pose))
        r = Rotation.from_euler("xyz", [180, 0, 90], degrees=True)
        target_pose.pose.orientation = rnp.msgify(Quaternion, r.as_quat(canonical=True))

        return target_pose

    def reset_joints(self) -> None:
        """Go to home position and neutral joint states."""
        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points.append(
            JointTrajectoryPoint(positions=self._home_joint_pos),
        )
        future = self._joint_action_client.send_goal_async(goal_msg)

        FutureResolver(
            self,
            "Follow_Joint_Trajectory_Action",
            future,
            lambda: self._joint_action_client.send_goal_async(goal_msg),
            lambda response: response.success,
        ).resolve_with_eh(
            n_retries=1,
            timeout=rclpy.duration.Duration(seconds=10),
        )

    @property
    def home_pose(self) -> PoseStamped:
        """The home configuration."""
        return self._home_pose

    @property
    def controller(self) -> str:
        """The name of the controller."""
        return self._controller

    @property
    def joint_names(self) -> list[str]:
        """The joint names."""
        return self._joint_names

    @property
    def home_joint_pos(self) -> list[float]:
        """The home configuration."""
        return self._home_joint_pos

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
    def relax_client(self) -> Client:
        """Client for the '/relax' service."""
        return self._relax_client

    @property
    def tighten_up_client(self) -> Client:
        """Client for the '/tighten_up' service."""
        return self._tighten_up_client

    @property
    def move_eef_client(self) -> Client:
        """Client for the '/move_eef_to' service."""
        return self._move_eef_client

    @property
    def geometric_grasp_client(self) -> Client:
        """Client for the '/geometric_grasp' service."""
        return self._geometric_grasp_client
