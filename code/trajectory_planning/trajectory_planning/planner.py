"""Parent planner class."""

import argparse
import operator
import pathlib
from functools import partial

import numpy as np
import rclpy
import torch
import yaml
from ament_index_python import get_package_share_directory
from builtin_interfaces.msg import Duration as InterfaceDuration
from control_msgs.action import FollowJointTrajectory
from curobo.cuda_robot_model.cuda_robot_model import (
    CudaRobotModel,
)
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as CuRoboPose
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState as CuRoboJointState
from curobo.util_file import (
    get_robot_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    MotionGenResult,
    MotionGenStatus,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

from aidara_common.async_utils import FutureResolver
from aidara_common.tf_utils import TfMixin
from aidara_common.topic_utils import get_latest_msg_from_topic
from aidara_common.types import RobotName
from aidara_msgs.srv import TargetPose

TIMEOUT = Duration(seconds=20)
FREQUENCY = 5


class TrajectoryPlanner(Node, TfMixin):
    """Receive goal pose. Plan trajectory. Call action server of the controller."""

    def __init__(
        self,
        planner_yml: str,
    ) -> None:
        """Initialize servers and clients. Set up cuRobo."""
        Node.__init__(self, node_name="trajectory_planner")
        TfMixin.__init__(self)

        self._read_config(planner_yml)
        self._cb_group = ReentrantCallbackGroup()

        self._move_eef_srv = self.create_service(
            TargetPose,
            "/move_eef",
            self._receive_goal_pose,
            callback_group=self._cb_group,
        )

        self._follow_joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._action_server_topic,
            callback_group=self._cb_group,
        )

        # cuRobo setup
        self._tensor_args = TensorDeviceType()

        self._motion_gen_config = MotionGenConfig.load_from_robot_config(
            self._robot_file,
            self._world_file,
            self._tensor_args,
            use_cuda_graph=True,
            num_ik_seeds=500,
            num_trajopt_seeds=30,
            num_graph_seeds=5,
            interpolation_dt=0.1,
        )

        self._motion_gen = MotionGen(self._motion_gen_config)
        self._motion_gen.warmup(enable_graph=True)

        robot_cfg = load_yaml(
            join_path(get_robot_configs_path(), self._robot_file),
        )["robot_cfg"]
        robot_cfg = RobotConfig.from_dict(robot_cfg, self._tensor_args)
        self._kin_model = CudaRobotModel(robot_cfg.kinematics)

        self.get_logger().info("Warmup completed. Ready to receive goals.")

    def _read_config(self, planner_yml: str) -> None:
        """Read the yaml file and set constants."""
        share_dir = get_package_share_directory("trajectory_planning")
        with (pathlib.Path(share_dir) / "config" / planner_yml).open() as f:
            config = yaml.safe_load(f)

        self._action_server_topic = config["action_server_topic"]
        self._world_file = config["world_yml"]
        self._robot_file = config["robot_yml"]
        self._robot_name = RobotName(config["robot_name"])
        self._dof = config["dof"]

        self._center_height = config["projection_params"]["center_height"]
        self._workspace_radius = config["projection_params"]["workspace_radius"]
        self._sphere_radius = config["projection_params"]["sphere_radius"]
        self._projected_radius = config["projection_params"]["projected_radius"]

    def _call_action_server(self, goal: FollowJointTrajectory.Goal) -> bool:
        """Call action server and wait for result."""
        action_server_name = f"Action server {self._action_server_topic}"

        goal_handle = FutureResolver.create(
            self,
            action_server_name,
            partial(self._follow_joint_trajectory_client.send_goal_async, goal),
            operator.attrgetter("accepted"),
        ).resolve_with_eh(timeout=TIMEOUT, n_retries=0)

        try:
            response = (
                FutureResolver.create(
                    self,
                    action_server_name,
                    lambda: goal_handle.get_result_async(),
                    lambda response: response.result.error_code
                    == response.result.SUCCESSFUL,
                )
                .resolve_with_eh(timeout=TIMEOUT, n_retries=0)
                .result
            )

        except RuntimeError as e:
            self.get_logger().info(str(e))
            return False

        return response.error_code == response.SUCCESSFUL

    def _get_alternative_goal(
        self,
        q_start: CuRoboJointState,
        goal_pose: CuRoboPose,
    ) -> FollowJointTrajectory.Goal | None:
        """Return None if inverse kinematics failure occurs."""
        if goal_pose.position is None:
            return None

        sphere_center = np.array([0, 0, self._center_height])

        point = goal_pose.position.cpu().numpy()[0]
        vector_to_point = point - sphere_center
        length_of_vector = np.linalg.norm(vector_to_point)

        if not self._verify_radius(
            self._sphere_radius,
            length_of_vector,
            self._workspace_radius,
        ):
            return None

        normalized_vector = vector_to_point / length_of_vector
        projected_point = sphere_center + normalized_vector * self._projected_radius
        direction = (
            np.array([1, 0, 0, 0])
            if self._robot_name == "franka"
            else self._projected_eef_quaternion(normalized_vector)
        )

        projected_goal = np.concatenate((projected_point, direction))
        projected_goal_pose = CuRoboPose.from_list(
            projected_goal,
            TensorDeviceType(),
            q_xyzw=True,
        )

        result = self._motion_gen.plan_single(
            q_start,
            projected_goal_pose,
            MotionGenPlanConfig(),
        )

        if result.status == MotionGenStatus.IK_FAIL:
            return None

        self.get_logger().warn("Goal is out of bounds. Projecting goal.")
        self._goal_pose = projected_goal
        return self._convert_trajectory(result)

    def _verify_radius(
        self,
        min_radius: float,
        radius: float,
        max_radius: float,
    ) -> bool:
        """Check whether the goal is in the specified range."""
        if radius < min_radius:
            self.get_logger().error("Pose within range but not reachable.")
            return False

        if radius > max_radius:
            self.get_logger().error("Target is too far away.")
            return False
        return True

    def _projected_eef_quaternion(self, normalized_vector: np.ndarray) -> np.ndarray:
        """Calculate the orientation of the eef for projected goals."""
        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(normalized_vector, z_axis)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        dot_product = np.dot(normalized_vector, z_axis)
        rotation_angle = -np.arccos(dot_product)
        rot_projected = Rotation.from_rotvec(rotation_angle * rotation_axis)
        rot_180 = Rotation.from_euler("z", 180, degrees=True)
        complete_rotation = rot_projected * rot_180
        return complete_rotation.as_quat()

    def _get_joint_states(self) -> CuRoboJointState:
        """Get the start position of the robot."""
        msg = get_latest_msg_from_topic(
            self,
            "/joint_states",
            JointState,
            self._cb_group,
        )
        if msg is not None:
            start_positions = msg.position[: self._dof]
            self._joint_names = msg.name[: self._dof]

            return CuRoboJointState.from_position(
                self._tensor_args.to_device([start_positions]),
                joint_names=self._joint_names,
            )

        error_msg = "Unable to retrieve the latest joint states message."
        raise RuntimeError(error_msg)

    def _generate_trajectory(
        self,
        q_start: CuRoboJointState,
        goal_pose: CuRoboPose,
    ) -> FollowJointTrajectory.Goal | None:
        """Generate collision free trajectory."""
        result = self._motion_gen.plan_single(
            q_start,
            goal_pose,
            MotionGenPlanConfig(),
        )

        if result.status == MotionGenStatus.IK_FAIL:
            return self._get_alternative_goal(q_start, goal_pose)

        return self._convert_trajectory(result)

    def _create_trajectory_point(
        self,
        duration_from_start: InterfaceDuration,
        positions: torch.Tensor,
    ) -> JointTrajectoryPoint:
        """Create trajectory point."""
        point = JointTrajectoryPoint()
        point.time_from_start = duration_from_start

        while len(positions.size()) > 1:
            positions = positions[0]

        point.positions = positions[: self._dof]

        return point

    def _convert_trajectory(
        self,
        result: MotionGenResult,
    ) -> FollowJointTrajectory.Goal | None:
        """Convert the cuRobo trajectory into a goal object for the action server."""
        dt = int(result.interpolation_dt * S_TO_NS)
        old = result.interpolated_plan

        if old is None or old.joint_names is None:
            return None

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = self._joint_names

        goal.trajectory.points = [
            self._create_trajectory_point(
                InterfaceDuration(nanosec=(dt * i)),
                p,
            )
            for i, p in enumerate(old.position)
        ]
        return goal

    def _receive_goal_pose(
        self,
        request: TargetPose.Request,
        response: TargetPose.Response,
    ) -> TargetPose.Response:
        """Receive goal pose. Call action server."""
        q_start = self._get_joint_states()
        self.get_logger().info("Received request.")
        response.success = False
        pose = request.target.pose
        pose_frame_id = request.target.header.frame_id

        self.get_logger().debug(
            f"Goal Position in {pose_frame_id} frame:\n {pose.position}",
        )
        self.get_logger().debug(
            f"Goal Quaternion in {pose_frame_id} frame:\n {pose.orientation}",
        )

        if pose_frame_id != "world":
            pose_msg = self.do_tf(request.target, "world")
            pose = pose_msg.pose
            if pose is None:
                return response

        self._goal_pose = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
        self.get_logger().info(f"Goal Position in world frame:\n {pose.position}")
        self.get_logger().info(f"Goal Quaternion in world frame:\n {pose.orientation}")
        goal_pose_tensor = CuRoboPose.from_list(self._goal_pose, self._tensor_args)
        goal = self._generate_trajectory(q_start, goal_pose_tensor)

        if goal is None:
            self.get_logger().error("No collision free path found.")
            return response

        response.success = self._call_action_server(goal)
        return response

    def _get_cartesian_position(
        self,
        joint_position: JointTrajectoryPoint,
    ) -> list[float]:
        """Convert a joint trajectory point to a Cartesian point."""
        joint_position_tensor = torch.tensor(
            joint_position.positions,
            dtype=self._tensor_args.dtype,
            device=self._tensor_args.device,
        )
        cartesian_tensor = self._kin_model.get_state(joint_position_tensor)
        return cartesian_tensor.ee_position[0].cpu().numpy()


def main(args: list[str] | None = None) -> None:
    """Spin Node."""
    parser = argparse.ArgumentParser(
        prog="ros2 run trajectory_planner planner",
        description="Trajectory planner based on cuRobo library.",
    )
    parser.add_argument(
        "--robot", type=RobotName, choices=RobotName, default=RobotName.FRANKA,
    )
    parser.add_argument(
        "--ros-args",
        action="store_true",
        help="Ignored. There for ros2 launch compatibility.",
    )

    robot_name = parser.parse_args()
    rclpy.init(args=args)

    try:
        node = TrajectoryPlanner(
            planner_yml=f"{robot_name.robot!s}_planner_config.yml",
        )

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
