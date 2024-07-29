"""
ROS2 controller for the Stäubli TX2-60 robot.

This controller communicates with the Robot via sockets. It sends joint positions as
commands and receives the current joint positions and publishes them.
"""

import math
import socket

import numpy as np
import rclpy
import ros2_numpy as rnp
import torch
from control_msgs.action import FollowJointTrajectory
from curobo.cuda_robot_model.cuda_robot_model import (
    CudaRobotModel,
)
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_path, join_path, load_yaml
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from aidara_common.tf_utils import TfMixin

HOST = "192.168.1.155"
PORT_READ = 1235
PORT_WRITE = 1237

FREQUENCY = 5
TIMER_PERIOD = 0.01
TIMEOUT = 40
TOLERANCE = 0.01

TIMEOUT_MSG = (
    "\n\n"
    "██     ██    █████    ██████    ███    ██   ██   ███    ██    ██████\n"
    "██     ██   ██   ██   ██   ██   ████   ██   ██   ████   ██   ██      \n"
    "██  █  ██   ███████   ██████    ██ ██  ██   ██   ██ ██  ██   ██   ███\n"
    "██ ███ ██   ██   ██   ██   ██   ██  ██ ██   ██   ██  ██ ██   ██    ██\n"
    " ███ ███    ██   ██   ██   ██   ██   ████   ██   ██   ████    ██████\n"
    "\n"
    "    ------------------------------------------------------------\n"
    "    ------------------------------------------------------------\n"
    "    || Eef did not reach goal. Restart the application on the ||\n"
    "    || Stäubli Controller.n                                   ||\n"
    "    || FAILURE TO COMPLY MAY LEAD TO DAMAGE TO THE ROBOT.     ||\n"
    "    ------------------------------------------------------------\n"
    "    ------------------------------------------------------------\n"
)


class StaubliController(Node, TfMixin):
    """
    ROS 2 Node for sending and reading joint positions of Stäubli robots.

    The controller can be called from an action client and sends trajectories to the
    robot. It also publishes the current joint states and the eef transform.
    """

    def __init__(self) -> None:
        """Initialize Stäubli_controller node."""
        Node.__init__(self, node_name="staubli_controller")
        TfMixin.__init__(self)

        # Establish connection with the robot via sockets
        self._read_setup()
        self._control_setup()

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/staubli_controller/follow_joint_trajectory",
            self._control_trajectory,
        )
        self._joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            1,
        )
        self._eef_pub = self.create_publisher(TransformStamped, "/eef_pose", 10)
        self._eef_pub_timer = self.create_timer(TIMER_PERIOD, self._check_current_pos)

        # Initiate cuRobo for forward kinematics
        self._tensor_args = TensorDeviceType()

        config_file = load_yaml(join_path(get_robot_path(), "tx2_60.yml"))

        urdf_file = config_file["robot_cfg"]["kinematics"]["urdf_path"]
        base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
        ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]

        robot_cfg = RobotConfig.from_basic(
            urdf_file,
            base_link,
            ee_link,
            self._tensor_args,
        )

        self._kin_model = CudaRobotModel(robot_cfg.kinematics)
        self._ee_position = np.array([0.0, 0.0, 0.0])

    def _read_setup(self) -> None:
        """Initialize socket for reading joint position."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT_READ))
            s.listen()
            self._conn_read, self._read_address = s.accept()
        self.get_logger().info(f"Connection for reading from: '{self._read_address}'.")

    def _control_setup(self) -> None:
        """Initialize socket for sending joint positions."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT_WRITE))
            s.listen()
            self._conn_control, address = s.accept()
        self.get_logger().info(f"Connection for commands from: '{address}'.")

    def _control_trajectory(
        self,
        goal_handle: ServerGoalHandle,
    ) -> FollowJointTrajectory.Result:
        """Slice trajectory in single points."""
        points = goal_handle.request.trajectory.points
        for point in points:
            self._control_point(point.positions)

        result = FollowJointTrajectory.Result()
        goal_joint_pose = points[-1]
        if self._wait_for_result(goal_joint_pose):
            goal_handle.succeed()
            result.error_code = 0
        else:
            goal_handle.abort()
            result.error_code = -5

        return result

    def _wait_for_result(self, goal_joint_pose: JointTrajectoryPoint) -> bool:
        """Compare goal with current eef position."""
        goal_position = self._get_cartesian_position(goal_joint_pose)
        rate = self.create_rate(FREQUENCY)

        for _ in range(int(TIMEOUT * FREQUENCY)):
            difference = np.linalg.norm(
                np.array(goal_position) - self._ee_position,
            ).item()
            if difference is None:
                return False

            if difference <= TOLERANCE:
                self.get_logger().info("Reached Goal Pose")
                return True
            rate.sleep()

        self.get_logger().error(TIMEOUT_MSG)
        return False

    def _control_point(self, point: list[float]) -> None:
        """Send single points to robot."""
        point_degrees = np.degrees(point)
        with np.printoptions(
            formatter={"float_kind": lambda x: f"{x:06.1f}"},
        ):
            data = np.array2string(point_degrees, separator=" ")

        data_formatted = data[1:-1] + " \n"
        try:
            self._conn_control.sendall(data_formatted.encode())
        except BrokenPipeError as e:
            self.get_logger().error(f"Robot is not reachable: {e}")
            self._reset_socket_connections()

    def _reset_socket_connections(self) -> None:
        """Close the existing connections and setup new sockets."""
        self._conn_read.close()
        self._read_setup()
        self._conn_control.close()
        self._control_setup()

    def _check_current_pos(self) -> None:
        """Publish current joint position."""
        data = self._conn_read.recv(1024).decode()
        filtered_data = data.strip().splitlines()[-1] if data != "" else None

        if filtered_data is None:
            self.get_logger().error(
                "Robot is not responding. Start the program on the robot.",
            )
            self._reset_socket_connections()
            return

        try:
            current_joint_position = [
                math.radians(float(pos.strip())) for pos in filtered_data.split(",")
            ]
        except ValueError as e:
            self.get_logger().error(f"Error converting data to float: {e}")
            raise RuntimeError from e

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            "tx2_60_joint1",
            "tx2_60_joint2",
            "tx2_60_joint3",
            "tx2_60_joint4",
            "tx2_60_joint5",
            "tx2_60_joint6",
        ]
        joint_state_msg.position = current_joint_position
        self._joint_state_pub.publish(joint_state_msg)
        self._publish_forward_kinematics(current_joint_position)

    def _get_cartesian_position(
        self,
        joint_position: JointTrajectoryPoint,
    ) -> list[float]:
        joint_position_tensor = torch.tensor(
            joint_position.positions,
            dtype=self._tensor_args.dtype,
            device=self._tensor_args.device,
        )
        cartesian_tensor = self._kin_model.get_state(joint_position_tensor)
        return cartesian_tensor.ee_position[0].cpu().numpy()

    def _publish_forward_kinematics(
        self,
        current_joint_position: list[float],
    ) -> None:
        """Do forward kinematics and publish tf2 transform."""
        joint_position_tensor = torch.tensor(
            current_joint_position,
            dtype=self._tensor_args.dtype,
            device=self._tensor_args.device,
        )
        eef_tensor = self._kin_model.get_state(joint_position_tensor)
        eef_position = eef_tensor.ee_position[0].cpu().numpy()
        self._ee_position = np.array(eef_position)

        # Quaternion is in the following format: w x y z
        eef_quaternion = eef_tensor.ee_quaternion[0].cpu().numpy()

        transform = TransformStamped()
        transform.header.frame_id = "world"
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.child_frame_id = "eef"

        transform.transform.translation = rnp.msgify(
            Vector3,
            eef_position.astype(np.float64),
        )
        transform.transform.rotation = rnp.msgify(
            Quaternion,
            eef_quaternion.astype(np.float64),
        )
        self._eef_pub.publish(transform)
        self.publish_static_transform(transform)


def main(args: list[str] | None = None) -> None:
    """Spin Node."""
    rclpy.init(args=args)

    try:
        node = StaubliController()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
