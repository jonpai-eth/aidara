"""ROS2 controller for the gripper of the Stäubli robot."""

import socket

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from aidara_common.node_utils import NodeMixin

HOST = "192.168.1.155"
PORT_GRIPPER = 1239
TIMER_PERIOD = 0.5


class StaubliGripper(Node, NodeMixin):
    """ROS 2 Node for sending gripper position."""

    def __init__(self) -> None:
        """Initialize Stäubli_gripper node."""
        Node.__init__(self, node_name="staubli_gripper")
        NodeMixin.__init__(self)

        self._close_gripper_service = self.create_service(
            Trigger,
            "/close_gripper",
            self._close_gripper,
        )
        self._open_gripper_service = self.create_service(
            Trigger,
            "/open_gripper",
            self._open_gripper,
        )

        self._socket_setup()

        self._maintain_connection_timer = self.create_timer(
            TIMER_PERIOD, self._maintain_connection,
        )

    def _socket_setup(self) -> None:
        """Connect to the robot via sockets to command the gripper."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT_GRIPPER))
            s.listen()
            self._gripper_connection, address = s.accept()
        self.get_logger().info(f"Connection for commands from: '{address}'.")

    def _maintain_connection(self) -> None:
        """Send newline to maintain connection to robot and prevent timeout."""
        empty_msg = "\n"
        self._send_data_with_eh(empty_msg)

    def _close_gripper(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Close Stäubli gripper."""
        closing_msg = "0\n"
        self._send_data_with_eh(closing_msg)
        response.success = True
        self.sleep(1.25)
        return response

    def _open_gripper(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Open Stäubli gripper."""
        opening_msg = "1\n"
        self._send_data_with_eh(opening_msg)
        response.success = True
        self.sleep(1.25)
        return response

    def _send_data_with_eh(self, msg: str) -> None:
        """Send commands and reestablish connection if error occurs."""
        try:
            self._gripper_connection.sendall(msg.encode())
        except BrokenPipeError as e:
            self.get_logger().error(f"Gripper is not reachable: {e}")
            self._gripper_connection.close()
            self._socket_setup()


def main(args: list[str] | None = None) -> None:
    """Spin Node."""
    rclpy.init(args=args)

    try:
        node = StaubliGripper()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
