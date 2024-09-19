"""
ROS2 controller for the gripper of the Franka robot.

Calls the action servers of the franka_ros2 to control the gripper.
"""

import rclpy
from franka_msgs.action import Grasp
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from aidara_common.async_utils import FutureResolver


class FrankaGripper(Node):
    """ROS 2 Node for sending gripper position."""

    def __init__(self) -> None:
        """Initialize franka_gripper node."""
        super().__init__("staubli_gripper")
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
        self._action_client = ActionClient(self, Grasp, "/panda_gripper/grasp")

    def _close_gripper(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Close Franka gripper."""
        response.sucess = False

        goal = Grasp.Goal()
        goal.epsilon.inner = 0.1
        goal.epsilon.outer = 0.1
        goal.force = 50.0
        goal.width = 0.0
        goal.speed = 0.2

        response.success = self._send_grasp_goal(goal)
        return response

    def _open_gripper(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Open Franka gripper."""
        response.sucess = False

        goal = Grasp.Goal()
        goal.width = 0.1
        goal.speed = 0.1
        goal.force = 50.0

        response.success = self._send_grasp_goal(goal)
        return response

    def _send_grasp_goal(self, goal: Grasp.Goal) -> bool:
        future = self._action_client.send_goal_async(goal)

        try:
            FutureResolver(
                self,
                "/panda_gripper/grasp",
                future,
                lambda: self._action_client.send_goal_async(goal),
                lambda _response: True,
                # response success depends on final gripper width (don't care)
            ).resolve_with_eh(
                n_retries=1,
                timeout=rclpy.duration.Duration(seconds=10),
            )
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return False

        return True


def main(args: list[str] | None = None) -> None:
    """Spin Node."""
    rclpy.init(args=args)

    try:
        node = FrankaGripper()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
