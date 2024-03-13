"""Top-level planner using a LLM interface."""

import argparse
from typing import Literal

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .llm_interfaces import VisionMode, get_interface
from aidara_common.image_utils import get_all_images

SCRIPT_IMPORTS = """\
from top_level_actions import (
    move_gripper_by,
    give_to_hand,
    take_from_hand,
    place_object_on_table_at,
    pick_object_from_table,
    say,
)

"""


class LLMPlanner(Node):
    """Top-level planner node using a LLM interface."""

    def __init__(
        self,
        llm_type: Literal["gemini", "gpt-4"],
        examples_vision_mode: VisionMode,
        request_vison_mode: VisionMode,
    ) -> None:
        """Create LLMPlanner node."""
        super().__init__("llm_planner")
        self._llm = get_interface(llm_type, examples_vision_mode)
        self._camera_names = request_vison_mode.get_camera_names()

        self._planning_cb_group = ReentrantCallbackGroup()
        self._instruction_sub = self.create_subscription(
            String,
            "/speech2text",
            self.plan,
            1,
            callback_group=self._planning_cb_group,
        )

    def plan(self, instruction: String) -> None:
        """Carry out the instruction on the robot."""
        images = get_all_images(self, self._planning_cb_group, self._camera_names)

        code = self._llm.send_request(instruction.data, images)

        script = SCRIPT_IMPORTS + code
        exec(script)  # noqa: S102


def main(args: list[str] | None = None) -> None:
    """Spin the LLMPlanner node."""
    parser = argparse.ArgumentParser(
        prog="ros2 run llm_planning llm_planner",
        description="Top-level planner using a LLM.",
    )
    parser.add_argument("--llm", choices=["gemini", "gpt-4"], default="gpt-4")
    parser.add_argument(
        "--examples-vision-mode",
        type=VisionMode,
        choices=VisionMode,
        default=VisionMode.ALL,
    )
    parser.add_argument(
        "--request-vision-mode",
        type=VisionMode,
        choices=VisionMode,
        default=VisionMode.ALL,
    )
    cli_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        node = LLMPlanner(
            cli_args.llm,
            cli_args.examples_vision_mode,
            cli_args.request_vision_mode,
        )

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
