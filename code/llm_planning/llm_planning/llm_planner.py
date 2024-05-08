"""Top-level planner using a LLM interface."""

import argparse
import string
import textwrap
from typing import Literal

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .llm_interfaces import LLMInterfaceError, VisionMode, get_interface
from aidara_common.image_utils import get_all_images
from top_level_actions import (
    ACTION_COLLECTION,
    LLMActions,
)
from top_level_actions.action_collection import say

_EXEC_TEMPLATE = string.Template(
    """\
try:
$source
except Exception as e:
    say("There was an error while carrying out your request. Please try again.")
    LLMActions().get_logger().error(str(e))
""",
)


class LLMPlanner(Node):
    """Top-level planner node using a LLM interface."""

    def __init__(
        self,
        llm_type: Literal["gemini", "gpt-4"],
        examples_vision_mode: VisionMode,
        request_vision_mode: VisionMode,
        *,
        is_dry_run: bool,
    ) -> None:
        """Create LLMPlanner node."""
        super().__init__("llm_planner")
        self._llm_name = llm_type
        self._llm = get_interface(llm_type, examples_vision_mode)

        self._camera_names = request_vision_mode.get_camera_names()

        self._is_dry_run = is_dry_run

        self._llm_cb_group = MutuallyExclusiveCallbackGroup()
        self._instruction_sub = self.create_subscription(
            String,
            "/speech_to_text",
            self._instruction_cb,
            1,
            callback_group=self._llm_cb_group,
        )
        self._reset_history_service = self.create_service(
            Trigger,
            "~/reset_history",
            self._reset_history_cb,
            callback_group=self._llm_cb_group,
        )

    def _instruction_cb(self, instruction: String) -> None:
        """Carry out the instruction on the robot."""
        images = get_all_images(self, ReentrantCallbackGroup(), self._camera_names)

        try:
            source = self._llm.send_request(instruction.data, images)
        except LLMInterfaceError as e:
            say(
                f"{self._llm_name} was unable to fulfil your request."
                " Please try again.",
            )
            self.get_logger().error(str(e))
            return

        if self._is_dry_run:
            self.get_logger().info(
                f"Generated code:\n{source}\nWill not execute since this is a dry run.",
            )
            return

        source = textwrap.indent(source, "\t")
        source = _EXEC_TEMPLATE.safe_substitute(source=source)

        try:
            code = compile(source, f"<{self._llm_name} response>", "exec")
        except SyntaxError as e:
            say(f"{self._llm_name} produced invalid code. Please try again.")
            self.get_logger().error(str(e))
            return

        exec(code, ACTION_COLLECTION)  # noqa: S102

    def _reset_history_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._llm.reset_history()

        response.success = True
        return response


def main(args: list[str] | None = None) -> None:
    """Spin the LLMPlanner node."""
    parser = argparse.ArgumentParser(
        prog="ros2 run llm_planning llm_planner",
        description="Top-level planner using a LLM.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "If set, do not execute the generated code."
            " This prints the response of the LLM and returns."
        ),
    )
    parser.add_argument("--llm", choices=["gemini", "gpt-4"], default="gpt-4")
    parser.add_argument("--robot", choices=["franka", "staubli"], default="franka")
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
    parser.add_argument(
        "--ros-args",
        action="store_true",
        help="Ignored. There for ros2 launch compatibility.",
    )
    cli_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        planner_node = LLMPlanner(
            cli_args.llm,
            cli_args.examples_vision_mode,
            cli_args.request_vision_mode,
            is_dry_run=cli_args.dry_run,
        )
        actions_node = LLMActions(f"{cli_args.robot}_config.yaml")

        executor = MultiThreadedExecutor()
        executor.add_node(planner_node)
        executor.add_node(actions_node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
