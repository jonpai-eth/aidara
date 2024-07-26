"""Top-level planner using a LLM interface."""

import argparse
import re
import string
import textwrap
from typing import Literal

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from termcolor import colored

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
    LLMActions().get_logger().error(str(e))
    error = e
""",
)


class LLMPlanner(Node):
    """Top-level planner node using a LLM interface."""

    def __init__(
        self,
        llm_type: Literal["gemini", "gpt-4"],
        examples_vision_mode: VisionMode,
        request_vision_mode: VisionMode,
        prompt_version: str,
        *,
        is_dry_run: bool,
    ) -> None:
        """Create LLMPlanner node."""
        super().__init__("llm_planner")
        self._llm_name = llm_type
        self._llm = get_interface(llm_type, examples_vision_mode, prompt_version)

        self._camera_names = list(request_vision_mode.get_camera_names())

        self._is_dry_run = is_dry_run

        self._llm_cb_group = MutuallyExclusiveCallbackGroup()
        self._instruction_sub = self.create_subscription(
            String,
            "/speech_to_text",
            self._instruction_cb,
            1,
            callback_group=self._llm_cb_group,
        )
        self._code_publisher = self.create_publisher(String, "/llm_code", 1)
        self._reset_history_service = self.create_service(
            Trigger,
            "~/reset_history",
            self._reset_history_cb,
            callback_group=self._llm_cb_group,
        )

    def _instruction_cb(self, instruction: String) -> None:  # noqa: C901
        """Carry out the instruction on the robot."""
        say("Understood")
        self.get_logger().info(colored("Callback started.", "green"))
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

        source = self._check_markdown(source)

        self._code_publisher.publish(String(data=source))

        code = textwrap.indent(source, "\t")
        code = _EXEC_TEMPLATE.safe_substitute(source=code)

        self.get_logger().info(f"Generated code:\n{code}")

        if self._is_dry_run:
            self.get_logger().info(
                "\nWill not execute since this is a dry run.",
            )
            return

        try:
            code = compile(code, f"<{self._llm_name} response>", "exec")
        except SyntaxError as e:
            say(f"{self._llm_name} produced invalid code. Please try again.")
            self.get_logger().error(str(e))
            return

        locals_ = {}
        exec(code, ACTION_COLLECTION, locals_)  # noqa: S102

        if "error" in locals_:
            self._llm.append_error(locals_["error"])

    def _reset_history_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._llm.reset_history()

        response.success = True
        return response

    @staticmethod
    def _check_markdown(response: str) -> str:
        """Return code from first markdown block or 'say(response)' for no markdown."""
        try:
            match = re.search(r"```python\s*?\n([\s\S]*?\n)\s*```", response)
        except ValueError as e:
            raise LLMInterfaceError from e

        if match is None:
            return f"say('{response}')"

        return match.group(1)


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
        "--prompt-version",
        choices=["minigame", "playground"],
        default="playground",
    )
    parser.add_argument(
        "--ros-args",
        action="store_true",
        help="Ignored. There for ros2 launch compatibility.",
    )
    cli_args = parser.parse_args()

    if cli_args.robot == "staubli" and cli_args.prompt_version == "playground":
        msg = colored(
            "The playground prompt is not compatible with the Staubli robot.",
            "red",
        )
        raise RuntimeError(msg)

    rclpy.init(args=args)

    try:
        planner_node = LLMPlanner(
            cli_args.llm,
            cli_args.examples_vision_mode,
            cli_args.request_vision_mode,
            cli_args.prompt_version,
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
