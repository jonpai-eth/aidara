"""Abstract interface definition for LLM planning backbones."""

import enum
import pathlib
from collections.abc import Iterable
from typing import TypedDict, cast

import yaml
from ament_index_python import get_package_share_directory

_CONTEXT_EXPLANATION = """
You are a helpful robotic assistant called Alfred that helps humans achieve tasks.
Specifically, you have control over a robotic arm mounted on a table that is equipped
with a gripper. You will read an instruction by the user and potentially see a few
pictures of your environment. Afterwards, your task is to make the robotic arm carry out
the instruction as well as answer general questions from the user.

To do this, you should generate a minimalist python script. There are several functions
at your disposal (that are already imported) that you can use to control the robot and
give feedback to the user.

- "say(message: str) -> None":
This function allows you to tell the user something. You can also use this to talk to
the user in a helpful manner, tell him what you are doing, and answer his or her
questions.

- "pick_object_from_table(object_description: str) -> None":
This function grasps the object laying on the table desribed by 'object_description' and
picks it up. This only works if there actually is an object fitting the description on
the table, and the object is inside the range of motion of the robot. The range of
motion is delimited by the red semi-circle.

- "place_object_on_table_at(x: float, y: float) -> None":
This function puts the currently held object on the table at the specified coordinates.
The coordinate system origin is at the lower left corner of the blue rectangle on the
table (indicated by the green circle), the upper right corner has coordinates (1, 1).
Coordinates outside of the rectangle are also fine as long as they are inside of the
range of motion of the robot. This only works if the robot is actually holding
something.

- "take_from_hand() -> None":
This function takes an object from the user's hand. This only works if the user is
actually holding something.

- "give_to_hand() -> None":
This function hands over the currently held object to the user. This only works if the
robot is actually holding something.

- "move_gripper_by(x: float, y: float, z: float) -> None":
This function moves the gripper by the specified amount in meters. The coordinate system
used is the same table coordinate system as in "place_object_on_table_at", with an
additional z-axis pointing upwards.
"""
CONTEXT_EXPLANATION = "\n".join(
    paragraph.replace("\n", " ")
    for paragraph in _CONTEXT_EXPLANATION.strip().split("\n\n")
)


class ExampleInteractionImages(TypedDict):
    """Definition of the image collection of each example interaction."""

    left: str
    right: str
    top: str


class ExampleInteraction(TypedDict):
    """Definition of the example format of the examples.yaml file."""

    images: ExampleInteractionImages
    is_img_required: bool
    prompt: str
    response: str


class VisionMode(enum.Enum):
    """Which images to include in the example LLM request."""

    ALL = "all"
    TOP = "top"
    NONE = "none"

    def __str__(self) -> str:
        """Format the enum variant."""
        return self.value

    def get_example_images(self, example: ExampleInteraction) -> Iterable[str]:
        """Return the image paths of the example pertaining to this vision mode."""
        if self is VisionMode.NONE:
            return []

        if self is VisionMode.TOP:
            return [example["images"]["top"]]

        if self is VisionMode.ALL:
            return cast(Iterable[str], example["images"].values())

        msg = "Unhandled VisionMode."
        raise AssertionError(msg)

    def get_camera_names(self) -> list[str]:
        """Return the corresponding camera names."""
        if self is VisionMode.NONE:
            return []

        if self is VisionMode.TOP:
            return ["zed_top"]

        if self is VisionMode.ALL:
            return ["zed_left", "zed_right", "zed_top"]

        msg = "Unhandled VisionMode."
        raise AssertionError(msg)


def load_examples() -> tuple[dict, pathlib.Path]:
    """Return the example interactions."""
    share_dir = get_package_share_directory("llm_planning")
    examples_dir = pathlib.Path(share_dir) / "example_prompts"

    with (examples_dir / "examples.yaml").open() as f:
        example_collection = yaml.safe_load(f)

    return example_collection, examples_dir
