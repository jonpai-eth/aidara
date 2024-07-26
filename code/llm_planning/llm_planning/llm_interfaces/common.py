"""Abstract interface definition for LLM planning backbones."""

import enum
import pathlib
from collections.abc import Iterable
from typing import TypedDict

import yaml
from ament_index_python import get_package_share_directory


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
    TOP_AND_LEFT = "top_and_left"
    TOP_AND_RIGHT = "top_and_right"
    NONE = "none"

    def __str__(self) -> str:
        """Format the enum variant."""
        return self.value

    @classmethod
    def get_valid_options(cls) -> list[str]:
        """Return all valid vision modes."""
        return [option.value for option in cls.__members__.values()]

    def get_example_images(self, example: ExampleInteraction) -> Iterable[str]:
        """Return the image paths of the example pertaining to this vision mode."""
        if self is VisionMode.NONE:
            return

        yield example["images"]["top"]

        if self is VisionMode.TOP_AND_LEFT or self is VisionMode.ALL:
            yield example["images"]["left"]

        if self is VisionMode.TOP_AND_RIGHT or self is VisionMode.ALL:
            yield example["images"]["right"]

    def get_camera_names(self) -> Iterable[str]:
        """Return the corresponding camera names."""
        if self is VisionMode.NONE:
            return

        yield "zed_top"

        if self is VisionMode.TOP_AND_LEFT or self is VisionMode.ALL:
            yield "zed_left"

        if self is VisionMode.TOP_AND_RIGHT or self is VisionMode.ALL:
            yield "zed_right"


def load_examples(prompt_version: str) -> tuple[dict, pathlib.Path]:
    """Return the example interactions."""
    share_dir = get_package_share_directory("llm_planning")
    examples_dir = pathlib.Path(share_dir) / "example_prompts"

    with (examples_dir / f"{prompt_version}_prompts.yaml").open() as f:
        example_collection = yaml.safe_load(f)

    return example_collection, examples_dir
