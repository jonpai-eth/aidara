"""Interface to GPT-4."""

import itertools
import pathlib
from collections.abc import Iterable, Iterator
from typing import TYPE_CHECKING

from openai import OpenAI
from sensor_msgs.msg import Image

from .common import (
    CONTEXT_EXPLANATION,
    VisionMode,
    load_examples,
)
from .llm_interface import LLMInterface, LLMInterfaceError
from aidara_common.image_utils import imgmsg_to_base64, jpg_to_base64

if TYPE_CHECKING:
    from openai.types.chat.chat_completion import Choice


def _make_img_message(image_encoding: str) -> dict:
    """Create OpenAI message containing an image."""
    return {
        "type": "image_url",
        "image_url": {
            "url": f"data:image/jpeg;base64,{image_encoding}",
            "detail": "high",
        },
    }


def _make_user_message(
    instruction: str,
    image_encodings: Iterable[str],
) -> list[dict]:
    instruction_message = {
        "type": "text",
        "text": instruction,
    }
    image_messages = (
        _make_img_message(image_encoding) for image_encoding in image_encodings
    )

    return [instruction_message, *image_messages]


def _make_example_interaction(
    example: dict,
    examples_dir: pathlib.Path,
    vision_mode: VisionMode,
) -> Iterator[dict]:
    if vision_mode == VisionMode.NONE and example["is_img_required"]:
        return

    images = (
        jpg_to_base64(examples_dir / "images" / image)
        for image in vision_mode.get_example_images(example)
    )
    user_message = _make_user_message(
        example["prompt"],
        images,
    )

    yield {
        "role": "user",
        "content": user_message,
    }
    yield {
        "role": "assistant",
        "content": example["response"],
    }


def _make_base_prompt(vision_mode: VisionMode) -> dict:
    example_collection, examples_dir = load_examples()

    example_interactions = itertools.chain(
        *(
            _make_example_interaction(example, examples_dir, vision_mode)
            for example in example_collection["examples"]
        ),
    )

    return {
        "model": "gpt-4-vision-preview",
        "messages": [
            {
                "role": "system",
                "content": CONTEXT_EXPLANATION,
            },
            *example_interactions,
        ],
    }


class GPT4PInterface(LLMInterface):
    """Interface to GPT-4."""

    def __init__(self, examples_vision_mode: VisionMode) -> None:
        """Create GPT-4 interface."""
        self._client = OpenAI()
        self._base_prompt = _make_base_prompt(examples_vision_mode)

    def send_request(self, instruction: str, images: Iterable[Image]) -> str:
        """Query GPT-4 given an user instruction and current camera images."""
        new_message = _make_user_message(
            instruction,
            (imgmsg_to_base64(img) for img in images),
        )

        prompt = self._base_prompt.copy()
        prompt["messages"].append({"role": "user", "content": new_message})

        response: Choice = self._client.chat.completions.create(**prompt).choices[0]

        if response.finish_reason != "stop":
            msg = (
                f"Unexpected finish reason: '{response.finish_reason}'\n."
                f"Response was:\n{response.message.content}."
            )
            raise LLMInterfaceError(msg)

        if response.message.content is None:
            msg = "Empty response from GPT-4."
            raise LLMInterfaceError(msg)

        return response.message.content
