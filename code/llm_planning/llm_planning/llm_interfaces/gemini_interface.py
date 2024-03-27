"""Interface to Gemini."""

import itertools
import os
import pathlib
from collections.abc import Iterable, Iterator

import google.generativeai as genai
import PIL.Image
from google.api_core.exceptions import ResourceExhausted
from sensor_msgs.msg import Image

from .common import (
    CONTEXT_EXPLANATION,
    ExampleInteraction,
    VisionMode,
    load_examples,
)
from .llm_interface import LLMInterface, LLMInterfaceError
from aidara_common.image_utils import imgmsg_to_pil


def _make_example_interaction(
    example: ExampleInteraction,
    examples_dir: pathlib.Path,
    vision_mode: VisionMode,
) -> Iterator[dict]:
    if vision_mode == VisionMode.NONE and example["is_img_required"]:
        return

    images = (
        PIL.Image.open(examples_dir / "images" / image)
        for image in vision_mode.get_example_images(example)
    )

    yield {
        "role": "user",
        "parts": [example["prompt"], *images],
    }
    yield {
        "role": "model",
        "parts": [example["response"]],
    }


def _make_example_history(vision_mode: VisionMode) -> list:
    example_collection, examples_dir = load_examples()

    example_interactions = itertools.chain(
        *(
            _make_example_interaction(
                example,
                examples_dir,
                vision_mode,
            )
            for example in example_collection["examples"]
        ),
    )

    return [
        {"role": "user", "parts": [CONTEXT_EXPLANATION]},
        {
            "role": "model",
            "parts": ["Understood. From now on I will respond with Python code."],
        },
        *example_interactions,
    ]


class GeminiInterface(LLMInterface):
    """Interface to Gemini."""

    def __init__(self, examples_vision_mode: VisionMode) -> None:
        """Create Gemini interface."""
        genai.configure(api_key=os.environ["GEMINI_API_KEY"])
        self._model = genai.GenerativeModel("gemini-1.5-pro-latest")
        self._generation_config = genai.types.GenerationConfig(candidate_count=1)

        self._example_history = _make_example_history(examples_vision_mode)
        self._chat = self._model.start_chat(history=self._example_history)

    def send_request(self, instruction: str, images: Iterable[Image]) -> str:
        """Query Gemini given an user instruction and current camera images."""
        new_message = {
            "role": "user",
            "parts": [instruction, *(imgmsg_to_pil(img) for img in images)],
        }

        try:
            response = self._chat.send_message(
                new_message,
                generation_config=self._generation_config,
            )
        except ResourceExhausted as e:
            msg = (
                "Exceeded the rate limit of Gemini 1.5 (2 requests / min)."
                " Please wait a bit before sending another request."
            )
            raise LLMInterfaceError(msg) from e

        response.resolve()

        return response.text

    def reset_history(self) -> None:
        """Delete the conversation history but keep the example interactions."""
        self._chat = self._model.start_chat(history=self._example_history)
