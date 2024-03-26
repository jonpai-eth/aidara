"""Interface to Gemini."""

import itertools
import os
import pathlib
from collections.abc import Iterable, Iterator

import google.generativeai as genai
import PIL.Image
from sensor_msgs.msg import Image

from .common import (
    CONTEXT_EXPLANATION,
    ExampleInteraction,
    VisionMode,
    load_examples,
)
from .llm_interface import LLMInterface
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


def _make_base_prompt(vision_mode: VisionMode) -> list:
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
        self._model = genai.GenerativeModel("gemini-pro-vision")
        self._generation_config = genai.types.GenerationConfig(candidate_count=1)

        self._base_prompt = _make_base_prompt(examples_vision_mode)

        msg = (
            "Gemini w/ vision does not support multi-turn conversations yet, which is"
            " required for us to show it our examples. So this just doesn't work yet."
        )
        raise NotImplementedError(msg)

    def send_request(self, instruction: str, images: Iterable[Image]) -> str:
        """Query Gemini given an user instruction and current camera images."""
        new_prompt = {
            "role": "user",
            "parts": [instruction, *(imgmsg_to_pil(img) for img in images)],
        }
        prompt = [*self._base_prompt, new_prompt]

        response = self._model.generate_content(
            prompt,
            generation_config=self._generation_config,
        )
        response.resolve()

        return response.text

    def reset_history(self) -> None:
        """Delete the conversation history but keep the example interactions."""
        # Gemini w/ vision does not support multi-turn conversations at all, currently
        # not even send_request will work. So we can't really implement this yet either.
        raise NotImplementedError
