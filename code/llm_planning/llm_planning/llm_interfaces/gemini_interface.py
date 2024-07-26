"""Interface to Gemini."""

import os
import pathlib
from collections.abc import Iterable, Iterator
from typing import cast

import google.generativeai as genai
import PIL.Image
import yaml
from ament_index_python import get_package_share_directory
from google.api_core.exceptions import ResourceExhausted
from google.generativeai.types import HarmBlockThreshold as HBThreshold
from google.generativeai.types import HarmCategory, content_types
from google.generativeai.types.content_types import ContentType
from google.generativeai.types.generation_types import (
    BlockedPromptException,
    GenerateContentResponse,
    StopCandidateException,
)
from sensor_msgs.msg import Image

from .common import (
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


def _make_example_history(vision_mode: VisionMode, prompt_version: str) -> list:
    example_collection, examples_dir = load_examples(prompt_version)

    return [
        interaction
        for example in example_collection["examples"]
        for interaction in _make_example_interaction(
            example,
            examples_dir,
            vision_mode,
        )
    ]


class GeminiInterface(LLMInterface):
    """Interface to Gemini."""

    def __init__(self, examples_vision_mode: VisionMode, prompt_version: str) -> None:
        """Create Gemini interface."""
        genai.configure(api_key=os.environ["GEMINI_API_KEY"])
        self._model = genai.GenerativeModel(
            model_name="gemini-1.5-pro-latest",
            system_instruction=self._get_context_explanation(prompt_version),
        )
        self._generation_config = genai.types.GenerationConfig(
            candidate_count=1,
            max_output_tokens=1000,
        )
        self._safety_settings = {
            HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: HBThreshold.BLOCK_ONLY_HIGH,
            HarmCategory.HARM_CATEGORY_HARASSMENT: HBThreshold.BLOCK_ONLY_HIGH,
            HarmCategory.HARM_CATEGORY_HATE_SPEECH: HBThreshold.BLOCK_ONLY_HIGH,
            HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HBThreshold.BLOCK_ONLY_HIGH,
        }

        self._example_history = _make_example_history(examples_vision_mode)
        self._chat = self._model.start_chat(history=self._example_history)

    def _get_context_explanation(self, prompt_version: str) -> str:
        share_dir = get_package_share_directory("llm_planning")
        examples_dir = pathlib.Path(share_dir) / "example_prompts"
        with (examples_dir / f"{prompt_version}_prompts.yaml").open() as stream:
            return yaml.safe_load(stream)["prompt"]

    def _send_message_checked(self, message: ContentType) -> GenerateContentResponse:
        try:
            return self._chat.send_message(
                message,
                generation_config=self._generation_config,
                safety_settings=self._safety_settings,
            )
        except ResourceExhausted as e:
            msg = (
                "Exceeded the rate limit of Gemini 1.5 (5 requests / min)."
                " Please wait a bit before sending another request."
            )
            raise LLMInterfaceError(msg) from e
        except BlockedPromptException as e:
            msg = "The given prompt was flagged as unsafe by Google."
            raise LLMInterfaceError(msg) from e
        except StopCandidateException as e:
            msg = "Gemini stopped generating a response for an unexpected reason."
            raise LLMInterfaceError(msg) from e

    def send_request(self, instruction: str, images: Iterable[Image]) -> str:
        """Query Gemini given an user instruction and current camera images."""
        new_message = {
            "role": "user",
            "parts": [instruction, *(imgmsg_to_pil(img) for img in images)],
        }

        response = self._send_message_checked(new_message)
        response.resolve()
        try:
            return response.text
        except ValueError as e:
            raise LLMInterfaceError from e

    def reset_history(self) -> None:
        """Delete the conversation history but keep the example interactions."""
        self._chat = self._model.start_chat(history=self._example_history)

    def append_error(self, error: str) -> None:
        """Add a retrieved error to the chat history of Gemini."""
        content_list = [
            {
                "role": "user",
                "parts": [
                    f"During the last task the following error was raised: {error}",
                ],
            },
            {
                "role": "model",
                "parts": ["Understood."],
            },
        ]
        self._chat.history.extend(
            content_types.to_contents(
                cast(Iterable[content_types.StrictContentType], content_list),
            ),
        )
