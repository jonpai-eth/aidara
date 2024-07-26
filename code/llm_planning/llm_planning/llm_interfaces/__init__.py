"""Interfaces to LLMs."""

from typing import Literal

from .common import VisionMode
from .gemini_interface import GeminiInterface
from .gpt_4_interface import GPT4PInterface
from .llm_interface import LLMInterface, LLMInterfaceError  # noqa: F401


def get_interface(
    llm_type: Literal["gemini", "gpt-4"],
    examples_vision_mode: VisionMode,
    prompt_version: str,
) -> LLMInterface:
    """Instantiate the interface to the requested LLM."""
    if llm_type == "gemini":
        return GeminiInterface(examples_vision_mode, prompt_version)

    if llm_type == "gpt-4":
        return GPT4PInterface(examples_vision_mode, prompt_version)

    # This should not be possible given the constraint in the argparser.
    msg = f"Invalid llm type: '{llm_type}'."
    raise AssertionError(msg)
