"""Abstract interface definition for LLM planning backbones."""

import abc
from collections.abc import Iterable

from sensor_msgs.msg import Image

from .common import VisionMode


class LLMInterfaceError(Exception):
    """Exception for the LLM interfaces."""


class LLMInterface(abc.ABC):
    """Abstract interface definition for LLM planning backbones."""

    @abc.abstractmethod
    def __init__(self, examples_vison_mode: VisionMode) -> None:
        """Initialize LLMInterface.

        Args:
            examples_vision_mode: Controls the images send alongside the example
                interactions. This does not affect the chosen model, or which images new
                requests will include.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def send_request(self, instruction: str, images: Iterable[Image]) -> str:
        """Query the LLM given an user instruction and the current camera image."""
        raise NotImplementedError
