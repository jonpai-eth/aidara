"""Implementation for the Singleton pattern."""

from typing import Any, ClassVar, TypeVar

T = TypeVar("T")


class Singleton(type):
    """Singleton metaclass."""

    _instances: ClassVar[dict] = {}

    def __call__(cls, *args: tuple, **kwargs: dict[str, Any]) -> object:
        """Create new class instance."""
        if cls not in cls._instances:
            cls._instances[cls] = super().__call__(*args, **kwargs)
        return cls._instances[cls]
