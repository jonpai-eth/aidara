"""Implementation for the Singleton pattern."""

from typing import Any, ClassVar, TypeVar

T = TypeVar("T")


class Singleton(type):
    """Singleton metaclass."""

    _instances: ClassVar[dict[type[T], T]] = {}

    def __call__(cls: type[T], *args: tuple, **kwargs: dict[str, Any]) -> T:
        """Create new class instance."""
        if cls not in cls._instances:
            cls._instances[cls] = super().__call__(*args, **kwargs)
        return cls._instances[cls]
