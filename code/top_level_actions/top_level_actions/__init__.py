"""Top-level actions for the LLM planner."""


def say(message: str) -> None:
    """Relay a message to the user."""
    print(f"say({message=})")  # noqa: T201


def pick_object_from_table(object_name: str) -> None:
    """Pick an object from the table."""
    print(f"pick_object_from_table({object_name=})")  # noqa: T201


def place_object_on_table_at(x: float, y: float) -> None:
    """Place on object on the table."""
    print(f"place_object_on_table_at({x=}, {y=})")  # noqa: T201


def take_from_hand() -> None:
    """Take an object from the user's hand."""
    print("take_from_hand()")  # noqa: T201


def give_to_hand() -> None:
    """Give an object to the user's hand."""
    print("give_to_hand()")  # noqa: T201


def move_gripper_by(x: float, y: float, z: float) -> None:
    """Move the gripper by some delta position."""
    print(f"move_gripper_by({x=}, {y=}, {z=})")  # noqa: T201
