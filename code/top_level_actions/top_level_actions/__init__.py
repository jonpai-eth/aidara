"""Top-level actions for the LLM planner."""

import inspect

from .llm_actions_node import LLMActions  # noqa: F401
from top_level_actions import action_collection

ACTION_COLLECTION = dict(inspect.getmembers(action_collection))
