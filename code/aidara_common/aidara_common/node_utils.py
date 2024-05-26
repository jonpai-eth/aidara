"""Utils for node operations."""

from functools import partial

import rerun as rr
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.node import Node

from .async_utils import AsyncServiceCall


class NodeMixin:
    """Mixin class for node utils."""

    def __init__(self) -> None:
        """Initialize NodeMixin."""
        if not isinstance(self, Node):
            msg = "NodeMixin can only be used for classes that are Nodes."
            raise TypeError(msg)

    def sleep(self, duration: Duration) -> None:
        """Sleep for the given duration."""
        rate = self.create_rate(
            S_TO_NS / duration.nanoseconds,
            self.get_clock(),
        )
        rate.sleep()
        rate.destroy()

    def init_rerun(self) -> None:
        """Initialize a connection to rerun."""
        if hasattr(self, "_rerun_timer"):
            return

        rr_recording_id = (
            AsyncServiceCall.create(  # noqa: PD011
                self,
                self.create_client(
                    GetParameters,
                    "/rerun_manager/get_parameters",
                    callback_group=ReentrantCallbackGroup(),
                ),
                GetParameters.Request(names=["recording_id"]),
                has_success_field=False,
            )
            .resolve_with_eh()
            .values[0]
            .string_value
        )

        init_rerun = partial(
            rr.init,
            "aidara",
            recording_id=rr_recording_id,
            default_enabled=True,
            spawn=True,
        )
        init_rerun()

        # make sure we reconnect in case rerun crashes.
        self._rerun_timer = self.create_timer(
            20,
            init_rerun,
        )
