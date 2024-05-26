"""Utilities for dealing with futures."""

import dataclasses
import operator
from collections.abc import Callable
from functools import partial
from typing import Generic, TypeVar

from rclpy.client import Client
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from typing_extensions import Self

T = TypeVar("T")


@dataclasses.dataclass
class FutureResolver(Generic[T]):
    """Utility to resolve futures with timeouts and retries."""

    _node: Node
    _server_name: str
    _initial_future: Future
    _future_factory: Callable[[], Future]
    _get_success: Callable[[T], bool]

    def _wait_with_timeout(
        self,
        future: Future,
        timeout: Duration,
    ) -> T:
        result: T | None = None

        def _get_result(future: Future) -> None:
            nonlocal result
            result = future.result()

        # add_done_callback works more reliably than future.done()
        future.add_done_callback(_get_result)

        rate = self._node.create_rate(10, self._node.get_clock())
        for _ in range(10 * int(timeout.nanoseconds) // S_TO_NS):
            rate.sleep()
            if result is not None:
                break
        else:
            msg = (
                f"'{self._server_name}' failed to respond within"
                f" {timeout.nanoseconds / S_TO_NS} seconds."
                " It presumably crashed."
            )
            raise RuntimeError(msg)

        return result

    @classmethod
    def create(
        cls,
        node: Node,
        server_name: str,
        future_factory: Callable[[], Future],
        get_success: Callable[[T], bool],
    ) -> Self:
        """Start new async call."""
        initial_future = future_factory()

        return cls(node, server_name, initial_future, future_factory, get_success)

    def resolve_with_eh(
        self,
        n_retries: int = 3,
        timeout: Duration | None = None,
    ) -> T:
        """Wait until the service call is done, retrying in case of failure."""
        timeout = timeout or Duration(seconds=30)

        initial_response = self._wait_with_timeout(self._initial_future, timeout)
        if self._get_success(initial_response):
            return initial_response

        for _ in range(n_retries):
            future = self._future_factory()
            response = self._wait_with_timeout(future, timeout)
            if self._get_success(response):
                return response

        msg = f"'{self._server_name}' failed to succeed after {n_retries} attempts."
        raise RuntimeError(msg)


class AsyncServiceCall:
    """Utility to call a service with timeout and retries."""

    @classmethod
    def create(
        cls,
        node: Node,
        client: Client,
        request: object,
        *,
        has_success_field: bool = True,
    ) -> FutureResolver:
        """Start new service call."""
        server_name = f"Service '{client.srv_name}'"

        if not client.wait_for_service(1.0):
            msg = f"'{server_name}' is not available."
            raise RuntimeError(msg)

        return FutureResolver.create(
            node,
            server_name,
            partial(client.call_async, request),
            operator.attrgetter("success") if has_success_field else lambda _: True,
        )
