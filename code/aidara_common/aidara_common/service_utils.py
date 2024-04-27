"""Utilities for calling services."""

import abc
import dataclasses
from typing import TypeVar, cast

from rclpy.client import Client
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future


class AidaraServiceResponse(abc.ABC):
    """Virtual parent class for the result of an aidara service call."""

    @property
    @abc.abstractmethod
    def success(self) -> bool:
        """Indicate whether the service call was successful."""
        raise NotImplementedError


T = TypeVar("T", bound="AsyncServiceCall")


@dataclasses.dataclass
class AsyncServiceCall:
    """Encapsulation of a running service call."""

    _node: Node
    _client: Client
    _request: object
    _initial_future: Future

    def _wait_with_timeout(
        self,
        future: Future,
        timeout: Duration,
    ) -> AidaraServiceResponse:
        rate = self._node.create_rate(10, self._node.get_clock())
        for _ in range(10 * int(timeout.nanoseconds) // S_TO_NS):
            rate.sleep()
            if future.done():
                break
        else:
            msg = (
                f"Service '{self._client.srv_name}' failed to respond within"
                f" {timeout.nanoseconds / S_TO_NS} seconds."
                " It presumably crashed."
            )
            raise RuntimeError(msg)

        return cast(AidaraServiceResponse, future.result())

    @classmethod
    def create(cls: type[T], node: Node, client: Client, request: object) -> T:
        """Start new service call."""
        if not client.wait_for_service(1.0):
            msg = f"Service '{client.srv_name}' is not available."
            raise RuntimeError(msg)

        initial_future = client.call_async(request)

        return cls(node, client, request, initial_future)

    @classmethod
    def create_and_resolve_with_eh(  # noqa: PLR0913.
        cls,
        node: Node,
        client: Client,
        request: object,
        n_retries: int = 3,
        timeout: Duration | None = None,
    ) -> AidaraServiceResponse:
        """Call the service and wait for the result."""
        return cls.create(node, client, request).resolve_with_eh(n_retries, timeout)

    def resolve_with_eh(
        self,
        n_retries: int = 3,
        timeout: Duration | None = None,
    ) -> AidaraServiceResponse:
        """Wait until the service call is done, retrying in case of failure."""
        timeout = timeout or Duration(seconds=30)

        initial_response = self._wait_with_timeout(self._initial_future, timeout)
        if initial_response.success:
            return initial_response

        for _ in range(n_retries):
            future = self._client.call_async(self._request)
            response = self._wait_with_timeout(future, timeout)
            if response.success:
                return response

        msg = (
            f"Service '{self._client.srv_name}' failed to succeed after"
            f" {n_retries} attempts."
        )
        raise RuntimeError(msg)
