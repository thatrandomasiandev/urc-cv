"""Shared helpers for urc_autonomy integration tests."""

from __future__ import annotations

import time
from collections.abc import Callable

import pytest

SPIN_TIMEOUT_S = 5.0


def wait_until(
    predicate: Callable[[], bool],
    timeout_s: float = SPIN_TIMEOUT_S,
    poll_s: float = 0.05,
    failure_message: str = 'Condition not met before timeout',
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(poll_s)
    pytest.fail(failure_message)


def twist_is_zero(twist) -> bool:
    return (
        abs(twist.linear.x) < 1e-6
        and abs(twist.linear.y) < 1e-6
        and abs(twist.linear.z) < 1e-6
        and abs(twist.angular.x) < 1e-6
        and abs(twist.angular.y) < 1e-6
        and abs(twist.angular.z) < 1e-6
    )
