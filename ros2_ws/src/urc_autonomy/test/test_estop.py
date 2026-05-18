"""Integration tests for estop_node safety outputs."""

from __future__ import annotations

import json
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String

from urc_autonomy.estop_node import EstopNode

from test_helpers import SPIN_TIMEOUT_S, wait_until


@pytest.fixture
def test_node():
    if not rclpy.ok():
        rclpy.init()

    executor = MultiThreadedExecutor(num_threads=4)
    client = rclpy.create_node(f'test_estop_{uuid.uuid4().hex[:8]}')
    estop = EstopNode()
    executor.add_node(client)
    executor.add_node(estop)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.1)

    yield client

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    executor.remove_node(estop)
    executor.remove_node(client)
    estop.destroy_node()
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def test_node_fast_heartbeat():
    if not rclpy.ok():
        rclpy.init()

    executor = MultiThreadedExecutor(num_threads=4)
    client = rclpy.create_node(f'test_estop_hb_{uuid.uuid4().hex[:8]}')
    estop = EstopNode()
    estop._heartbeat_timeout_s = 1.0
    executor.add_node(client)
    executor.add_node(estop)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.1)

    yield client

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    executor.remove_node(estop)
    executor.remove_node(client)
    estop.destroy_node()
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _estop_subscriber(node):
    values: list[bool] = []
    event = threading.Event()

    def cb(msg: Bool) -> None:
        values.append(msg.data)
        if msg.data:
            event.set()

    sub = node.create_subscription(Bool, '/e_stop', cb, 10)
    return sub, values, event


def _estop_release_subscriber(node):
    values: list[bool] = []
    event = threading.Event()

    def cb(msg: Bool) -> None:
        values.append(msg.data)
        if not msg.data:
            event.set()

    sub = node.create_subscription(Bool, '/e_stop', cb, 10)
    return sub, values, event


def _wait_for_estop_true(event, timeout_s: float) -> None:
    if event.wait(timeout=timeout_s):
        return
    pytest.fail(f'Expected /e_stop True within {timeout_s}s')


def _wait_for_estop_false(event, timeout_s: float) -> None:
    if event.wait(timeout=timeout_s):
        return
    pytest.fail(f'Expected /e_stop False within {timeout_s}s')


def test_manual_trigger_halts(test_node):
    sub, _values, event = _estop_subscriber(test_node)
    pub_trigger = test_node.create_publisher(Bool, '/estop/trigger', 10)

    try:
        pub_trigger.publish(Bool(data=True))
        _wait_for_estop_true(event, timeout_s=1.0)
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_publisher(pub_trigger)


def test_manual_release(test_node):
    sub_true, _values_true, true_event = _estop_subscriber(test_node)
    sub_false, _values_false, false_event = _estop_release_subscriber(test_node)
    pub_trigger = test_node.create_publisher(Bool, '/estop/trigger', 10)

    try:
        pub_trigger.publish(Bool(data=True))
        _wait_for_estop_true(true_event, timeout_s=1.0)

        pub_trigger.publish(Bool(data=False))
        _wait_for_estop_false(false_event, timeout_s=2.0)
    finally:
        test_node.destroy_subscription(sub_true)
        test_node.destroy_subscription(sub_false)
        test_node.destroy_publisher(pub_trigger)


def test_marker_too_close_triggers(test_node):
    sub, _values, event = _estop_subscriber(test_node)
    pub_detection = test_node.create_publisher(String, '/aruco/detection', 10)

    try:
        payload = {'distance_m': 0.3}
        pub_detection.publish(String(data=json.dumps(payload)))
        _wait_for_estop_true(event, timeout_s=1.0)
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_publisher(pub_detection)


def test_heartbeat_loss(test_node_fast_heartbeat):
    sub, values, event = _estop_subscriber(test_node_fast_heartbeat)

    try:
        time.sleep(2.0)

        def estop_active() -> bool:
            return bool(values) and values[-1] is True

        wait_until(
            estop_active,
            timeout_s=SPIN_TIMEOUT_S,
            failure_message=(
                'Expected /e_stop True after mission heartbeat timeout '
                '(no /mission/state for 2s)'
            ),
        )
        assert values[-1] is True
    finally:
        test_node_fast_heartbeat.destroy_subscription(sub)
