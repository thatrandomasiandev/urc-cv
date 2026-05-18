"""Integration tests for visual_servo servo_node control law."""

from __future__ import annotations

import json
import threading
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from visual_servo.servo_node import ServoNode

from test_helpers import SPIN_TIMEOUT_S, twist_is_zero, wait_until


@pytest.fixture
def test_node():
    if not rclpy.ok():
        rclpy.init()

    executor = MultiThreadedExecutor(num_threads=4)
    client = rclpy.create_node(f'test_servo_{uuid.uuid4().hex[:8]}')
    servo = ServoNode()
    executor.add_node(client)
    executor.add_node(servo)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.1)

    yield client

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    executor.remove_node(servo)
    executor.remove_node(client)
    servo.destroy_node()
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _twist_subscriber(node):
    twists: list[Twist] = []
    event = threading.Event()

    def cb(msg: Twist) -> None:
        twists.append(msg)
        event.set()

    sub = node.create_subscription(Twist, '/cmd_vel_servo', cb, 10)
    return sub, twists, event


def _latest_twist(twists: list[Twist]) -> Twist | None:
    return twists[-1] if twists else None


def _wait_for_twist(
    twists,
    event,
    predicate,
    timeout_s: float,
    failure_message: str,
) -> Twist:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        twist = _latest_twist(twists)
        if twist is not None and predicate(twist):
            return twist
        event.clear()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        event.wait(timeout=min(0.1, remaining))
    pytest.fail(failure_message)


def _publish_detection(pub, bearing_deg: float, distance_m: float) -> None:
    payload = {
        'id': 0,
        'bearing_deg': bearing_deg,
        'distance_m': distance_m,
        'confidence': 'confirmed',
    }
    pub.publish(String(data=json.dumps(payload)))


def _start_servo(pub_cmd) -> None:
    pub_cmd.publish(String(data=json.dumps({'cmd': 'START_SERVO'})))


def test_inactive_publishes_zero(test_node):
    sub, twists, event = _twist_subscriber(test_node)
    try:
        twist = _wait_for_twist(
            twists,
            event,
            lambda t: True,
            timeout_s=1.0,
            failure_message='Expected /cmd_vel_servo message within 1s',
        )
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.linear.y == pytest.approx(0.0)
        assert twist.linear.z == pytest.approx(0.0)
        assert twist.angular.x == pytest.approx(0.0)
        assert twist.angular.y == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)
    finally:
        test_node.destroy_subscription(sub)


def test_activates_on_start_servo(test_node):
    sub, twists, event = _twist_subscriber(test_node)
    pub_cmd = test_node.create_publisher(String, '/mission/cmd', 10)
    pub_det = test_node.create_publisher(String, '/aruco/detection', 10)

    try:
        _start_servo(pub_cmd)
        _publish_detection(pub_det, bearing_deg=10.0, distance_m=5.0)

        twist = _wait_for_twist(
            twists,
            event,
            lambda t: abs(t.angular.z) > 1e-6,
            timeout_s=1.0,
            failure_message='Expected non-zero angular.z within 1s of activation',
        )
        assert twist.angular.z != pytest.approx(0.0)
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_publisher(pub_cmd)
        test_node.destroy_publisher(pub_det)


def test_bearing_sign(test_node):
    sub, twists, event = _twist_subscriber(test_node)
    pub_cmd = test_node.create_publisher(String, '/mission/cmd', 10)
    pub_det = test_node.create_publisher(String, '/aruco/detection', 10)

    try:
        _start_servo(pub_cmd)
        _publish_detection(pub_det, bearing_deg=15.0, distance_m=5.0)

        twist = _wait_for_twist(
            twists,
            event,
            lambda t: abs(t.angular.z) > 1e-6,
            timeout_s=1.0,
            failure_message='Expected non-zero angular.z for +15 deg bearing',
        )
        assert twist.angular.z < 0.0, (
            'Positive bearing (marker right) must yield negative angular.z (turn right)'
        )
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_publisher(pub_cmd)
        test_node.destroy_publisher(pub_det)


def test_stops_at_distance(test_node):
    sub, twists, event = _twist_subscriber(test_node)
    pub_cmd = test_node.create_publisher(String, '/mission/cmd', 10)
    pub_det = test_node.create_publisher(String, '/aruco/detection', 10)

    try:
        _start_servo(pub_cmd)
        _publish_detection(pub_det, bearing_deg=0.0, distance_m=1.5)

        _wait_for_twist(
            twists,
            event,
            lambda t: True,
            timeout_s=1.0,
            failure_message='Expected /cmd_vel_servo message within 1s',
        )

        def linear_stopped() -> bool:
            twist = _latest_twist(twists)
            return twist is not None and abs(twist.linear.x) < 1e-6

        wait_until(
            linear_stopped,
            timeout_s=SPIN_TIMEOUT_S,
            failure_message='Expected linear.x == 0 below stop_distance_m',
        )
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_publisher(pub_cmd)
        test_node.destroy_publisher(pub_det)


def test_zero_on_stale_detection(test_node):
    sub, twists, event = _twist_subscriber(test_node)
    pub_cmd = test_node.create_publisher(String, '/mission/cmd', 10)
    pub_det = test_node.create_publisher(String, '/aruco/detection', 10)

    detection_timeout_s = 1.5
    stale_wait_s = detection_timeout_s + 0.5

    try:
        _start_servo(pub_cmd)
        _publish_detection(pub_det, bearing_deg=0.0, distance_m=5.0)

        _wait_for_twist(
            twists,
            event,
            lambda t: abs(t.angular.z) > 1e-6 or abs(t.linear.x) > 1e-6,
            timeout_s=1.0,
            failure_message='Expected active servo output before going stale',
        )

        time.sleep(stale_wait_s)

        def all_zero() -> bool:
            twist = _latest_twist(twists)
            return twist is not None and twist_is_zero(twist)

        wait_until(
            all_zero,
            timeout_s=SPIN_TIMEOUT_S,
            failure_message='Expected zero cmd_vel after detection timeout',
        )
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_publisher(pub_cmd)
        test_node.destroy_publisher(pub_det)
