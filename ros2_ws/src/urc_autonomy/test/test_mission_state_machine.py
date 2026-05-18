"""Integration tests for mission_node state transitions."""

from __future__ import annotations

import json
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from urc_autonomy.mission_node import MissionNode

from test_helpers import SPIN_TIMEOUT_S, wait_until

WAYPOINT_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

CONFIRMED_DETECTION = {
    'id': 0,
    'distance_m': 10.0,
    'bearing_deg': 2.5,
    'depth_m': 10.1,
    'depth_validated': True,
    'confidence': 'confirmed',
}


@pytest.fixture
def test_node():
    if not rclpy.ok():
        rclpy.init()

    executor = MultiThreadedExecutor(num_threads=4)
    client = rclpy.create_node(f'test_mission_{uuid.uuid4().hex[:8]}')
    mission = MissionNode()
    executor.add_node(client)
    executor.add_node(mission)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(0.1)

    yield client

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    executor.remove_node(mission)
    executor.remove_node(client)
    mission.destroy_node()
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _state_subscriber(node):
    last_state: list[str] = ['']
    event = threading.Event()

    def cb(msg: String) -> None:
        last_state[0] = msg.data
        event.set()

    sub = node.create_subscription(String, '/mission/state', cb, 10)
    return sub, last_state, event


def _cmd_subscriber(node):
    commands: list[dict] = []
    event = threading.Event()

    def cb(msg: String) -> None:
        try:
            commands.append(json.loads(msg.data))
        except json.JSONDecodeError:
            return
        event.set()

    sub = node.create_subscription(String, '/mission/cmd', cb, 10)
    return sub, commands, event


def _wait_for_state(last_state, event, expected: str, timeout_s: float) -> None:
    def matches() -> bool:
        return last_state[0] == expected

    if matches():
        return

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if matches():
            return
        event.clear()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        event.wait(timeout=min(0.1, remaining))
    pytest.fail(
        f'Expected /mission/state == {expected!r}, got {last_state[0]!r} '
        f'within {timeout_s}s'
    )


def test_idle_on_startup(test_node):
    sub, last_state, event = _state_subscriber(test_node)
    try:
        _wait_for_state(last_state, event, 'IDLE', timeout_s=2.0)
    finally:
        test_node.destroy_subscription(sub)


def test_nav_to_wp_on_waypoints(test_node):
    state_sub, last_state, state_event = _state_subscriber(test_node)
    cmd_sub, commands, _cmd_event = _cmd_subscriber(test_node)
    pub_waypoints = test_node.create_publisher(
        String, '/mission/waypoints', WAYPOINT_QOS
    )

    try:
        pub_waypoints.publish(
            String(data=json.dumps([{'lat': 40.0, 'lon': -111.0}]))
        )
        _wait_for_state(last_state, state_event, 'NAV_TO_WP', timeout_s=2.0)

        start_servo_cmds = [c for c in commands if c.get('cmd') == 'START_SERVO']
        assert not start_servo_cmds, (
            'START_SERVO must not be sent before marker confirmation'
        )
    finally:
        test_node.destroy_subscription(state_sub)
        test_node.destroy_subscription(cmd_sub)
        test_node.destroy_publisher(pub_waypoints)


def test_visual_servo_on_confirmed_detection(test_node):
    state_sub, last_state, state_event = _state_subscriber(test_node)
    cmd_sub, commands, _cmd_event = _cmd_subscriber(test_node)
    pub_waypoints = test_node.create_publisher(
        String, '/mission/waypoints', WAYPOINT_QOS
    )
    pub_detection = test_node.create_publisher(String, '/aruco/detection', 10)

    try:
        pub_waypoints.publish(
            String(data=json.dumps([{'lat': 40.0, 'lon': -111.0}]))
        )
        _wait_for_state(last_state, state_event, 'NAV_TO_WP', timeout_s=2.0)

        pub_detection.publish(String(data=json.dumps(CONFIRMED_DETECTION)))
        _wait_for_state(last_state, state_event, 'VISUAL_SERVO', timeout_s=2.0)

        def saw_start_servo() -> bool:
            return any(c.get('cmd') == 'START_SERVO' for c in commands)

        wait_until(
            saw_start_servo,
            timeout_s=SPIN_TIMEOUT_S,
            failure_message='Expected START_SERVO on /mission/cmd',
        )
    finally:
        test_node.destroy_subscription(state_sub)
        test_node.destroy_subscription(cmd_sub)
        test_node.destroy_publisher(pub_waypoints)
        test_node.destroy_publisher(pub_detection)


def test_success_on_close_distance(test_node):
    state_sub, last_state, state_event = _state_subscriber(test_node)
    pub_waypoints = test_node.create_publisher(
        String, '/mission/waypoints', WAYPOINT_QOS
    )
    pub_detection = test_node.create_publisher(String, '/aruco/detection', 10)

    try:
        pub_waypoints.publish(
            String(data=json.dumps([{'lat': 40.0, 'lon': -111.0}]))
        )
        _wait_for_state(last_state, state_event, 'NAV_TO_WP', timeout_s=2.0)

        pub_detection.publish(String(data=json.dumps(CONFIRMED_DETECTION)))
        _wait_for_state(last_state, state_event, 'VISUAL_SERVO', timeout_s=2.0)

        close_detection = {**CONFIRMED_DETECTION, 'distance_m': 1.5}
        pub_detection.publish(String(data=json.dumps(close_detection)))
        _wait_for_state(last_state, state_event, 'SUCCESS', timeout_s=2.0)
    finally:
        test_node.destroy_subscription(state_sub)
        test_node.destroy_publisher(pub_waypoints)
        test_node.destroy_publisher(pub_detection)


def test_pending_detection_ignored(test_node):
    state_sub, last_state, state_event = _state_subscriber(test_node)
    pub_waypoints = test_node.create_publisher(
        String, '/mission/waypoints', WAYPOINT_QOS
    )
    pub_detection = test_node.create_publisher(String, '/aruco/detection', 10)

    try:
        pub_waypoints.publish(
            String(data=json.dumps([{'lat': 40.0, 'lon': -111.0}]))
        )
        _wait_for_state(last_state, state_event, 'NAV_TO_WP', timeout_s=2.0)

        pending = {**CONFIRMED_DETECTION, 'confidence': 'pending'}
        pub_detection.publish(String(data=json.dumps(pending)))
        time.sleep(1.0)

        assert last_state[0] == 'NAV_TO_WP', (
            f'Pending detection must not change state; got {last_state[0]!r}'
        )
    finally:
        test_node.destroy_subscription(state_sub)
        test_node.destroy_publisher(pub_waypoints)
        test_node.destroy_publisher(pub_detection)
