"""URC 2026 mission state machine: GPS waypoints, ArUco search, visual servo."""

from __future__ import annotations

import json
import math
import time
from enum import Enum
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

EARTH_RADIUS_M = 6_371_000.0
MAP_FRAME = 'map'
_NAV_GOAL_SUCCEEDED = 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED


class MissionState(str, Enum):
    IDLE = 'IDLE'
    NAV_TO_WP = 'NAV_TO_WP'
    SEARCHING = 'SEARCHING'
    VISUAL_SERVO = 'VISUAL_SERVO'
    SUCCESS = 'SUCCESS'
    FAILED = 'FAILED'


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_node')

        self.declare_parameter('search_timeout_s', 45.0)
        self.declare_parameter('total_mission_timeout_s', 600.0)
        self.declare_parameter('stop_distance_m', 1.8)

        self._search_timeout_s = float(self.get_parameter('search_timeout_s').value)
        self._total_mission_timeout_s = float(
            self.get_parameter('total_mission_timeout_s').value
        )
        self._stop_distance_m = float(self.get_parameter('stop_distance_m').value)

        self._state = MissionState.IDLE
        self._waypoints: list[dict[str, float]] = []
        self._waypoint_index = 0
        self._mission_start_monotonic: float | None = None
        self._search_start_monotonic: float | None = None
        self._marker_confirmed = False
        self._latest_distance_m: float | None = None

        self._gps_anchor: tuple[float, float] | None = None
        self._nav_goal_handle = None
        self._nav_goal_pending = False
        self._advance_pending = False

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._sub_waypoints = self.create_subscription(
            String,
            '/mission/waypoints',
            self._waypoints_cb,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self._sub_detection = self.create_subscription(
            String,
            '/aruco/detection',
            self._detection_cb,
            10,
        )
        self._sub_fix = self.create_subscription(
            NavSatFix,
            '/fix',
            self._fix_cb,
            10,
        )

        self._pub_state = self.create_publisher(String, '/mission/state', 10)
        self._pub_cmd = self.create_publisher(String, '/mission/cmd', 10)

        self._state_timer = self.create_timer(0.5, self._publish_state)
        self._tick_timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('Mission node started in IDLE')

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _set_state(self, new_state: MissionState) -> None:
        if new_state == self._state:
            return

        old_state = self._state
        self._on_exit_state(old_state)
        self._state = new_state
        self._on_enter_state(new_state)
        self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')

    def _on_enter_state(self, state: MissionState) -> None:
        if state == MissionState.NAV_TO_WP:
            self._marker_confirmed = False
            self._latest_distance_m = None
            self._send_nav_goal()

        elif state == MissionState.SEARCHING:
            self._search_start_monotonic = time.monotonic()
            self._publish_cmd('START_SEARCH')

        elif state == MissionState.VISUAL_SERVO:
            self._publish_cmd('START_SERVO')

        elif state == MissionState.SUCCESS:
            self._cancel_nav_goal()
            self._advance_pending = True

    def _on_exit_state(self, state: MissionState) -> None:
        if state == MissionState.SEARCHING:
            self._search_start_monotonic = None
            self._publish_cmd('STOP_SEARCH')

        elif state == MissionState.VISUAL_SERVO:
            self._publish_cmd('STOP_SERVO')

        elif state == MissionState.NAV_TO_WP:
            self._cancel_nav_goal()

    def _tick(self) -> None:
        if self._advance_pending and self._state == MissionState.SUCCESS:
            self._advance_pending = False
            self._advance_waypoint()
            return

        if self._mission_timed_out():
            self._set_state(MissionState.FAILED)
            return

        if self._state == MissionState.SEARCHING:
            if self._marker_confirmed:
                self._set_state(MissionState.VISUAL_SERVO)
            elif self._search_timed_out():
                self.get_logger().error('Search timed out with no marker detection')
                self._set_state(MissionState.FAILED)

        elif self._state == MissionState.VISUAL_SERVO:
            if (
                self._latest_distance_m is not None
                and self._latest_distance_m < self._stop_distance_m
            ):
                self._set_state(MissionState.SUCCESS)

        elif self._state == MissionState.NAV_TO_WP:
            if (
                self._gps_anchor is not None
                and self._nav_goal_handle is None
                and not self._nav_goal_pending
            ):
                self._send_nav_goal()

    def _advance_waypoint(self) -> None:
        self._waypoint_index += 1
        self._marker_confirmed = False
        self._latest_distance_m = None

        if self._waypoint_index >= len(self._waypoints):
            self.get_logger().info('All waypoints complete — returning to IDLE')
            self._waypoints = []
            self._waypoint_index = 0
            self._mission_start_monotonic = None
            self._set_state(MissionState.IDLE)
        else:
            self.get_logger().info(
                f'Advancing to waypoint {self._waypoint_index + 1}/{len(self._waypoints)}'
            )
            self._set_state(MissionState.NAV_TO_WP)

    # ------------------------------------------------------------------
    # Timeouts
    # ------------------------------------------------------------------

    def _mission_timed_out(self) -> bool:
        if self._mission_start_monotonic is None:
            return False
        if self._state in (MissionState.IDLE, MissionState.FAILED):
            return False
        elapsed = time.monotonic() - self._mission_start_monotonic
        return elapsed > self._total_mission_timeout_s

    def _search_timed_out(self) -> bool:
        if self._search_start_monotonic is None:
            return False
        elapsed = time.monotonic() - self._search_start_monotonic
        return elapsed > self._search_timeout_s

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _waypoints_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid waypoint JSON: {exc}')
            return

        if not isinstance(payload, list):
            self.get_logger().error('Waypoint payload must be a JSON list')
            return

        waypoints: list[dict[str, float]] = []
        for entry in payload:
            if not isinstance(entry, dict) or 'lat' not in entry or 'lon' not in entry:
                self.get_logger().error('Each waypoint must be {"lat": float, "lon": float}')
                return
            waypoints.append({'lat': float(entry['lat']), 'lon': float(entry['lon'])})

        if not waypoints:
            self.get_logger().warn('Received empty waypoint list — ignoring')
            return

        self._waypoints = waypoints
        self._waypoint_index = 0
        self._mission_start_monotonic = time.monotonic()
        self._marker_confirmed = False
        self._latest_distance_m = None

        self.get_logger().info(f'Loaded {len(waypoints)} waypoint(s)')
        # Cancel any in-flight Nav2 goal so _on_exit_state does not need to
        # be relied on (it is skipped when state is already NAV_TO_WP).
        self._cancel_nav_goal()
        self._nav_goal_pending = False
        self._marker_confirmed = False
        self._latest_distance_m = None
        self._set_state(MissionState.NAV_TO_WP)

    def _detection_cb(self, msg: String) -> None:
        try:
            det: dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if det.get('confidence') != 'confirmed':
            return

        distance_m = det.get('distance_m')
        if distance_m is not None:
            self._latest_distance_m = float(distance_m)

        if self._state in (MissionState.NAV_TO_WP, MissionState.SEARCHING):
            if not self._marker_confirmed:
                self._marker_confirmed = True
                dist_str = (
                    f'{self._latest_distance_m:.2f} m'
                    if self._latest_distance_m is not None
                    else 'unknown distance'
                )
                self.get_logger().info(f'Marker {det.get("id")} confirmed at {dist_str}')
                if self._state == MissionState.NAV_TO_WP:
                    self._set_state(MissionState.VISUAL_SERVO)

        elif self._state == MissionState.VISUAL_SERVO:
            pass  # distance updates handled in _tick

    def _fix_cb(self, msg: NavSatFix) -> None:
        if self._gps_anchor is not None:
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        self._gps_anchor = (msg.latitude, msg.longitude)
        self.get_logger().info(
            f'GPS anchor set: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}'
        )
        if (
            self._state == MissionState.NAV_TO_WP
            and self._nav_goal_handle is None
            and not self._nav_goal_pending
        ):
            self._send_nav_goal()

    # ------------------------------------------------------------------
    # Nav2
    # ------------------------------------------------------------------

    def _send_nav_goal(self) -> None:
        if self._waypoint_index >= len(self._waypoints):
            return

        if self._gps_anchor is None:
            self.get_logger().warn('Waiting for GPS anchor on /fix before sending Nav2 goal')
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Nav2 action server not available yet')
            return

        wp = self._waypoints[self._waypoint_index]
        x, y = self._gps_to_map(wp['lat'], wp['lon'])

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _yaw_to_quaternion(0.0)

        self._cancel_nav_goal()

        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_cb,
        )
        send_future.add_done_callback(self._nav_goal_response_cb)
        self._nav_goal_pending = True
        self.get_logger().info(
            f'Nav2 goal sent for waypoint {self._waypoint_index + 1}: '
            f'lat={wp["lat"]:.7f}, lon={wp["lon"]:.7f} -> map ({x:.2f}, {y:.2f})'
        )

    def _nav_goal_response_cb(self, future) -> None:
        self._nav_goal_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected')
            self._set_state(MissionState.FAILED)
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future) -> None:
        if self._state != MissionState.NAV_TO_WP:
            return

        result = future.result()
        status = result.status

        if status == _NAV_GOAL_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded')
            if self._marker_confirmed:
                self._set_state(MissionState.VISUAL_SERVO)
            else:
                self._set_state(MissionState.SEARCHING)
        else:
            self.get_logger().error(f'Nav2 goal failed with status {status}')
            self._set_state(MissionState.FAILED)

        self._nav_goal_handle = None

    def _nav_feedback_cb(self, feedback_msg) -> None:
        del feedback_msg

    def _cancel_nav_goal(self) -> None:
        if self._nav_goal_handle is not None:
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _: self.get_logger().debug('Nav2 goal cancel requested')
            )
            self._nav_goal_handle = None

    # ------------------------------------------------------------------
    # GPS conversion (flat-earth, anchored to first fix)
    # ------------------------------------------------------------------

    def _gps_to_map(self, lat: float, lon: float) -> tuple[float, float]:
        lat0, lon0 = self._gps_anchor
        lat_rad = math.radians(lat0)

        d_lat = math.radians(lat - lat0)
        d_lon = math.radians(lon - lon0)

        x = d_lon * math.cos(lat_rad) * EARTH_RADIUS_M
        y = d_lat * EARTH_RADIUS_M
        return x, y

    # ------------------------------------------------------------------
    # Publications
    # ------------------------------------------------------------------

    def _publish_state(self) -> None:
        self._pub_state.publish(String(data=self._state.value))

    def _publish_cmd(self, cmd: str) -> None:
        self._pub_cmd.publish(String(data=json.dumps({'cmd': cmd})))
        self.get_logger().info(f'Mission command: {cmd}')


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
