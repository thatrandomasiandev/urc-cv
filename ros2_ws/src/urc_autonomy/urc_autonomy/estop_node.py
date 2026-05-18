"""E-stop node: monitors health and publishes /e_stop for twist_mux."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class EstopNode(Node):
    def __init__(self) -> None:
        super().__init__('estop_node')

        self.declare_parameter('heartbeat_timeout_s', 3.0)
        self.declare_parameter('min_marker_distance_m', 0.5)

        self._heartbeat_timeout_s = float(
            self.get_parameter('heartbeat_timeout_s').value
        )
        self._min_marker_distance_m = float(
            self.get_parameter('min_marker_distance_m').value
        )

        self._manual_latched = False
        self._heartbeat_lost = False
        self._marker_too_close = False
        self._estop_active = False

        self._last_heartbeat_monotonic: float | None = None
        self._node_start_monotonic = time.monotonic()
        self._status_tick = 0

        self._sub_trigger = self.create_subscription(
            Bool, '/estop/trigger', self._trigger_cb, 10
        )
        self._sub_mission_state = self.create_subscription(
            String, '/mission/state', self._mission_state_cb, 10
        )
        self._sub_detection = self.create_subscription(
            String, '/aruco/detection', self._detection_cb, 10
        )

        self._pub_estop = self.create_publisher(Bool, '/e_stop', 10)
        self._pub_status = self.create_publisher(String, '/estop/status', 10)

        self._estop_timer = self.create_timer(0.1, self._estop_tick)

        self.get_logger().info('E-stop node started')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _trigger_cb(self, msg: Bool) -> None:
        if msg.data:
            self._manual_latched = True
        else:
            self._manual_latched = False
        self._evaluate_estop()

    def _mission_state_cb(self, msg: String) -> None:
        del msg
        self._last_heartbeat_monotonic = time.monotonic()
        if self._heartbeat_lost:
            self._heartbeat_lost = False
            self._evaluate_estop()

    def _detection_cb(self, msg: String) -> None:
        try:
            det: dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        distance_m = det.get('distance_m')
        if distance_m is None:
            return

        too_close = float(distance_m) < self._min_marker_distance_m
        if too_close and not self._marker_too_close:
            self.get_logger().error(
                f'E-stop: marker closer than {self._min_marker_distance_m}m — stopping'
            )
        self._marker_too_close = too_close
        self._evaluate_estop()

    # ------------------------------------------------------------------
    # Watchdog and publishing
    # ------------------------------------------------------------------

    def _estop_tick(self) -> None:
        self._check_heartbeat()
        self._evaluate_estop()

        self._pub_estop.publish(Bool(data=self._estop_active))

        self._status_tick += 1
        if self._status_tick >= 10:
            self._status_tick = 0
            self._publish_status()

    def _check_heartbeat(self) -> None:
        if self._last_heartbeat_monotonic is None:
            elapsed = time.monotonic() - self._node_start_monotonic
        else:
            elapsed = time.monotonic() - self._last_heartbeat_monotonic

        lost = elapsed > self._heartbeat_timeout_s
        if lost and not self._heartbeat_lost:
            self.get_logger().error('E-stop: mission_node heartbeat lost')
        self._heartbeat_lost = lost

    def _active_reasons(self) -> list[str]:
        reasons: list[str] = []
        if self._manual_latched:
            reasons.append('manual_trigger')
        if self._heartbeat_lost:
            reasons.append('mission_node heartbeat lost')
        if self._marker_too_close:
            reasons.append(f'marker closer than {self._min_marker_distance_m}m')
        return reasons

    def _evaluate_estop(self) -> None:
        active = bool(self._active_reasons())
        if active == self._estop_active:
            return

        self._estop_active = active
        if active:
            self.get_logger().warn(
                f'E-stop ACTIVATED: {", ".join(self._active_reasons())}'
            )
        else:
            self.get_logger().warn('E-stop RELEASED — all triggers cleared')

    def _publish_status(self) -> None:
        payload = {
            'active': self._estop_active,
            'reasons': self._active_reasons(),
        }
        self._pub_status.publish(String(data=json.dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
