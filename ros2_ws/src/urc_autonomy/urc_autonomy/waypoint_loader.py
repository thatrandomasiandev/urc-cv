"""ROS2 CLI: publish mission waypoints to /mission/waypoints once, then exit."""

from __future__ import annotations

import json
import sys
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _normalize_waypoints(raw: str) -> list[dict[str, float]]:
    try:
        data: Any = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f'Invalid JSON for waypoints parameter: {exc}') from exc

    if not isinstance(data, list):
        raise ValueError('waypoints must be a JSON array')

    normalized: list[dict[str, float]] = []
    for index, entry in enumerate(data):
        if isinstance(entry, (list, tuple)) and len(entry) == 2:
            lat, lon = entry[0], entry[1]
        elif isinstance(entry, dict) and 'lat' in entry and 'lon' in entry:
            lat, lon = entry['lat'], entry['lon']
        else:
            raise ValueError(
                f'waypoint {index}: expected [lat, lon] or '
                '{"lat": float, "lon": float}'
            )

        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'waypoint {index}: lat and lon must be numeric') from exc

        if not (-90.0 <= lat_f <= 90.0):
            raise ValueError(f'waypoint {index}: lat {lat_f} out of range [-90, 90]')
        if not (-180.0 <= lon_f <= 180.0):
            raise ValueError(f'waypoint {index}: lon {lon_f} out of range [-180, 180]')

        normalized.append({'lat': lat_f, 'lon': lon_f})

    return normalized


class WaypointLoader(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_loader')
        self.declare_parameter('waypoints', '[]')

    def run(self) -> int:
        raw = self.get_parameter('waypoints').get_parameter_value().string_value

        try:
            waypoints = _normalize_waypoints(raw)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return 1

        if not waypoints:
            self.get_logger().warn('Empty waypoint list — nothing published')
            return 0

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        publisher = self.create_publisher(String, '/mission/waypoints', qos)

        # Allow discovery before the one-shot publish.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        message = String()
        message.data = json.dumps(waypoints)
        publisher.publish(message)

        self.get_logger().info(
            f'Published {len(waypoints)} waypoint(s) to /mission/waypoints'
        )

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        return 0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaypointLoader()
    exit_code = 0
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
