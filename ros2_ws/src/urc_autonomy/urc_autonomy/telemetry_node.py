"""Telemetry node: logs rover events to CSV and publishes live JSON summary."""

from __future__ import annotations

import csv
import json
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class TelemetryNode(Node):
    _FLUSH_EVERY_N_EVENTS = 10

    def __init__(self) -> None:
        super().__init__('telemetry_node')

        self.declare_parameter('log_dir', '~/urc_logs')
        log_dir_raw = str(self.get_parameter('log_dir').value)
        log_dir = Path(os.path.expanduser(log_dir_raw))
        log_dir.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = log_dir / f'run_{stamp}.csv'
        self._csv_file: TextIO = open(self._log_path, 'w', newline='', buffering=1)
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['timestamp_s', 'event_type', 'data'])
        self._events_since_flush = 0

        self._run_start_monotonic = time.monotonic()
        self._mission_state = ''
        self._last_logged_mission_state = ''
        self._estop_active = False
        self._last_logged_estop_active: bool | None = None
        self._aruco_detections_total = 0
        self._object_detections_total = 0
        self._gps_msg_count = 0
        self._shutdown_logged = False

        self._sub_mission_state = self.create_subscription(
            String, '/mission/state', self._mission_state_cb, 10
        )
        self._sub_aruco = self.create_subscription(
            String, '/aruco/detection', self._aruco_cb, 10
        )
        self._sub_objects = self.create_subscription(
            String, '/objects/detections', self._objects_cb, 10
        )
        self._sub_estop = self.create_subscription(
            String, '/estop/status', self._estop_cb, 10
        )
        self._sub_fix = self.create_subscription(
            NavSatFix, '/fix', self._fix_cb, 10
        )
        self._sub_search = self.create_subscription(
            String, '/search/status', self._search_cb, 10
        )
        self._sub_approach = self.create_subscription(
            String, '/approach/status', self._approach_cb, 10
        )

        self._pub_summary = self.create_publisher(String, '/telemetry/summary', 10)
        self._summary_timer = self.create_timer(1.0, self._summary_tick)

        self.get_logger().info(f'Telemetry logging to {self._log_path}')

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log_event(self, event_type: str, data: str) -> None:
        timestamp_s = time.monotonic()
        self._csv_writer.writerow([f'{timestamp_s:.4f}', event_type, data])
        self._events_since_flush += 1
        if self._events_since_flush >= self._FLUSH_EVERY_N_EVENTS:
            self._csv_file.flush()
            self._events_since_flush = 0

    def _shutdown(self) -> None:
        if self._shutdown_logged:
            return
        self._shutdown_logged = True
        self._log_event('SHUTDOWN', '')
        self._csv_file.flush()
        self._csv_file.close()

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _mission_state_cb(self, msg: String) -> None:
        state = msg.data
        self._mission_state = state
        if state == self._last_logged_mission_state:
            return
        self._last_logged_mission_state = state
        self._log_event('MISSION_STATE', state)

    def _aruco_cb(self, msg: String) -> None:
        self._aruco_detections_total += 1
        self._log_event('ARUCO_DETECTION', msg.data)

    def _objects_cb(self, msg: String) -> None:
        try:
            detections: list[Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(detections, list) or not detections:
            return
        self._object_detections_total += 1
        self._log_event('OBJECT_DETECTION', msg.data)

    def _estop_cb(self, msg: String) -> None:
        try:
            payload: dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        active = bool(payload.get('active', False))
        self._estop_active = active
        if active == self._last_logged_estop_active:
            return
        self._last_logged_estop_active = active
        self._log_event('ESTOP_STATUS', msg.data)

    def _fix_cb(self, msg: NavSatFix) -> None:
        self._gps_msg_count += 1
        if self._gps_msg_count % 5 != 0:
            return
        payload = {
            'lat': float(msg.latitude),
            'lon': float(msg.longitude),
            'status': int(msg.status.status),
        }
        self._log_event('GPS_FIX', json.dumps(payload))

    def _search_cb(self, msg: String) -> None:
        self._log_event('SEARCH_STATUS', msg.data)

    def _approach_cb(self, msg: String) -> None:
        self._log_event('APPROACH_STATUS', msg.data)

    # ------------------------------------------------------------------
    # Summary publication
    # ------------------------------------------------------------------

    def _summary_tick(self) -> None:
        summary = {
            'run_elapsed_s': round(time.monotonic() - self._run_start_monotonic, 4),
            'mission_state': self._mission_state,
            'aruco_detections_total': self._aruco_detections_total,
            'object_detections_total': self._object_detections_total,
            'estop_active': self._estop_active,
            'log_file': str(self._log_path),
        }
        self._pub_summary.publish(String(data=json.dumps(summary)))

    def destroy_node(self) -> bool:
        self._shutdown()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
