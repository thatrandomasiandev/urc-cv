"""Timer-driven search behavior for ArUco markers at GPS waypoints."""

import json
import math
from enum import IntEnum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

ROTATE_DURATION_S = 3.0
ARC_SWEEP_RAD = math.pi  # 180 degrees
CONTROL_PERIOD_S = 0.05  # 20 Hz


class SearchPhase(IntEnum):
    INACTIVE = 0
    ROTATE = 1
    ARC_CCW = 2
    ARC_CW = 3


class SearchBehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__('search_behavior')

        self.declare_parameter('rotate_speed_rad', 0.3)
        self.declare_parameter('arc_radius_m', 4.0)
        self.declare_parameter('arc_omega_rad', 0.12)
        self.declare_parameter('search_timeout_s', 45.0)

        self._phase = SearchPhase.INACTIVE
        self._phase_start: Time | None = None
        self._search_start: Time | None = None
        self._arc_angle_rad = 0.0
        self._last_tick: Time | None = None

        self._sub_detection = self.create_subscription(
            String,
            '/aruco/detection',
            self._detection_cb,
            10,
        )
        self._sub_mission = self.create_subscription(
            String,
            '/mission/cmd',
            self._mission_cmd_cb,
            10,
        )
        self._pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_search', 10)
        self._pub_status = self.create_publisher(String, '/search/status', 10)

        self._timer = self.create_timer(CONTROL_PERIOD_S, self._tick)

        self.get_logger().info('Search behavior node ready (inactive).')

    # ------------------------------------------------------------------
    # Parameters (refreshed each search start)
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        self._rotate_speed = float(self.get_parameter('rotate_speed_rad').value)
        self._arc_radius = float(self.get_parameter('arc_radius_m').value)
        self._arc_omega = float(self.get_parameter('arc_omega_rad').value)
        self._search_timeout = float(self.get_parameter('search_timeout_s').value)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _detection_cb(self, msg: String) -> None:
        if self._phase == SearchPhase.INACTIVE:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not payload:
            return
        if payload.get('confidence') != 'confirmed':
            return
        self._on_marker_found()

    def _mission_cmd_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON on /mission/cmd')
            return

        cmd = payload.get('cmd', '')
        if cmd == 'START_SEARCH':
            self._start_search()
        elif cmd == 'STOP_SEARCH':
            self._stop_search()

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _start_search(self) -> None:
        if self._phase != SearchPhase.INACTIVE:
            self.get_logger().warn('START_SEARCH ignored — search already active.')
            return

        self._load_params()
        now = self.get_clock().now()
        self._phase = SearchPhase.ROTATE
        self._phase_start = now
        self._search_start = now
        self._last_tick = now
        self._arc_angle_rad = 0.0
        self.get_logger().info('Search started — Phase 1: in-place rotate (3 s).')

    def _stop_search(self) -> None:
        if self._phase == SearchPhase.INACTIVE:
            return
        self.get_logger().info('Search stopped by mission command.')
        self._halt_and_reset()

    def _on_marker_found(self) -> None:
        phase = int(self._phase)
        self.get_logger().info(f'ArUco marker found during phase {phase}.')
        self._publish_status('FOUND', phase=phase)
        self._halt_and_reset()

    def _on_search_failed(self) -> None:
        self.get_logger().info('Search complete — no marker detected (Phase 4: FAILED).')
        self._publish_status('FAILED')
        self._halt_and_reset()

    def _on_timeout(self) -> None:
        self.get_logger().warn(
            f'Search timed out after {self._search_timeout:.1f} s.',
        )
        self._publish_status('FAILED')
        self._halt_and_reset()

    def _advance_phase(self) -> None:
        now = self.get_clock().now()
        self._phase_start = now
        self._arc_angle_rad = 0.0
        self._last_tick = now

        if self._phase == SearchPhase.ROTATE:
            self._phase = SearchPhase.ARC_CCW
            self.get_logger().info('Phase 2: counterclockwise arc sweep (180°).')
        elif self._phase == SearchPhase.ARC_CCW:
            self._phase = SearchPhase.ARC_CW
            self.get_logger().info('Phase 3: clockwise reverse arc (180°).')
        elif self._phase == SearchPhase.ARC_CW:
            self._on_search_failed()

    def _halt_and_reset(self) -> None:
        self._publish_zero_vel()
        self._phase = SearchPhase.INACTIVE
        self._phase_start = None
        self._search_start = None
        self._last_tick = None
        self._arc_angle_rad = 0.0

    # ------------------------------------------------------------------
    # Timer-driven control loop
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._phase == SearchPhase.INACTIVE:
            return

        now = self.get_clock().now()
        if self._last_tick is None or self._search_start is None or self._phase_start is None:
            self._last_tick = now
            return

        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now

        search_elapsed = (now - self._search_start).nanoseconds * 1e-9
        if search_elapsed >= self._search_timeout:
            self._on_timeout()
            return

        phase_elapsed = (now - self._phase_start).nanoseconds * 1e-9
        twist = Twist()

        if self._phase == SearchPhase.ROTATE:
            twist.angular.z = self._rotate_speed
            if phase_elapsed >= ROTATE_DURATION_S:
                self._advance_phase()
                self._pub_cmd_vel.publish(twist)
                return

        elif self._phase == SearchPhase.ARC_CCW:
            omega = self._arc_omega
            twist.linear.x = self._arc_radius * omega
            twist.angular.z = omega
            self._arc_angle_rad += abs(omega) * dt
            if self._arc_angle_rad >= ARC_SWEEP_RAD:
                self._advance_phase()
                self._pub_cmd_vel.publish(twist)
                return

        elif self._phase == SearchPhase.ARC_CW:
            omega = self._arc_omega
            twist.linear.x = self._arc_radius * omega
            twist.angular.z = -omega
            self._arc_angle_rad += abs(omega) * dt
            if self._arc_angle_rad >= ARC_SWEEP_RAD:
                self._advance_phase()
                self._pub_cmd_vel.publish(twist)
                return

        self._pub_cmd_vel.publish(twist)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_zero_vel(self) -> None:
        self._pub_cmd_vel.publish(Twist())

    def _publish_status(self, status: str, phase: int | None = None) -> None:
        payload: dict[str, object] = {'status': status}
        if phase is not None:
            payload['phase'] = phase
        self._pub_status.publish(String(data=json.dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SearchBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._halt_and_reset()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
