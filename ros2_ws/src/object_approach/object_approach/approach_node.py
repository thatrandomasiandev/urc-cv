"""Visual approach controller driven by YOLO object detections for URC 2026."""

import json

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

CONTROL_RATE_HZ = 20.0
FAR_DISTANCE_M = 5.0
NEAR_RAMP_START_M = 2.5
NEAR_CRUISE_SPEED_MPS = 0.10
RAMP_END_SPEED_MPS = 0.15
ALIGN_BEARING_LIMIT_DEG = 20.0


class ApproachNode(Node):
    def __init__(self) -> None:
        super().__init__('approach_node')

        self.declare_parameter('target_class', 'mallet')
        self.declare_parameter('kp_bearing', 0.03)
        self.declare_parameter('ki_bearing', 0.0005)
        self.declare_parameter('kd_bearing', 0.008)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('stop_distance_m', 0.8)
        self.declare_parameter('bearing_deadband_deg', 2.0)
        self.declare_parameter('detection_timeout_s', 2.0)
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('obstacle_check_enabled', True)
        self.declare_parameter('obstacle_halt_distance_m', 0.6)
        self.declare_parameter('obstacle_cone_width_frac', 0.25)

        self._target_class = str(self.get_parameter('target_class').value)
        self._bridge = CvBridge()
        self._depth_image: np.ndarray | None = None
        self._obstacle_warned = False

        self._active = False
        self._latest_detection: dict | None = None
        self._last_detection_time = None
        self._detection_stale_warned = False
        self._lost_reported = False
        self._success_reported = False

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_pid_time = None

        self._pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_approach', 10)
        self._pub_status = self.create_publisher(String, '/approach/status', 10)
        self.create_subscription(String, '/objects/detections', self._detection_cb, 10)
        self.create_subscription(String, '/mission/cmd', self._mission_cb, 10)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10,
        )
        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_timer_cb)

        self.get_logger().info(
            'Object approach node started (inactive until START_APPROACH).',
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _kp(self) -> float:
        return float(self.get_parameter('kp_bearing').value)

    def _ki(self) -> float:
        return float(self.get_parameter('ki_bearing').value)

    def _kd(self) -> float:
        return float(self.get_parameter('kd_bearing').value)

    def _max_angular(self) -> float:
        return float(self.get_parameter('max_angular').value)

    def _max_linear(self) -> float:
        return float(self.get_parameter('max_linear').value)

    def _stop_distance_m(self) -> float:
        return float(self.get_parameter('stop_distance_m').value)

    def _bearing_deadband_deg(self) -> float:
        return float(self.get_parameter('bearing_deadband_deg').value)

    def _detection_timeout_s(self) -> float:
        return float(self.get_parameter('detection_timeout_s').value)

    def _min_confidence(self) -> float:
        return float(self.get_parameter('min_confidence').value)

    def _obstacle_check_enabled(self) -> bool:
        return bool(self.get_parameter('obstacle_check_enabled').value)

    def _obstacle_halt_distance_m(self) -> float:
        return float(self.get_parameter('obstacle_halt_distance_m').value)

    def _obstacle_cone_width_frac(self) -> float:
        return float(self.get_parameter('obstacle_cone_width_frac').value)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _detection_cb(self, msg: String) -> None:
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON on /objects/detections')
            return

        if not isinstance(detections, list):
            self.get_logger().warn('Expected JSON array on /objects/detections')
            return

        best: dict | None = None
        best_conf = -1.0
        min_conf = self._min_confidence()

        for det in detections:
            if det.get('class_name') != self._target_class:
                continue
            if 'bearing_deg' not in det or det.get('distance_m') is None:
                continue

            conf = float(det.get('confidence', 0.0))
            if conf < min_conf:
                continue

            if conf > best_conf:
                best_conf = conf
                best = det

        if best is None:
            return

        self._latest_detection = best
        self._last_detection_time = self.get_clock().now()
        self._detection_stale_warned = False
        self._lost_reported = False

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='16UC1',
            )
        except Exception as exc:
            self.get_logger().warn(f'Depth image conversion failed: {exc}')

    def _mission_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON on /mission/cmd')
            return

        cmd = data.get('cmd', '')
        if cmd == 'START_APPROACH':
            target = data.get('target')
            if target:
                self._target_class = str(target)
            else:
                self._target_class = str(self.get_parameter('target_class').value)

            self._active = True
            self._latest_detection = None
            self._last_detection_time = None
            self._detection_stale_warned = False
            self._lost_reported = False
            self._success_reported = False
            self._reset_pid()
            self.get_logger().info(
                f'Object approach activated — target: {self._target_class}.',
            )
        elif cmd == 'STOP_APPROACH':
            self._active = False
            self._reset_pid()
            self._pub_cmd_vel.publish(Twist())
            self.get_logger().info('Object approach deactivated.')

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status(self, status: str, distance_m: float | None = None) -> None:
        payload: dict = {
            'status': status,
            'class_name': self._target_class,
        }
        if distance_m is not None:
            payload['distance_m'] = round(distance_m, 3)
        self._pub_status.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------
    # Control loop (20 Hz)
    # ------------------------------------------------------------------

    def _control_timer_cb(self) -> None:
        twist = Twist()

        if not self._active:
            self._pub_cmd_vel.publish(twist)
            return

        if self._detection_is_stale():
            if not self._detection_stale_warned:
                self.get_logger().warn(
                    f'No {self._target_class} detection for >'
                    f'{self._detection_timeout_s():.1f}s — publishing zero velocity.',
                )
                self._detection_stale_warned = True
            if not self._lost_reported:
                self._publish_status('LOST')
                self._lost_reported = True
            self._pub_cmd_vel.publish(twist)
            return

        bearing_deg = float(self._latest_detection['bearing_deg'])
        distance_m = float(self._latest_detection['distance_m'])

        if distance_m < self._stop_distance_m():
            if not self._success_reported:
                self._publish_status('SUCCESS', distance_m)
                self._success_reported = True
                self.get_logger().info(
                    f'Approach success — {self._target_class} at {distance_m:.2f} m.',
                )
            self._pub_cmd_vel.publish(twist)
            return

        twist.angular.z = self._compute_angular(bearing_deg)
        twist.linear.x = self._compute_linear(bearing_deg, distance_m)
        self._pub_cmd_vel.publish(twist)

    def _detection_is_stale(self) -> bool:
        if self._last_detection_time is None:
            return True
        age_s = (self.get_clock().now() - self._last_detection_time).nanoseconds * 1e-9
        return age_s > self._detection_timeout_s()

    # ------------------------------------------------------------------
    # PID (bearing → angular.z)
    # ------------------------------------------------------------------

    def _reset_pid(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_pid_time = None

    def _compute_angular(self, bearing_deg: float) -> float:
        if abs(bearing_deg) < self._bearing_deadband_deg():
            self._prev_error = 0.0
            self._prev_pid_time = None
            return 0.0

        now = self.get_clock().now()
        if self._prev_pid_time is None:
            dt = 1.0 / CONTROL_RATE_HZ
        else:
            dt = (now - self._prev_pid_time).nanoseconds * 1e-9
            if dt <= 0.0:
                dt = 1.0 / CONTROL_RATE_HZ

        self._prev_pid_time = now

        error = -bearing_deg
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        p_term = self._kp() * error
        d_term = self._kd() * derivative
        i_term = self._ki() * self._integral

        output_unclamped = p_term + i_term + d_term
        max_ang = self._max_angular()
        output = max(-max_ang, min(max_ang, output_unclamped))

        if output == output_unclamped:
            self._integral += error * dt
        elif (output > 0.0 and error < 0.0) or (output < 0.0 and error > 0.0):
            self._integral += error * dt

        if self._ki() > 0.0:
            i_limit = max_ang / self._ki()
            self._integral = max(-i_limit, min(i_limit, self._integral))

        return output

    # ------------------------------------------------------------------
    # Forward obstacle check (depth camera)
    # ------------------------------------------------------------------

    def _obstacle_in_path(self) -> bool:
        if not self._obstacle_check_enabled() or self._depth_image is None:
            return False

        img = self._depth_image
        h, w = img.shape[:2]

        row_start = h // 3
        row_end = 2 * h // 3
        strip_half_width = int(self._obstacle_cone_width_frac() * w)
        col_center = w // 2
        col_start = max(0, col_center - strip_half_width)
        col_end = min(w, col_center + strip_half_width)

        region = img[row_start:row_end, col_start:col_end]
        valid = region[region > 0]
        if valid.size == 0:
            return False

        depths_m = valid.astype(np.float64) / 1000.0
        return bool(np.any(depths_m < self._obstacle_halt_distance_m()))

    # ------------------------------------------------------------------
    # Linear velocity profile
    # ------------------------------------------------------------------

    def _compute_linear(self, bearing_deg: float, distance_m: float) -> float:
        if abs(bearing_deg) > ALIGN_BEARING_LIMIT_DEG:
            return 0.0

        if distance_m < self._stop_distance_m():
            return 0.0

        if distance_m < NEAR_RAMP_START_M:
            speed = NEAR_CRUISE_SPEED_MPS
        else:
            far_speed = self._max_linear()
            if distance_m > FAR_DISTANCE_M:
                speed = far_speed
            else:
                t = (distance_m - NEAR_RAMP_START_M) / (
                    FAR_DISTANCE_M - NEAR_RAMP_START_M
                )
                speed = RAMP_END_SPEED_MPS + t * (far_speed - RAMP_END_SPEED_MPS)

        if self._obstacle_in_path():
            if not self._obstacle_warned:
                self.get_logger().warn(
                    'Obstacle detected in path — halting linear velocity.',
                )
                self._obstacle_warned = True
            return 0.0
        self._obstacle_warned = False
        return speed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
